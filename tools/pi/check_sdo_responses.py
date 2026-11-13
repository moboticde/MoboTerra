"""Offline request/reply matching for parallel CAN/ROS recorder output."""
import collections
import json
from pathlib import Path
import re
import statistics
import sys


def percentiles(values):
    if not values:
        return None
    ordered = sorted(values)
    return {'n': len(ordered), 'min_ms': ordered[0], 'median_ms': statistics.median(ordered),
            'p95_ms': ordered[int(.95 * (len(ordered)-1))],
            'p99_ms': ordered[int(.99 * (len(ordered)-1))], 'max_ms': ordered[-1]}


def analyze(root):
    source = [json.loads(line) for line in (root/'ros_can_tx.jsonl').read_text().splitlines()]
    requests = []
    for item in source:
        if item.get('extended') or item.get('rtr') or item.get('error'):
            continue
        if not (0x601 <= item['id'] <= 0x608 or item['id'] == 0x60a):
            continue
        data = bytes.fromhex(item['data'])
        if len(data) != 8 or data[0] not in (0x40, 0x23, 0x27, 0x2b, 0x2f):
            continue
        requests.append({'t_ns':item['source_ns'], 'node':item['id']-0x600,
            'object':f'{int.from_bytes(data[1:3], "little"):04X}:{data[3]}',
            'read':data[0] == 0x40, 'data':item['data'], 'reply':None})
    replies = []
    faults = []
    first_rx = last_rx = None
    for line in (root/'can_rx.log').read_text().splitlines():
        match = re.match(r'\(([0-9.]+)\)\s+\S+\s+([0-9A-F]+)#([0-9A-F]*)$',line)
        if match is None:
            continue
        stamp = round(float(match[1]) * 1e9)
        first_rx = stamp if first_rx is None else min(first_rx,stamp)
        last_rx = stamp if last_rx is None else max(last_rx,stamp)
        cob = int(match[2],16)
        data = bytes.fromhex(match[3])
        if cob == 0x8a:
            faults.append({'t_ns':stamp,'data':data.hex().upper()})
        if not (0x581 <= cob <= 0x588 or cob == 0x58a) or len(data) != 8:
            continue
        if data[0] not in (0x43,0x47,0x4b,0x4f,0x60,0x80):
            continue
        replies.append({'t_ns':stamp, 'node':cob-0x580,
            'object':f'{int.from_bytes(data[1:3],"little"):04X}:{data[3]}',
            'command':data[0], 'data':data.hex().upper()})
    events = [(r['t_ns'],0,r) for r in requests] + [(r['t_ns'],1,r) for r in replies]
    pending = collections.defaultdict(list)
    overlaps = []
    orphan_replies = []
    maximum_inflight = 0
    for stamp, kind, item in sorted(events, key=lambda event:(event[0],event[1])):
        node = item['node']
        if kind == 0:
            active = [r for r in pending[node] if r['reply'] is None and stamp-r['t_ns'] < 100_000_000]
            if active:
                overlaps.append({'node':node,'new':item['object'],'new_ns':stamp,
                    'previous':[{'object':r['object'],'t_ns':r['t_ns']} for r in active]})
            pending[node].append(item)
            maximum_inflight = max(maximum_inflight,sum(
                r['reply'] is None and 0 <= stamp-r['t_ns'] < 100_000_000
                for entries in pending.values() for r in entries))
        else:
            candidates = [r for r in pending[node] if r['reply'] is None and r['object']==item['object']
                and (item['command']==0x80 or (r['read'] and item['command'] in (0x43,0x47,0x4b,0x4f))
                     or (not r['read'] and item['command']==0x60))]
            timely = [r for r in candidates if stamp-r['t_ns'] <= 100_000_000]
            if candidates:
                chosen = (timely or candidates)[0]
                chosen['reply'] = item
                chosen['latency_ms'] = (stamp-chosen['t_ns'])/1e6
            else:
                orphan_replies.append(item)
    start = max(first_rx, min(r['t_ns'] for r in requests)) + 250_000_000
    end = min(last_rx, max(r['t_ns'] for r in requests)) - 250_000_000
    interior = [r for r in requests if start <= r['t_ns'] <= end]
    result = {'capture':str(root),'requests_recorded':len(requests),'replies_recorded':len(replies),
        'interior_start_ns':start,'interior_end_ns':end,'interior_requests':len(interior),
        'interior_unanswered':[], 'aborts':[], 'per_node':{},
        'request_before_prior_reply':overlaps,'maximum_observed_global_inflight_under_100ms':maximum_inflight,
        'unmatched_reply_count':len(orphan_replies), 'faults':faults,
        'unmatched_replies_before_first_recorded_request':sum(
            r['t_ns'] < min(q['t_ns'] for q in requests) for r in orphan_replies),
        'unmatched_reply_time_range_ns':[
            min((r['t_ns'] for r in orphan_replies),default=None),
            max((r['t_ns'] for r in orphan_replies),default=None)],
        'limitations':'Source ROS publication timestamps precede actual transmission; '
            'latencies include the sender path and USB receive delivery. '
            'Incomplete ROS discovery and capture boundaries can leave orphan replies. '
            'Repeated same-object requests following a timeout have no unique transaction ID.'}
    for node in sorted({r['node'] for r in interior}):
        items = [r for r in interior if r['node']==node]
        matched = [r for r in items if r['reply'] is not None]
        missing = [r for r in items if r['reply'] is None]
        aborted = [r for r in matched if r['reply']['command']==0x80]
        result['interior_unanswered'].extend(missing)
        result['aborts'].extend(aborted)
        result['per_node'][str(node)] = {'requests':len(items),'matched':len(matched),
            'unanswered':len(missing),'aborts':len(aborted),
            'latency':percentiles([r['latency_ms'] for r in matched])}
    result['reply_latency'] = percentiles([r['latency_ms'] for r in interior if r['reply'] is not None])
    fault = next((f for f in faults if f['data'].startswith('5081')), None)
    if fault:
        before = [r for r in interior if r['t_ns'] < fault['t_ns']]
        result['before_collision'] = {'requests':len(before),'unanswered':sum(r['reply'] is None for r in before),
            'aborts':sum(r['reply'] is not None and r['reply']['command']==0x80 for r in before),
            'latency':percentiles([r['latency_ms'] for r in before if r['reply'] is not None])}
    (root/'sdo_response_analysis.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in (
        'interior_unanswered','aborts','request_before_prior_reply','faults')},indent=2))
    print('Unanswered objects:',dict(collections.Counter((r['node'],r['object']) for r in result['interior_unanswered'])))
    print('Overlap examples:',overlaps[:5])
    print('Abort examples:',result['aborts'][:3])


if __name__=='__main__':
    for argument in sys.argv[1:]:
        analyze(Path(argument))

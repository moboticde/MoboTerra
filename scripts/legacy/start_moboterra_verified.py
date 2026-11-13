"""Run the selected-profile smoke test, start the managed stack, report health."""
import hashlib,json,subprocess,time
from pathlib import Path

def run(args,**kwargs):
    return subprocess.run(args,check=True,text=True,**kwargs)

values=dict(line.split('=',1) for line in Path('/etc/moboterra/moboterra.env').read_text().splitlines() if line and not line.startswith('#') and '=' in line)
image=values['MOBOTERRA_IMAGE']
proof_path=Path('/home/mobotic/moboterra_selected_profile_test.json')
proof=json.loads(proof_path.read_text()) if proof_path.exists() else {}
profile_digest=hashlib.sha256(Path('/etc/moboterra/platform.yaml').read_bytes()).hexdigest()
if proof.get('passed') and proof.get('image')==image and proof.get('profile_sha256')==profile_digest:
    print('Exact image and selected profile already passed isolated smoke validation.',flush=True)
else:
    print(f'Checking selected two-module profile with image {image}',flush=True)
    run(['docker','run','--rm','--init','--network','none','--env','ROS_DOMAIN_ID=42',
         '--mount','type=bind,src=/etc/moboterra,dst=/etc/moboterra,readonly',image,
         'python3','/opt/moboterra-checks/virtual_smoke.py','--platform-config','/etc/moboterra/platform.yaml'])
run(['systemctl','restart','moboterra-supervisor.service'])
deadline=time.monotonic()+125
while time.monotonic()<deadline:
    result=subprocess.run(['docker','inspect','--format','{{json .State}}','moboterra-robot'],capture_output=True,text=True)
    state=json.loads(result.stdout) if result.returncode==0 else {}
    health=state.get('Health',{}).get('Status','absent')
    host_path=Path('/run/moboterra-supervisor/state.json')
    try: host=json.loads(host_path.read_text())
    except (FileNotFoundError,json.JSONDecodeError): host={}
    print(json.dumps({'container':state.get('Status','absent'),'health':health,'host':host}),flush=True)
    if health=='healthy': break
    if state.get('Status') in ('exited','dead') or health=='unhealthy': break
    time.sleep(5)
run(['systemctl','--no-pager','--full','status','moboterra-supervisor.service'])
if health!='healthy':
    subprocess.run(['docker','logs','--tail','100','moboterra-robot'],check=False)
    raise SystemExit('Hardware container did not become healthy; inspect logs.')
probe=Path('/home/mobotic/moboterra_startup_probe.py').read_text()
result=run(['docker','exec','-i','moboterra-robot','bash','-c',
            'source /opt/ros/jazzy/setup.bash && source /opt/moboterra/setup.bash && python3 -'],input=probe,capture_output=True,timeout=35)
Path('/home/mobotic/moboterra_startup_status.json').write_text(result.stdout)
print(result.stdout,flush=True)
if result.stderr: print(result.stderr,flush=True)
print('Managed MoboTerra container is running and healthy.',flush=True)

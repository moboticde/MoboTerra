#include <mobotic_driver/virtual_sync.hpp>
#include <stdexcept>

void check(bool value)
{
  if (!value) { throw std::runtime_error("virtual SYNC window regression"); }
}

int main()
{
  constexpr int64_t start = 1'000'000'000;
  check(virtualSyncPending(true, start, 0, start + 20'000'000, 0.3));
  check(!virtualSyncPending(false, start, 0, start + 20'000'000, 0.3));
  check(!virtualSyncPending(true, start, start, start + 20'000'000, 0.3));
  check(!virtualSyncPending(true, start, 0, start + 100'000'000, 0.3));
  check(virtualSyncPending(true, start, 0, start + 49'000'000, 0.1));
  check(!virtualSyncPending(true, start, 0, start + 50'000'000, 0.1));
  check(!virtualSyncPending(true, start, 0, start - 1, 0.3));
  check(!virtualSyncPending(true, 0, 0, start, 0.3));
}

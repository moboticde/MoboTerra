#pragma once
#include <cstdint>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <optional>
#include <string>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

struct PositionState
{
  std::string robot, encoder, joint, fingerprint, calibration_epoch, hardware_epoch;
  int64_t count{0}, calibration_offset{0}, retained_reference{0};
  int32_t raw{0};
};

// Durable, exclusively owned calibration records. This is storage, NOT evidence
// that an incremental encoder measured movement while power was absent.
class PositionStateStore
{
public:
  explicit PositionStateStore(const std::string& directory)
  {
    directory_ = ::open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    if (directory_ < 0) { throw std::runtime_error("position state directory unavailable"); }
    lock_ = ::openat(directory_, "writer.lock", O_CREAT | O_RDWR | O_NOFOLLOW | O_CLOEXEC, 0600);
    if (lock_ < 0 || ::flock(lock_, LOCK_EX | LOCK_NB) != 0)
    { if (lock_ >= 0) { ::close(lock_); } ::close(directory_); throw std::runtime_error("position state already owned/unavailable"); }
  }
  PositionStateStore(const PositionStateStore&) = delete;
  PositionStateStore& operator=(const PositionStateStore&) = delete;
  ~PositionStateStore() { ::close(lock_); ::close(directory_); }

  std::optional<PositionState> read(const std::string& key, const PositionState& identity) const
  {
    validateKey(key);
    const int fd = ::openat(directory_, (key + ".state").c_str(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
    if (fd < 0) { if (errno == ENOENT) { return std::nullopt; } throw std::runtime_error("position state unreadable"); }
    std::string data; char buffer[1024]; ssize_t n;
    while ((n = ::read(fd, buffer, sizeof(buffer))) > 0)
    { data.append(buffer, static_cast<std::size_t>(n)); if (data.size() > 8192) { ::close(fd); throw std::runtime_error("oversized position state"); } }
    ::close(fd);
    if (n < 0) { throw std::runtime_error("position state read failed"); }
    const auto split = data.rfind('\n', data.size() > 1 ? data.size()-2 : 0);
    if (split == std::string::npos) { throw std::runtime_error("incomplete position state"); }
    const auto payload = data.substr(0, split+1);
    uint64_t checksum = 0; std::istringstream checksum_stream(data.substr(split+1));
    if (!(checksum_stream >> checksum) || checksum != hash(payload) || !(checksum_stream >> std::ws).eof())
    { throw std::runtime_error("position state checksum mismatch"); }
    PositionState state; unsigned version;
    std::istringstream stream(payload);
    if (!(stream >> version >> std::quoted(state.robot) >> std::quoted(state.encoder) >> std::quoted(state.joint)
          >> std::quoted(state.fingerprint) >> std::quoted(state.calibration_epoch) >> std::quoted(state.hardware_epoch)
          >> state.count >> state.raw >> state.calibration_offset >> state.retained_reference) || version != 1)
    { throw std::runtime_error("invalid position state schema"); }
    if (!(stream >> std::ws).eof() || state.robot != identity.robot || state.encoder != identity.encoder || state.joint != identity.joint ||
        state.fingerprint != identity.fingerprint || state.calibration_epoch.empty() || state.calibration_epoch != identity.calibration_epoch)
    { throw std::runtime_error("position state identity/calibration mismatch"); }
    return state;
  }

  void write(const std::string& key, const PositionState& state) const
  {
    validateKey(key);
    if (state.robot.empty() || state.encoder.empty() || state.joint.empty() || state.fingerprint.empty() || state.calibration_epoch.empty())
    { throw std::runtime_error("position state identity/calibration required"); }
    std::ostringstream stream;
    stream << "1 " << std::quoted(state.robot) << ' ' << std::quoted(state.encoder) << ' ' << std::quoted(state.joint) << ' '
           << std::quoted(state.fingerprint) << ' ' << std::quoted(state.calibration_epoch) << ' ' << std::quoted(state.hardware_epoch) << ' '
           << state.count << ' ' << state.raw << ' ' << state.calibration_offset << ' ' << state.retained_reference << '\n';
    const std::string payload = stream.str();
    const std::string data = payload + std::to_string(hash(payload)) + "\n";
    const std::string temporary = key + ".pending-" + std::to_string(::getpid());
    const int fd = ::openat(directory_, temporary.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_NOFOLLOW | O_CLOEXEC, 0600);
    if (fd < 0) { throw std::runtime_error("position checkpoint temporary file unavailable"); }
    std::size_t offset = 0;
    while (offset < data.size())
    {
      const ssize_t n = ::write(fd, data.data()+offset, data.size()-offset);
      if (n < 0 && errno == EINTR) { continue; }
      if (n <= 0) { ::close(fd); ::unlinkat(directory_, temporary.c_str(), 0); throw std::runtime_error("position checkpoint write failed"); }
      offset += static_cast<std::size_t>(n);
    }
    const bool synced = ::fsync(fd) == 0; ::close(fd);
    if (!synced || ::renameat(directory_, temporary.c_str(), directory_, (key + ".state").c_str()) != 0)
    { ::unlinkat(directory_, temporary.c_str(), 0); throw std::runtime_error("position checkpoint commit failed"); }
    if (::fsync(directory_) != 0) { throw std::runtime_error("position checkpoint directory sync failed"); }
  }
private:
  static void validateKey(const std::string& key)
  {
    if (key.empty() || key.size() > 128 || key.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos)
    { throw std::runtime_error("invalid position record key"); }
  }
  static uint64_t hash(const std::string& value)
  { uint64_t result = 14695981039346656037ULL; for (unsigned char c : value) { result = (result ^ c) * 1099511628211ULL; } return result; }
  int directory_{-1}, lock_{-1};
};

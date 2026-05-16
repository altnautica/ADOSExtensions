#include "ipc_channel.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>

namespace ados::vision_nav {

IpcChannel::IpcChannel(const std::string& socket_path)
    : socket_path_(socket_path) {
  fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd_ < 0) {
    throw std::runtime_error("socket() failed");
  }
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);
  if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("connect() failed: " + socket_path_);
  }
}

IpcChannel::~IpcChannel() {
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

bool IpcChannel::send(const std::vector<unsigned char>& payload) {
  uint32_t len = htonl(static_cast<uint32_t>(payload.size()));
  if (::write(fd_, &len, sizeof(len)) != sizeof(len)) return false;
  ssize_t written = 0;
  while (written < static_cast<ssize_t>(payload.size())) {
    ssize_t n = ::write(fd_, payload.data() + written,
                        payload.size() - written);
    if (n <= 0) return false;
    written += n;
  }
  return true;
}

bool IpcChannel::recv(std::vector<unsigned char>& out) {
  uint32_t net_len = 0;
  if (::read(fd_, &net_len, sizeof(net_len)) != sizeof(net_len)) return false;
  uint32_t len = ntohl(net_len);
  out.resize(len);
  ssize_t got = 0;
  while (got < static_cast<ssize_t>(len)) {
    ssize_t n = ::read(fd_, out.data() + got, len - got);
    if (n <= 0) return false;
    got += n;
  }
  return true;
}

}  // namespace ados::vision_nav

// IPC channel adapter shared by both vendor binaries.
//
// Matches the Python-side ``MsgpackChannel`` at
// ../../agent/src/altnautica_vision_nav/shim/ipc.py: length-prefixed
// msgpack frames over a connect-side UDS. The binary opens the
// socket as a client; the plugin's Python side runs the listening
// server.

#pragma once

#include <string>
#include <vector>

namespace ados::vision_nav {

class IpcChannel {
 public:
  explicit IpcChannel(const std::string& socket_path);
  ~IpcChannel();

  // Write a length-prefixed msgpack frame.
  bool send(const std::vector<unsigned char>& payload);

  // Read one length-prefixed msgpack frame. Blocks until a complete
  // frame arrives or the peer closes the socket.
  bool recv(std::vector<unsigned char>& out);

 private:
  std::string socket_path_;
  int fd_ = -1;
};

}  // namespace ados::vision_nav

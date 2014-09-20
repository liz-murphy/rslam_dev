// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <stdint.h>
#include <string>

struct ServerCVars {
  ServerCVars();
  ~ServerCVars() = default;
  ServerCVars(const ServerCVars& cvars) = delete;

  uint32_t& debug_level;
  uint16_t& server_bind_port;
  uint16_t& client_connect_port;
  std::string& client_connect_ip;
  size_t& fetch_check_msg_size_skip;
  size_t& upload_depth;
};

extern ServerCVars g_server_cvars;

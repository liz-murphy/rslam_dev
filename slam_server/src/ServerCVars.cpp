// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <SlamServer/ServerCVars.h>

#include <Utils/CVarHelpers.h>
#include <CVars/CVar.h>

ServerCVars::ServerCVars()
    : debug_level(CVarUtils::CreateCVar<>("debug.Server", 1U, "Server debug.")),
      server_bind_port(CVarUtils::CreateCVar<uint16_t>(
          "server.node.bind_port", 1776,
          "Port for NodeSlamServer to bind on.")),

      client_connect_port(CVarUtils::CreateCVar<uint16_t>(
          "server.node.connect_port", 1776,
          "Port for NodeSlamClient to connect to without autodiscovery.")),

      client_connect_ip(CVarUtils::CreateCVar<std::string>(
          "server.node.connect_ip", "rpg.robotics.gwu.edu",
          "IP for NodeSlamClient to connect to without autodiscovery.")),

      fetch_check_msg_size_skip(CVarUtils::CreateCVar<size_t>(
          "server.fetch.check_msg_size_skip", 50,
          "Number of frames to skip between checks to message size "
          "when fetching the map.")),

      upload_depth(CVarUtils::CreateCVar<size_t>(
          "server.upload_depth", 500,
          "Depth to fetch frames when uploading map chunks"))
{}

ServerCVars g_server_cvars;

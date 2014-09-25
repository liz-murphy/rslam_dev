// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <SlamServer/NodeSlamClient.h>

#include <chrono>
#include <thread>
#include <SlamServer.pb.h>
#include <SlamServer/NodeDefinitions.h>
//#include <SlamServer/ServerCVars.h>
#include <slam_server/ServerConfig.h>

#ifdef ANDROID
static const bool kUseAutoDiscovery = false;
#else
static const bool kUseAutoDiscovery = true;
#endif

NodeSlamClient::NodeSlamClient() : node_(kUseAutoDiscovery) {
  node_.init("slam-client");
  if (!kUseAutoDiscovery) {
    msg::GetTableResponse response_msg;

    // Keep trying to connect in case the server eventually comes
    // online or we connect to the internet
    while (!node_.ConnectNode(ServerConfig::getConfig()->client_connect_ip,
                              ServerConfig::getConfig()->client_connect_port,
                              &response_msg)) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  // Upload an empty map to force the connection immediately so later
  // maps are not lost.
  pb::PlaceMapMsg empty_map;
  UploadMap(empty_map);
}

NodeSlamClient::~NodeSlamClient() {}

void NodeSlamClient::UploadMap(const pb::PlaceMapMsg& map) {
  if (!node_.send(rpg::kServerEndpoint + "/" + rpg::kUploadMapEndpoint, map)) {
    LOG(ERROR) << "Failed to UploadMap";
  }
}

void NodeSlamClient::UploadMap(const pb::ServerUploadRequest& request,
                               pb::ServerUploadResponse* response) {
  node_.call_rpc(rpg::kServerEndpoint, rpg::kUploadMapRPC, request, *response);
}

void NodeSlamClient::QueryPlace(const pb::ServerQueryRequest& request,
                                pb::ServerQueryResponse* response) {
  node_.call_rpc(rpg::kServerEndpoint, rpg::kQueryPlaceRPC, request, *response);
}

void NodeSlamClient::DownloadMap(const pb::ServerDownloadRequest& request,
                                 pb::ServerDownloadResponse* response) const {
  node_.call_rpc(rpg::kServerEndpoint, rpg::kDownloadMapRPC,
                 request, *response);
}

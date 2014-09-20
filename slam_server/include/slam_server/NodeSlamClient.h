// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <common/config.h>

#ifdef HAVE_NODE
#include <slam_server/SlamServerInterface.h>
#include <slam_server/SlamServerFwd.h>
#include <node/Node.h>

class NodeSlamClient : public SlamServerInterface {
 public:
  NodeSlamClient();
  virtual ~NodeSlamClient();

  void UploadMap(const pb::PlaceMapMsg& request) override;
  void UploadMap(const pb::ServerUploadRequest& request,
                 pb::ServerUploadResponse* response) override;

  void QueryPlace(const pb::ServerQueryRequest& request,
                  pb::ServerQueryResponse* response) override;

  void DownloadMap(const pb::ServerDownloadRequest& request,
                   pb::ServerDownloadResponse* response) const override;

 private:
  mutable node::node node_;
};

#endif  // HAVE_NODE

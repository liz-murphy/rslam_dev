// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <memory>
#include <slam_server/SlamServerFwd.h>

class SlamServerInterface {
 public:
  virtual ~SlamServerInterface() {}

  /** Upload a section of map to the server. */
  virtual void UploadMap(const pb::PlaceMapMsg& request) = 0;

  /** Upload a section of map to the server. */
  virtual void UploadMap(const pb::ServerUploadRequest& request,
                         pb::ServerUploadResponse* response) = 0;

  /** Query for a matching place. */
  virtual void QueryPlace(const pb::ServerQueryRequest& request,
                          pb::ServerQueryResponse* response) = 0;

  /** Fetch a section of the map from the server. */
  virtual void DownloadMap(const pb::ServerDownloadRequest& request,
                           pb::ServerDownloadResponse* response) const = 0;
};

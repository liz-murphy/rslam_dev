// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#pragma once

#include <string>
#include <slam_map/DataStore/SqliteTable.h>
#include <slam_map/ProtobufIO.h>

class CameraRigPtrTable : public SqliteTable<SessionId,
                                             _CameraRigPtr,
                                             pb::CameraRigMsg,
                                             CameraRigPtrTable>  {
 public:
  enum IdColumn {
    MapIdColumn,
  };

  explicit CameraRigPtrTable(const std::string& filename) :
      SqliteTable<SessionId,
                  _CameraRigPtr,
                  pb::CameraRigMsg,
                  CameraRigPtrTable>(filename) {
    set_name("rigs");
    AddIdColumn(MapIdColumn, "MapId", "TEXT");
  }

  static void Bind(const SessionId& id, int id_n,
                   int column, sqlite3_stmt* stmt) {
    int sql_result = 0;
    switch (id_n) {
      case MapIdColumn:
        sql_result = sqlite3_bind_text(stmt, column,
                                       rslam::uuid::uuid_begin(id.uuid),
                                       rslam::uuid::uuid_size(id.uuid),
                                       NULL);
        break;
    }
    CheckSqliteReturn(SQLITE_OK, sql_result, "CameraRigPtrTable::Bind", NULL);
  }

  static inline void ConvertId(const SessionId& id, SessionId* out) {
    *out = id;
  }

  static inline void ParseColumn(
      int column, const void* data, int size, SessionId* id) {
    CHECK_EQ(size, sizeof(id->uuid));
    rslam::uuid::uuid_copy_from(data, &id->uuid);
  }
};

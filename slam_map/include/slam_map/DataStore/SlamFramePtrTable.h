// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#pragma once

#include <string>

#include <slam_map/DataStore/SqliteTable.h>
#include <slam_map/ProtobufIO.h>

class SlamFramePtrTable : public SqliteTable<ReferenceFrameId,
                                             SlamFramePtr,
                                             pb::ReferenceFrameMsg,
                                             SlamFramePtrTable>  {
 public:
  enum IdColumn {
    MapIdColumn,
    FrameIdColumn,
  };

  explicit SlamFramePtrTable(const std::string& filename) :
      SqliteTable<ReferenceFrameId,
                  SlamFramePtr,
                  pb::ReferenceFrameMsg,
                  SlamFramePtrTable>(filename) {
    set_name("frames");
    AddIdColumn(MapIdColumn, "MapId", "TEXT");
    AddIdColumn(FrameIdColumn, "Id", "INTEGER");
  }

  static void Bind(const ReferenceFrameId& id, int id_n,
                   int column, sqlite3_stmt* stmt) {
    int sql_result = 0;
    switch (id_n) {
      case MapIdColumn:
        sql_result = sqlite3_bind_text(stmt, column,
                                       rslam::uuid::uuid_begin(id.session_id.uuid),
                                       rslam::uuid::uuid_size(id.session_id.uuid),
                                       NULL);
        break;
      case FrameIdColumn:
        sql_result = sqlite3_bind_int(stmt, column, id.id);
        break;
    }
    CheckSqliteReturn(SQLITE_OK, sql_result, "SlamFramePtrTable::Bind", NULL);
  }

  static inline void ConvertId(const ReferenceFrameId& id, SessionId* out) {
    *out = id.session_id;
  }

  static inline void ParseColumn(
      int column, const void* data, int size, SessionId* id) {
    CHECK_EQ(size, sizeof(id->uuid));
    rslam::uuid::uuid_copy_from(data, &id->uuid);
  }
};

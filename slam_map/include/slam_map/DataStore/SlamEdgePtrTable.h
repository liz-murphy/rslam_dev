// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#pragma once

#include <string>

#include <slam_map/DataStore/SqliteTable.h>
#include <slam_map/ProtobufIO.h>

class SlamEdgePtrTable : public SqliteTable<TransformEdgeId,
                                            SlamEdgePtr,
                                            pb::TransformEdgeMsg,
                                            SlamEdgePtrTable> {
 public:
  enum IdColumn {
    MapIdColumn,
    EdgeIdColumn,
  };

  explicit SlamEdgePtrTable(const std::string& filename) :
      SqliteTable<TransformEdgeId,
                  SlamEdgePtr,
                  pb::TransformEdgeMsg,
                  SlamEdgePtrTable>(filename) {
    set_name("edges");
    AddIdColumn(MapIdColumn, "MapId", "TEXT");
    AddIdColumn(EdgeIdColumn, "Id", "INTEGER");
  }

  static void Bind(const TransformEdgeId& id, int id_n,
                   int column, sqlite3_stmt* stmt) {
    int sql_result = 0;
    switch (id_n) {
      case MapIdColumn:
        sql_result = sqlite3_bind_text(
            stmt, column,
            rslam::uuid::uuid_begin(id.session_id.uuid),
            rslam::uuid::uuid_size(id.session_id.uuid),
            NULL);
        break;
      case EdgeIdColumn:
        sql_result = sqlite3_bind_int(stmt, column, id.id);
        break;
    }
    CheckSqliteReturn(SQLITE_OK, sql_result, "SlamEdgePtrTable::Bind", NULL);
  }

  static inline void ConvertId(const TransformEdgeId& id, SessionId* out) {
    *out = id.session_id;
  }

  static inline void ParseColumn(
      int column, const void* data, int size, SessionId* id) {
    CHECK_EQ(size, sizeof(id->uuid));
    rslam::uuid::uuid_copy_from(data, &id->uuid);
  }
};

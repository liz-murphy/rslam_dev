// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#pragma once

#include <pthread.h>
#include <map>
#include <string>
#include <utility>

#include <slam_map/sqlite3/sqlite3.h>
#include <slam_map/ProtobufIO.h>

#include <ros/ros.h>

int busy_handler(void*, int) {
  static const timespec ts = {0, 1000};
  nanosleep(&ts, NULL);
  return 1;
}

inline void PrintError(const std::string& op, int result, char* errmsg) {
  ROS_ERROR("SqliteDataStore database %s failed! Exit code: %d Error message: %s", op.c_str(), result, errmsg);
}

inline void CheckSqliteReturn(int expected,
                              int result,
                              const std::string& op,
                              char* errmsg) {
  if (result != expected) {
    PrintError(op, result, errmsg);
  }
}

/** An interface to a single sqlite table */
template <typename IdT, typename PtrT, typename MsgT, typename Derived>
class SqliteTable {
 public:
  /** STL-style types forwarded from the template */
  typedef PtrT pointer_type;
  typedef IdT id_type;
  typedef MsgT message_type;

  SqliteTable(const std::string& filename) :
      initialized_(false),
      filename_(filename),
      db_(nullptr),
      write_stmt_(nullptr),
      load_stmt_(nullptr),
      begin_stmt_(nullptr),
      commit_stmt_(nullptr),
      debug_level_(1) {}

  void Init() {
    if (!sqlite3_threadsafe()) {
      ROS_ERROR("Platform's sqlite3 is not threadsafe, SqliteDataStore will not work correctly.");
    }

    sqlite3_config(SQLITE_CONFIG_MULTITHREAD);
    int sql_result = sqlite3_open(filename_.c_str(), &db_);
    CheckSqliteReturn(0, sql_result, "open", NULL);

    sqlite3_busy_handler(db_, &busy_handler, NULL);
    sqlite3_extended_result_codes(db_, 1);

    std::string journal_wal = "PRAGMA journal_mode=WAL;";
    char* errmsg = nullptr;
    sql_result = sqlite3_exec(db_, journal_wal.c_str(),
                                  NULL, NULL, &errmsg);
    CheckSqliteReturn(0, sql_result, "PRAGMA WAL", errmsg);

    std::string sync_off = "PRAGMA synchronous=NORMAL;";
    sql_result = sqlite3_exec(db_, sync_off.c_str(),
                              NULL, NULL, NULL);
    CheckSqliteReturn(0, sql_result, "PRAGMA synchronous", errmsg);

    build();
    PrepareStatements();
  }

  bool Exists() {
    sqlite3_stmt* exists_stmt;
    std::string select_stmt = "SELECT name FROM sqlite_master WHERE "
        "type='table' AND name='" + name_ + "'";
    int sql_result = sqlite3_prepare_v2(db_, select_stmt.c_str(),
                                        select_stmt.size(), &exists_stmt,
                                        NULL);
    CheckSqliteReturn(SQLITE_OK, sql_result, "prepare exists " + name_, NULL);

    sql_result = sqlite3_step(exists_stmt);
    sqlite3_finalize(exists_stmt);

    return sql_result == SQLITE_ROW;
  }

  void BeginTransaction() {
    int sql_result = sqlite3_step(begin_stmt_);
    CheckSqliteReturn(SQLITE_DONE, sql_result, "begin transaction", NULL);
    sqlite3_reset(begin_stmt_);
  }

  bool CommitTransaction() {
    int sql_result = sqlite3_step(commit_stmt_);
    sqlite3_reset(commit_stmt_);
    ROS_ERROR_COND(sql_result != SQLITE_DONE,"Commit transaction failed. Error code: %d",sql_result);
    return sql_result == SQLITE_DONE;
  }

  /**
   * Write the given frame to disk.
   *
   * @requires Write lock is already in place on sqlite
   */
  void Write(const id_type& id, const pointer_type& ptr) {
    Encode(ptr);

    int column = 1;
    for (const auto& header_type : id_headers_) {
      Derived::Bind(id, header_type.first, column++, write_stmt_);
    }

    int sql_result = sqlite3_bind_blob(write_stmt_, id_headers_.size() + 1,
                                       encoded_string_.c_str(),
                                       encoded_string_.size(), NULL);
    CheckSqliteReturn(0, sql_result, "bind blob", NULL);

    do {
      sql_result = sqlite3_step(write_stmt_);
    } while (sql_result != SQLITE_DONE);
    sqlite3_reset(write_stmt_);
  }

  /**
   * Read the given object from the DB and write into the output pointer
   *
   * @returns False if the desired id cannot be found in the table
   */
  bool Read(const id_type& id, pointer_type* out) {
    bool found = true;
    int column = 1;
    for (const auto& header_type : id_headers_) {
      Derived::Bind(id, header_type.first, column++, load_stmt_);
    }

    int sql_result = sqlite3_step(load_stmt_);

    if (sql_result == SQLITE_ROW) {
      LoadFromQuery(load_stmt_, out);
    } else {
      ROS_DEBUG_NAMED("slam_map_sqlite","No object found in select on table %s with given Id", name_.c_str());
      found = false;
    }
    sqlite3_reset(load_stmt_);
    return found;
  }

  /** Load the number of rows in the table */
  size_t Size() {
    sqlite3_stmt* select_max_stmt;
    std::string select_stmt = "SELECT Count(Proto) FROM " + name_ + ";";
    int sql_result = sqlite3_prepare_v2(db_, select_stmt.c_str(),
                                        select_stmt.size(), &select_max_stmt,
                                        NULL);
    CheckSqliteReturn(0, sql_result, "prepare select all" + name_, NULL);

    sql_result = sqlite3_step(select_max_stmt);

    size_t size = 0;
    if (sql_result == SQLITE_ROW) {
      size = sqlite3_column_int(select_max_stmt, 0);
    }
    sqlite3_finalize(select_max_stmt);

    return size;
  }

  /** Erase all data from the table */
  void Clear() {
    const std::string delete_stmt = "DELETE FROM " + name_;
    char* errmsg = nullptr;
    int sql_result = sqlite3_exec(db_, delete_stmt.c_str(),
                                  NULL, NULL, &errmsg);
    CheckSqliteReturn(0, sql_result, delete_stmt, errmsg);
  }

  /** Destroy self */
  void Close() {
    int sql_result = sqlite3_finalize(write_stmt_);
    CheckSqliteReturn(SQLITE_OK, sql_result, "finalize write", NULL);

    sql_result = sqlite3_finalize(load_stmt_);
    CheckSqliteReturn(SQLITE_OK, sql_result, "finalize load", NULL);

    sql_result = sqlite3_finalize(begin_stmt_);
    CheckSqliteReturn(SQLITE_OK, sql_result, "finalize begin", NULL);

    sql_result = sqlite3_finalize(commit_stmt_);
    CheckSqliteReturn(SQLITE_OK, sql_result, "finalize commit", NULL);

    sql_result = sqlite3_close(db_);
    CheckSqliteReturn(0, sql_result, "close", NULL);
    db_ = nullptr;
  }

  template <typename T>
  void UniqueIds(int column, std::set<T>* ids) const {
    std::string unique_id_str =
        ("SELECT DISTINCT " + id_headers_.at(column).first + " FROM " + name_);

    sqlite3_stmt* unique_id_stmt;
    int sql_result = sqlite3_prepare_v2(db_, unique_id_str.c_str(),
                                        unique_id_str.size(),
                                        &unique_id_stmt, NULL);
    CheckSqliteReturn(0, sql_result, "prepare unique_id", NULL);

    T to_add;
    while (sqlite3_step(unique_id_stmt) == SQLITE_ROW) {
      Derived::ParseColumn(column, sqlite3_column_text(unique_id_stmt, 0),
                           sqlite3_column_bytes(unique_id_stmt, 0),
                           &to_add);
      ids->insert(to_add);
    }
    sqlite3_reset(unique_id_stmt);

    sql_result = sqlite3_finalize(unique_id_stmt);
    CheckSqliteReturn(SQLITE_OK, sql_result, "finalize unique_id", NULL);
  }

 protected:
  void PrepareStatements() {
    std::stringstream insert_ss;
    insert_ss << "INSERT OR REPLACE INTO " << name_ << " (";
    for (const auto& header_type : id_headers_) {
      insert_ss << header_type.second.first << ", ";
    }
    insert_ss << "Proto) VALUES (";
    for (size_t i = 0; i < id_headers_.size(); ++i) {
      insert_ss << "?,";
    }
    insert_ss << "?);";

    const std::string insert_stmt = insert_ss.str();
    int sql_result = 0;
    sql_result = sqlite3_prepare_v2(db_, insert_stmt.c_str(),
                                    insert_stmt.size(), &write_stmt_, NULL);
    CheckSqliteReturn(0, sql_result, "prepare insert", NULL);

    std::stringstream select_ss;
    select_ss << "SELECT * FROM " << name_ << " WHERE ";

    for (const auto& header_type : id_headers_) {
      select_ss << name_ << "." << header_type.second.first << " = ?";
      if (header_type.first != id_headers_.rbegin()->first) {
        select_ss << " AND ";
      }
    }
    select_ss << ";";

    std::string select_stmt = select_ss.str();
    sql_result = sqlite3_prepare_v2(db_, select_stmt.c_str(),
                                    select_stmt.size(), &load_stmt_, NULL);
    CheckSqliteReturn(0, sql_result, "prepare select", NULL);

    std::string begin = "BEGIN TRANSACTION";
    sql_result = sqlite3_prepare_v2(db_, begin.c_str(),
                                    begin.size(), &begin_stmt_, NULL);
    CheckSqliteReturn(0, sql_result, "prepare begin", NULL);

    std::string commit = "COMMIT TRANSACTION";
    sql_result = sqlite3_prepare_v2(db_, commit.c_str(),
                                    commit.size(), &commit_stmt_, NULL);
    CheckSqliteReturn(0, sql_result, "prepare commit", NULL);
  }

  /** Initialize the name of the table */
  void set_name(const std::string& name) {
    name_ = name;
  }

  void AddIdColumn(int id,
                   const std::string& header,
                   const std::string& type) {
    id_headers_[id] = {header, type};
  }

  void build() {
    if (initialized_) {
      ROS_ERROR("Table %s name_ already initialized.", name_.c_str());
    } else if (id_headers_.empty()) {
      ROS_ERROR("SQLITE: No columns set.");
    } else if (Exists()) {
      initialized_ = true;
      return;
    }

    char* errmsg;
    std::stringstream create_ss;
    create_ss << "CREATE TABLE IF NOT EXISTS "  << name_
              << " ( ";
    for (const auto& header_type : id_headers_) {
      create_ss << header_type.second.first << " "
                << header_type.second.second << ", ";
    }
    create_ss << "Proto BLOB, ";

    create_ss << "PRIMARY KEY (";
    for (const auto& header_type : id_headers_) {
      create_ss << header_type.second.first;
      if (header_type.first != id_headers_.rbegin()->first) {
        create_ss << ", ";
      }
    }
    create_ss << ") ON CONFLICT ABORT)";

    std::string create_stmt = create_ss.str();
    int sql_result = sqlite3_exec(db_, create_stmt.c_str(),
                                  NULL, NULL, &errmsg);
    CheckSqliteReturn(0, sql_result, "CREATE TABLE", errmsg);

    initialized_ = true;
  }

  void Encode(const pointer_type& ptr) {
    proto_msg_.Clear();
    pb::fill_message(*ptr, &proto_msg_);
    proto_msg_.SerializeToString(&encoded_string_);
  }

  /** Given a statement pointed at an object row, load into the given
   * output pointer */
  void LoadFromQuery(sqlite3_stmt* stmt, pointer_type* out) {
    proto_msg_.Clear();
    proto_msg_.ParseFromArray(sqlite3_column_blob(stmt, id_headers_.size()),
                              sqlite3_column_bytes(stmt, id_headers_.size()));

    pb::parse_message(proto_msg_, out->get());
  }

 private:
  bool initialized_;
  std::string filename_, name_;
  sqlite3* db_;
  sqlite3_stmt* write_stmt_, *load_stmt_, *begin_stmt_, *commit_stmt_;
  std::string encoded_string_;
  message_type proto_msg_;

  // Column headers:
  // Key: column index
  // Value: header text, SQL type
  std::map<int, std::pair<std::string, std::string> > id_headers_;

  /** @todo Add a cvar to control this... */
  uint32_t debug_level_;
};

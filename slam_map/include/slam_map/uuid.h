// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <iostream>

#ifndef ANDROID
extern "C" {
#ifdef __SSE2__
#include <emmintrin.h>
#endif  // __SSE2

#ifdef __SSE3__
#include "pmmintrin.h"
#endif  // __SSE3

#ifdef __SSE4_1__
#include <smmintrin.h>
#endif  // __SSE4__
}

#include <uuid/uuid.h>
#include <string.h>

namespace std {
template <> struct hash<uuid_t> {
  size_t operator()(const uuid_t& x) const {
    static_assert(sizeof(size_t) == 8,
                  "Function not implemented for 32-bit size_t");
#ifdef __SSE3__
    union data {
      __m128i u128;
      __m64 u64[2];
    };

    union hash {
      __m64 u64;
      uint64_t v;
    };

    data v;
    hash h;

    v.u128 = _mm_lddqu_si128((__m128i*)&x[0]);
    h.u64 = _m_pxor(v.u64[0], v.u64[1]);
    return h.v;
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
    return
        static_cast<uint64_t>(*reinterpret_cast<const uint64_t*>(&x[0])) ^
        static_cast<uint64_t>(*reinterpret_cast<const uint64_t*>(&x[8]));
#pragma GCC diagnostic pop
#endif
  }
};
}

namespace rslam {
namespace uuid {

#ifndef _UUID_T
#define _UUID_T
typedef char uuid_string_t[37];
#else
using ::uuid_string_t;
#endif /* _UUID_T */

using ::uuid_t;
using ::uuid_clear;
using ::uuid_unparse;
using ::uuid_parse;
using ::uuid_generate;

inline char* uuid_begin(uuid_t& u) {
  return reinterpret_cast<char*>(&u[0]);
}

inline const char* uuid_begin(const uuid_t& u) {
  return reinterpret_cast<const char*>(&u[0]);
}

constexpr inline size_t uuid_size(const uuid_t& /* u */) {
  return sizeof(uuid_t);
}

inline void uuid_copy(uuid_t& out, const uuid_t& u) {
#ifdef __SSE2__
  _mm_storeu_si128((__m128i*)out, _mm_loadu_si128((const __m128i*)u));
#else  // __SSE2__
  ::uuid_copy(out, u);
#endif  // __SSE2__
}

inline bool uuid_less_than(const uuid_t& u1, const uuid_t& u2) {
#ifdef __SSE2__
  __v2di mu1 = _mm_loadu_si128((const __m128i*)u1);
  __v2di mu2 = _mm_loadu_si128((const __m128i*)u2);
  if (mu1[0] == mu2[0]) {
    return mu1[1] < mu2[1];
  }
  return mu1[0] < mu2[0];
#else  // __SSE2__
  return uuid_compare(u1, u2) < 0;
#endif  // __SSE2__
}

inline bool uuid_equal(const uuid_t& u1, const uuid_t& u2) {
#ifdef __SSE4_1__
  static const __m128i zero = {0};
  __m128i c = _mm_xor_si128(_mm_loadu_si128((const __m128i*)u1),
                            _mm_loadu_si128((const __m128i*)u2));
  return _mm_testc_si128(zero, c);
#else  // __SSE4_1__
  return uuid_compare(u1, u2) == 0;
#endif  // __SSE4_1__
}

inline void uuid_copy_from(const void* data, uuid_t* u) {
  memcpy(*u, data, sizeof(uuid_t));
}
}  // namespace uuid
}  // namespace rslam

#else  // ON ANDROID

#include <string>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/nil_generator.hpp>
#include <boost/uuid/string_generator.hpp>

namespace std {
template <> struct hash<boost::uuids::uuid> {
  size_t operator()(const boost::uuids::uuid& x) const {
    return boost::uuids::hash_value(x);
  }
};
}

namespace rslam {
namespace uuid {

typedef boost::uuids::uuid uuid_t;
typedef std::string uuid_string_t;

// Here we recreate the simple libuuid interface with Boost types
inline void uuid_generate(uuid_t& u) {
  static boost::uuids::random_generator gen;
  u = gen();
}

inline void uuid_clear(uuid_t& u) {
  u = boost::uuids::nil_uuid();
}

inline bool uuid_less_than(const uuid_t& u1, const uuid_t& u2) {
  return (u1 < u2);
}

inline bool uuid_equal(const uuid_t& u1, const uuid_t& u2) {
  return (u1 == u2);
}

inline void uuid_unparse(const uuid_t& u, uuid_string_t& str) {
  str = boost::uuids::to_string(u);
}

inline int uuid_parse(const uuid_string_t& str, uuid_t& u) {
  try {
    u = boost::uuids::string_generator()(str);
  } catch(...) {
    return false;
  }
  return true;
}

inline void uuid_copy(uuid_t& out, const uuid_t& u) {
  out = u;
}

inline size_t uuid_size(const uuid_t& u) {
  return boost::uuids::uuid::static_size();
}

inline char* uuid_begin(uuid_t& u) {
  return reinterpret_cast<char*>(&u.data[0]);
}

inline const char* uuid_begin(const uuid_t& u) {
  return reinterpret_cast<const char*>(&u.data[0]);
}

inline void uuid_copy_from(const void* data, uuid_t* u) {
  memcpy(u->data, data, sizeof(uuid_t));
}
}  // namespace uuid
}  // namespace rslam

#endif  // ANDROID

// Stream UUID straight to string rather than as jibberish char*
inline std::ostream&
operator<<(std::ostream& os, const rslam::uuid::uuid_t& uuid) {
  rslam::uuid::uuid_string_t str;
  rslam::uuid::uuid_unparse(uuid, str);
  os << std::string(str);
  return os;
}

// Stream UUID straight to string rather than as jibberish char*
inline std::istream&
operator>>(std::istream& is, rslam::uuid::uuid_t& uuid) {
  rslam::uuid::uuid_string_t str;
  is >> str;

  rslam::uuid::uuid_copy_from(&str[0], &uuid);
  return is;
}

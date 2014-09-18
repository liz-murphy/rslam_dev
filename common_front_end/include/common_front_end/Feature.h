// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#ifndef _COMMONFRONTEND_FEATURE_H_
#define _COMMONFRONTEND_FEATURE_H_

#include <algorithm>
#include <vector>

typedef uint8_t UINT8;

class Feature {
 public:

  // Default constructor
  Feature() : used(false), id(0), descsize(0), descriptor(),
              scale(0), orientation(0), x(0), y(0), response(0) {}

  // Constructor with basic initialization
  Feature(int iid, //< Input: feat Id, only used when TRACK2D is active
          float ix,
          float iy,
          float fScale,
          float fOrientation,
          float iresponse,
          const unsigned char* idescriptor = nullptr,
          unsigned int isize = 0,
          bool iused = false) : used(iused), id(iid),
                                descsize(0), descriptor(),
                                scale(fScale), orientation(fOrientation) {
    _Copy(iid, ix, iy, iresponse, idescriptor, isize, iused);
  }

  // Copy constructor
  Feature(const Feature& feat) : descsize(0),
                                 descriptor(),
                                 scale(feat.scale),
                                 orientation(feat.orientation) {
    _Copy(feat.id, feat.x, feat.y, feat.response,
          &feat.descriptor[0], feat.descsize, feat.used);
  }

  ~Feature() {}

  // assignment operator
  Feature& operator=(const Feature &rhs);

  void Set(int iid,
           float ix,
           float iy,
           float iresponse,
           const unsigned char* idescriptor,
           unsigned int isize,
           bool iused = false) {
    _Copy(iid, ix, iy, iresponse, idescriptor, isize, iused);
  }

 private:
  void _Copy(int iid, float ix, float iy, float iresponse,
             const unsigned char* idescriptor, unsigned int isize, bool iused) {
    id = iid;
    x = ix;
    y = iy;
    response = iresponse;
    used = iused;

    if (idescriptor) {
      descriptor.assign(idescriptor, idescriptor + isize);
      descsize = isize;
    }
  }

 public:
  bool              used;
  int               id;
  unsigned int      descsize;
  std::vector<uint8_t> descriptor;
  float             scale;
  float             orientation;
  float             x;
  float             y;
  float             response;
};

#endif

// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <memory>
#include <FrameObject.pb.h>
#include <Utils/MathTypes.h>
#include <slam_map/uuid.h>

/** Arbitrary renderable objects that can be attached to a ReferenceFrame */
struct FrameObject {
  enum class Type {
    Cube,
    Teapot,
    Text,
  };

  FrameObject(Type t) : type(t) {
    rslam::uuid::uuid_generate(id);
  }

  bool operator==(const FrameObject& other) const {
    return id == other.id;
  }

  Type type;
  rslam::uuid::uuid_t id;
};

struct CubeObject : public FrameObject {
  CubeObject() : FrameObject(Type::Cube) {
    scale.setOnes();
  }

  // Transform: parent from object
  Sophus::SE3t t_po;

  // Scale of object in X, Y, Z axes
  Eigen::Vector3t scale;
};

struct TeapotObject : public FrameObject {
  TeapotObject() : FrameObject(Type::Teapot) {
    scale.setOnes();
  }

  // Transform: parent from object
  Sophus::SE3t t_po;

  // Scale of object in X, Y, Z axes
  Eigen::Vector3t scale;
};

struct TextObject : public FrameObject {
  TextObject() : FrameObject(Type::Text) {
    scale = {0.1, 0.1, 0.1};
    color = {1.0, 1.0, 1.0, 1.0};
  }

  // Transform: parent from object
  Sophus::SE3t t_po;

  // Scale of object in X, Y, Z axes
  Eigen::Vector3t scale;

  // The content of the string to display
  std::string text;

  // RGBA color vector
  // range: (0.0, 1.0)
  Eigen::Vector4t color;
};

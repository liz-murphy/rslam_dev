#pragma once

#include <memory>
#include <common/scalar.h>

namespace calibu {
template <typename S> class CameraRigT;
};

enum EdgeAttribute {
  EdgeAttrib_IsBeingOptimized = 1 << 0,
  EdgeAttrib_AsyncIsBeingOptimized = 1 << 1,
  EdgeAttrib_Broken = 1 << 2,
};

typedef std::shared_ptr<calibu::CameraRigT<Scalar> > _CameraRigPtr;
typedef std::shared_ptr<const calibu::CameraRigT<Scalar> > CameraRigPtr;

class ReferenceFrame;
class TransformEdge;

typedef std::shared_ptr<TransformEdge> SlamEdgePtr;
typedef std::shared_ptr<ReferenceFrame> SlamFramePtr;

class Landmark;
struct PoseContainerT;
class slam_map;
class slam_mapDataStore;
struct MeasurementContainer;
class MultiViewMeasurement;

struct MeasurementId;
struct LandmarkId;
struct ReferenceFrameId;
struct TransformEdgeId;
struct SessionId;

class SlamEdgePtrTable;
class SlamFramePtrTable;
class CameraRigPtrTable;
template<typename TableT> class PersistentBackend;
class MapVisitor;

class slam_mapProxy;
class Pointerslam_mapProxy;
struct FrameObject;
struct CubeObject;
struct TeapotObject;
struct TextObject;

class GlobalMapView;
class GlobalMapViewUpdater;

namespace rslam {
namespace map {
class NotificationCenter;
struct PoseMeasurement;
struct VelocityMeasurement;
}  // namespace map
}  // namespace rslam

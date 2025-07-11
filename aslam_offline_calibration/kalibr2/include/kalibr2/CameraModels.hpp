#pragma once

#include <aslam/Frame.hpp>
#include <aslam/backend/CameraDesignVariable.hpp>
#include <aslam/backend/ReprojectionError.hpp>
#include <aslam/backend/SimpleReprojectionError.hpp>
#include <aslam/cameras.hpp>

// Definition of useful structs for camera models in Kalibr2.
// The following structs define, for each camera model used in Kalibr2
// calibration:
// - Geometry: the camera geometry type
// - ReprojectionError: the reprojection error type for optimization
// - Frame: the frame type associated with the geometry
// - ReprojectionErrorSimple: a simplified reprojection error type
// - DesignVariable: the design variable for optimization
// - Distortion: the distortion model used
// - Shutter: the shutter type (global or rolling)
// - ProjectionType: the projection model used

namespace kalibr2 {

namespace models {

struct Omni {
  using Geometry = aslam::cameras::OmniCameraGeometry;
  using ReprojectionError = aslam::backend::ReprojectionError<Geometry>;
  using Frame = aslam::Frame<Geometry>;
  using ReprojectionErrorSimple =
      aslam::backend::SimpleReprojectionError<Frame>;
  using DesignVariable = aslam::backend::CameraDesignVariable<Geometry>;
  using Distortion = aslam::cameras::NoDistortion;
  using Shutter = aslam::cameras::GlobalShutter;
  using ProjectionType = aslam::cameras::OmniProjection<Distortion>;
};

struct DistortedOmni {
  using Geometry = aslam::cameras::DistortedOmniCameraGeometry;
  using ReprojectionError = aslam::backend::ReprojectionError<Geometry>;
  using Frame = aslam::Frame<Geometry>;
  using ReprojectionErrorSimple =
      aslam::backend::SimpleReprojectionError<Frame>;
  using DesignVariable = aslam::backend::CameraDesignVariable<Geometry>;
  using Distortion = aslam::cameras::RadialTangentialDistortion;
  using Shutter = aslam::cameras::GlobalShutter;
  using ProjectionType = aslam::cameras::OmniProjection<Distortion>;
};

struct DistortedOmniRs {
  using Geometry = aslam::cameras::DistortedOmniRsCameraGeometry;
  using ReprojectionError = aslam::backend::ReprojectionError<Geometry>;
  using Frame = aslam::Frame<Geometry>;
  using ReprojectionErrorSimple =
      aslam::backend::SimpleReprojectionError<Frame>;
  using DesignVariable = aslam::backend::CameraDesignVariable<Geometry>;
  using Distortion = aslam::cameras::RadialTangentialDistortion;
  using Shutter = aslam::cameras::RollingShutter;
  using ProjectionType = aslam::cameras::OmniProjection<Distortion>;
};

struct DistortedPinhole {
  using Geometry = aslam::cameras::DistortedPinholeCameraGeometry;
  using ReprojectionError = aslam::backend::ReprojectionError<Geometry>;
  using Frame = aslam::Frame<Geometry>;
  using ReprojectionErrorSimple =
      aslam::backend::SimpleReprojectionError<Frame>;
  using DesignVariable = aslam::backend::CameraDesignVariable<Geometry>;
  using Distortion = aslam::cameras::RadialTangentialDistortion;
  using Shutter = aslam::cameras::GlobalShutter;
  using ProjectionType = aslam::cameras::PinholeProjection<Distortion>;
};

struct DistortedPinholeRs {
  using Geometry = aslam::cameras::DistortedPinholeRsCameraGeometry;
  using ReprojectionError = aslam::backend::ReprojectionError<Geometry>;
  using Frame = aslam::Frame<Geometry>;
  using ReprojectionErrorSimple =
      aslam::backend::SimpleReprojectionError<Frame>;
  using DesignVariable = aslam::backend::CameraDesignVariable<Geometry>;
  using Distortion = aslam::cameras::RadialTangentialDistortion;
  using Shutter = aslam::cameras::RollingShutter;
  using ProjectionType = aslam::cameras::PinholeProjection<Distortion>;
};

struct EquidistantPinhole {
  using Geometry = aslam::cameras::EquidistantDistortedPinholeCameraGeometry;
  using ReprojectionError = aslam::backend::ReprojectionError<Geometry>;
  using Frame = aslam::Frame<Geometry>;
  using ReprojectionErrorSimple =
      aslam::backend::SimpleReprojectionError<Frame>;
  using DesignVariable = aslam::backend::CameraDesignVariable<Geometry>;
  using Distortion = aslam::cameras::EquidistantDistortion;
  using Shutter = aslam::cameras::GlobalShutter;
  using ProjectionType = aslam::cameras::PinholeProjection<Distortion>;
};

struct EquidistantPinholeRs {
  using Geometry = aslam::cameras::EquidistantDistortedPinholeRsCameraGeometry;
  using ReprojectionError = aslam::backend::ReprojectionError<Geometry>;
  using Frame = aslam::Frame<Geometry>;
  using ReprojectionErrorSimple =
      aslam::backend::SimpleReprojectionError<Frame>;
  using DesignVariable = aslam::backend::CameraDesignVariable<Geometry>;
  using Distortion = aslam::cameras::EquidistantDistortion;
  using Shutter = aslam::cameras::RollingShutter;
  using ProjectionType = aslam::cameras::PinholeProjection<Distortion>;
};

struct FovPinhole {
  using Geometry = aslam::cameras::FovDistortedPinholeCameraGeometry;
  using ReprojectionError = aslam::backend::ReprojectionError<Geometry>;
  using Frame = aslam::Frame<Geometry>;
  using ReprojectionErrorSimple =
      aslam::backend::SimpleReprojectionError<Frame>;
  using DesignVariable = aslam::backend::CameraDesignVariable<Geometry>;
  using Distortion = aslam::cameras::FovDistortion;
  using Shutter = aslam::cameras::GlobalShutter;
  using ProjectionType = aslam::cameras::PinholeProjection<Distortion>;
};

struct ExtendedUnified {
  using Geometry = aslam::cameras::ExtendedUnifiedCameraGeometry;
  using ReprojectionError = aslam::backend::ReprojectionError<Geometry>;
  using Frame = aslam::Frame<Geometry>;
  using ReprojectionErrorSimple =
      aslam::backend::SimpleReprojectionError<Frame>;
  using DesignVariable = aslam::backend::CameraDesignVariable<Geometry>;
  using Distortion = aslam::cameras::NoDistortion;
  using Shutter = aslam::cameras::GlobalShutter;
  using ProjectionType = aslam::cameras::ExtendedUnifiedProjection<Distortion>;
};

struct DoubleSphere {
  using Geometry = aslam::cameras::DoubleSphereCameraGeometry;
  using ReprojectionError = aslam::backend::ReprojectionError<Geometry>;
  using Frame = aslam::Frame<Geometry>;
  using ReprojectionErrorSimple =
      aslam::backend::SimpleReprojectionError<Frame>;
  using DesignVariable = aslam::backend::CameraDesignVariable<Geometry>;
  using Distortion = aslam::cameras::NoDistortion;
  using Shutter = aslam::cameras::GlobalShutter;
  using ProjectionType = aslam::cameras::DoubleSphereProjection<Distortion>;
};

}  // namespace models

}  // namespace kalibr2

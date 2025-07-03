#pragma once

#include <aslam/cameras.hpp>
#include <aslam/Frame.hpp>
#include <aslam/backend/ReprojectionError.hpp>
#include <aslam/backend/SimpleReprojectionError.hpp>
#include <aslam/backend/CameraDesignVariable.hpp>


// Definition of useful structs for camera models in Kalibr2.
// The following structs define, for each camera model used in Kalibr2 calibration:
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
    typedef aslam::cameras::OmniCameraGeometry Geometry;
    typedef aslam::backend::ReprojectionError<Geometry> ReprojectionError;
    typedef aslam::Frame<Geometry> Frame;
    typedef aslam::backend::SimpleReprojectionError<Frame> ReprojectionErrorSimple;
    typedef aslam::backend::CameraDesignVariable<Geometry> DesignVariable;
    typedef aslam::cameras::NoDistortion Distortion;
    typedef aslam::cameras::GlobalShutter Shutter;
    typedef aslam::cameras::OmniProjection<Distortion> ProjectionType;
};


struct DistortedOmni {
    typedef aslam::cameras::DistortedOmniCameraGeometry Geometry;
    typedef aslam::backend::ReprojectionError<Geometry> ReprojectionError;
    typedef aslam::Frame<Geometry> Frame;
    typedef aslam::backend::SimpleReprojectionError<Frame> ReprojectionErrorSimple;
    typedef aslam::backend::CameraDesignVariable<Geometry> DesignVariable;
    typedef aslam::cameras::RadialTangentialDistortion Distortion;
    typedef aslam::cameras::GlobalShutter Shutter;
    typedef aslam::cameras::OmniProjection<Distortion> ProjectionType;
};


struct DistortedOmniRs {
    typedef aslam::cameras::DistortedOmniRsCameraGeometry Geometry;
    typedef aslam::backend::ReprojectionError<Geometry> ReprojectionError;
    typedef aslam::Frame<Geometry> Frame;
    typedef aslam::backend::SimpleReprojectionError<Frame> ReprojectionErrorSimple;
    typedef aslam::backend::CameraDesignVariable<Geometry> DesignVariable;
    typedef aslam::cameras::RadialTangentialDistortion Distortion;
    typedef aslam::cameras::RollingShutter Shutter;
    typedef aslam::cameras::OmniProjection<Distortion> ProjectionType;
};


struct DistortedPinhole {
    typedef aslam::cameras::DistortedPinholeCameraGeometry Geometry;
    typedef aslam::backend::ReprojectionError<Geometry> ReprojectionError;
    typedef aslam::Frame<Geometry> Frame;
    typedef aslam::backend::SimpleReprojectionError<Frame> ReprojectionErrorSimple;
    typedef aslam::backend::CameraDesignVariable<Geometry> DesignVariable;
    typedef aslam::cameras::RadialTangentialDistortion Distortion;
    typedef aslam::cameras::GlobalShutter Shutter;
    typedef aslam::cameras::PinholeProjection<Distortion> ProjectionType;
};


struct DistortedPinholeRs {
    typedef aslam::cameras::DistortedPinholeRsCameraGeometry Geometry;
    typedef aslam::backend::ReprojectionError<Geometry> ReprojectionError;
    typedef aslam::Frame<Geometry> Frame;
    typedef aslam::backend::SimpleReprojectionError<Frame> ReprojectionErrorSimple;
    typedef aslam::backend::CameraDesignVariable<Geometry> DesignVariable;
    typedef aslam::cameras::RadialTangentialDistortion Distortion;
    typedef aslam::cameras::RollingShutter Shutter;
    typedef aslam::cameras::PinholeProjection<Distortion> ProjectionType;
};


struct EquidistantPinhole {
    typedef aslam::cameras::EquidistantDistortedPinholeCameraGeometry Geometry;
    typedef aslam::backend::ReprojectionError<Geometry> ReprojectionError;
    typedef aslam::Frame<Geometry> Frame;
    typedef aslam::backend::SimpleReprojectionError<Frame> ReprojectionErrorSimple;
    typedef aslam::backend::CameraDesignVariable<Geometry> DesignVariable;
    typedef aslam::cameras::EquidistantDistortion Distortion;
    typedef aslam::cameras::GlobalShutter Shutter;
    typedef aslam::cameras::PinholeProjection<Distortion> ProjectionType;
};


struct EquidistantPinholeRs {
    typedef aslam::cameras::EquidistantDistortedPinholeRsCameraGeometry Geometry;
    typedef aslam::backend::ReprojectionError<Geometry> ReprojectionError;
    typedef aslam::Frame<Geometry> Frame;
    typedef aslam::backend::SimpleReprojectionError<Frame> ReprojectionErrorSimple;
    typedef aslam::backend::CameraDesignVariable<Geometry> DesignVariable;
    typedef aslam::cameras::EquidistantDistortion Distortion;
    typedef aslam::cameras::RollingShutter Shutter;
    typedef aslam::cameras::PinholeProjection<Distortion> ProjectionType;
};


struct FovPinhole {
    typedef aslam::cameras::FovDistortedPinholeCameraGeometry Geometry;
    typedef aslam::backend::ReprojectionError<Geometry> ReprojectionError;
    typedef aslam::Frame<Geometry> Frame;
    typedef aslam::backend::SimpleReprojectionError<Frame> ReprojectionErrorSimple;
    typedef aslam::backend::CameraDesignVariable<Geometry> DesignVariable;
    typedef aslam::cameras::FovDistortion Distortion;
    typedef aslam::cameras::GlobalShutter Shutter;
    typedef aslam::cameras::PinholeProjection<Distortion> ProjectionType;
};


struct ExtendedUnified {
    typedef aslam::cameras::ExtendedUnifiedCameraGeometry Geometry;
    typedef aslam::backend::ReprojectionError<Geometry> ReprojectionError;
    typedef aslam::Frame<Geometry> Frame;
    typedef aslam::backend::SimpleReprojectionError<Frame> ReprojectionErrorSimple;
    typedef aslam::backend::CameraDesignVariable<Geometry> DesignVariable;
    typedef aslam::cameras::NoDistortion Distortion;
    typedef aslam::cameras::GlobalShutter Shutter;
    typedef aslam::cameras::ExtendedUnifiedProjection<Distortion> ProjectionType;
};


struct DoubleSphere {
    typedef aslam::cameras::DoubleSphereCameraGeometry Geometry;
    typedef aslam::backend::ReprojectionError<Geometry> ReprojectionError;
    typedef aslam::Frame<Geometry> Frame;
    typedef aslam::backend::SimpleReprojectionError<Frame> ReprojectionErrorSimple;
    typedef aslam::backend::CameraDesignVariable<Geometry> DesignVariable;
    typedef aslam::cameras::NoDistortion Distortion;
    typedef aslam::cameras::GlobalShutter Shutter;
    typedef aslam::cameras::DoubleSphereProjection<Distortion> ProjectionType;
};

} // namespace models

} // namespace kalibr2

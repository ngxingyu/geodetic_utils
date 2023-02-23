//
// Created by mpantic on 11.06.19.
//

#ifndef GDAL_ROS_GEODETIC_CONVERTER_H
#define GDAL_ROS_GEODETIC_CONVERTER_H

#include <gdal/ogr_spatialref.h>
#include <gdal/cpl_conv.h>
#include <gdal/gdal_version.h>
#include <iostream>
#include <Eigen/Dense>
#include <optional>
#include <vector>
#include <memory>
#include <map>
#include <rclcpp/time.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>

namespace geotf {
class GeodeticConverter : public rclcpp::Node {
  typedef std::shared_ptr<OGRSpatialReference> OGRSpatialReferencePtr;
  typedef std::shared_ptr<OGRCoordinateTransformation>
      OGRCoordinateTransformationPtr;
  typedef std::pair<std::string, std::string> TransformId;

 public:
   explicit GeodeticConverter();

  // Adds a coordinate frame by its EPSG identifier
  // see https://spatialreference.org/ref/epsg/
  // Example: CH1903+ has epsg id 2056
  //          https://spatialreference.org/ref/epsg/2056/
  bool addFrameByEPSG(const std::string& name, const int& id);

  // Add a frame by UTM Zone
  // zone is the UTM zone (e.g. switzerland is in Zone 32)
  // north is true for northern hemisphere zones.
  bool addFrameByUTM(const std::string& name,
                     const uint zone,
                     const bool north);

  // Add a coordinate frame by a Geo Coordinate System
// Prominent example: WGS84
  bool addFrameByGCSCode(const std::string& name,
                         const std::string& gcscode);

  // Creates a new ENU Frame with its origin at the given
  // Location (lat, lon, alt)
  // Where (lat,lon,alt) are defined w.r.t. WGS84
  bool addFrameByENUOrigin(const std::string& name,
                           double lat,
                           double lon,
                           double alt);

  // Creates a new frame based on the Well-Known-Text definition
  // (can be obtained from spatialreference.org for example.)
  bool addFrameByWKT(const std::string& name, const std::string& wktformat);

  // Removes a frame if it exists.
  void removeFrame(const std::string& name);

  // Check whether a frame is set..
  bool hasFrame(const std::string& name);

  // Checks if two geo frames can be converted
  bool canConvert(const std::string& input_frame,
                  const std::string& output_frame);

  // Converts Pose from one input_frame to output_frame
  // Both frames are assumed to be geoframes
  // Currently, Attitude is not adjusted.
  bool convert(const std::string& input_frame,
               const Eigen::Affine3d& input,
               const std::string& output_frame,
               Eigen::Affine3d* output);

  // Converts Position from one input_frame to output_frame
  // Both frames are assumed to be geoframes
  bool convert(const std::string& input_frame,
               const Eigen::Vector3d& input,
               const std::string& output_frame,
               Eigen::Vector3d* output);

  // Convets a Pose in a geoframe to a pose in a tf frame
  bool convertToTf(const std::string& geo_input_frame,
                   const Eigen::Affine3d& input,
                   const std::string& tf_output_frame,
                   Eigen::Affine3d* output,
                   const rclcpp::Time& time = rclcpp::Time(0.0)) ;

  // Convets a Pose in a TF to a pose in a Geo frame
  bool convertFromTf(const std::string& tf_input_frame,
                     const Eigen::Affine3d& input,
                     const std::string& geo_output_frame,
                     Eigen::Affine3d* output,
                     const rclcpp::Time& time = rclcpp::Time(0.0)) ;

  // Publishes a geolocation as a tf frame
  bool publishAsTf(const std::string& geo_input_frame,
                   const Eigen::Vector3d& input,
                   const std::string& frame_name) ;

  // Publishes a geolocation as a tf frame
  bool publishAsTf(const std::string& geo_input_frame,
                   const Eigen::Affine3d& input,
                   const std::string& frame_name) ;

  // Writes a list of all frame definition to console
  void writeDebugInfo() const;

 private:
  bool getTransform(const std::string& input_frame,
                    const std::string& output_frame,
                    OGRCoordinateTransformationPtr* transform) ;

  bool checkTransform(const std::string& input_frame,
                      const std::string& output_frame);

  // Coordinate frame definitions
  std::map<const std::string, const OGRSpatialReferencePtr> mappings_;

  // Cordinate frame transformations
  std::map<const TransformId, const OGRCoordinateTransformationPtr> transforms_;

  // Altitude offsets for frames to convert to WGS84
  std::map<const std::string, const double> altitude_offsets_;

  //first = geotf frame, second = tf frame
  // Defines that these two frames (on a geo frame, on a tf frame)
  // are equal an can be used for geo<->TF conversions
  // Note: Geoframe must be a cartesian frame.
  std::optional<std::pair<std::string, std::string>>  tf_mapping_;

  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

};
}
#endif //GDAL_ROS_GEODETIC_CONVERTER_H

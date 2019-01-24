/**
   * Copyright (c) 2017, California Institute of Technology.
   * All rights reserved.
   *
   * Redistribution and use in source and binary forms, with or without
   * modification, are permitted provided that the following conditions are met:
   *
   * 1. Redistributions of source code must retain the above copyright notice,
   *    this list of conditions and the following disclaimer.
   * 2. Redistributions in binary form must reproduce the above copyright notice,
   *    this list of conditions and the following disclaimer in the documentation
   *    and/or other materials provided with the distribution.
   *
   * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
   * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   * POSSIBILITY OF SUCH DAMAGE.
   *
   * The views and conclusions contained in the software and documentation are
   * those of the authors and should not be interpreted as representing official
   * policies, either expressed or implied, of the California Institute of
   * Technology.
   *
   ** common_functions.h *********************************************************
   *
   * Wrapper classes for AprilTag standalone and bundle detection. Main function
   * is TagDetector::detectTags which wraps the call to core AprilTags 2
   * algorithm, apriltag_detector_detect().
   *
   * $Revision: 1.0 $
   * $Date: 2017/12/17 13:23:14 $
   * $Author: dmalyuta $
   *
   * Originator:        Danylo Malyuta, JPL
   ******************************************************************************/

#ifndef APRILTAGS2_ROS_COMMON_FUNCTIONS_H
#define APRILTAGS2_ROS_COMMON_FUNCTIONS_H

/////////////////////////////////////////
//Three method for estiming camera pose//
#define DIRECT_USING_IPPE 5
#define PLANE_POSE_ESTIMATION 4
#define POSE_ESTIMATION 3
#define BUNDLE_ADJUSTER_OPTIMIZATION 2
#define USING_KALMAN_FILTER 1
#define DIRECT_USING_SOLVER_PNP 0
/////////////////////////////////////////

#define OUTPUT_DEBUG_INFO_TO_FILE 0

#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros/console.h>
#include <XmlRpcException.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>

#include "apriltags2_ros/AprilTagDetection.h"
#include "apriltags2_ros/AprilTagDetectionArray.h"
#include "apriltags2_ros/AprilTagPosition.h"

#include "CommonHeader.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <yaml-cpp/yaml.h>
#include "ippe.h"
#include <pnc_msgs/VehicleInfo.h>
#include <localization_msgs/StopPoint.h>
#include <localization_msgs/StopPoints.h>
#include <center_msgs/VehicleTarget.h>
#include <center_msgs/Ack.h>

#include "apriltags2/apriltag.h"
#include "apriltags2/common/homography.h"
#include "apriltags2/tag36h11.h"
#include "apriltags2/tag36h10.h"
#include "apriltags2/tag25h9.h"
#include "apriltags2/tag25h7.h"
#include "apriltags2/tag16h5.h"

namespace apriltags2_ros
{

template <typename T>
T getAprilTagOption(ros::NodeHandle &pnh,
                    const std::string &param_name, const T &default_val)
{
    T param_val;
    pnh.param<T>(param_name, param_val, default_val);
    return param_val;
}

// Stores the properties of a tag member of a bundle
struct TagBundleMember
{
    int id;           // Payload ID
    double size;      // [m] Side length
    cv::Matx44d T_oi; // Rigid transform from tag i frame to bundle origin frame
};

class StandaloneTagDescription
{
  public:
    StandaloneTagDescription(){};
    StandaloneTagDescription(int id, double size,
                             std::string &frame_name) : id_(id),
                                                        size_(size),
                                                        frame_name_(frame_name) {}

    double size() { return size_; }
    int id() { return id_; }
    std::string &frame_name() { return frame_name_; }

  private:
    // Tag description
    int id_;
    double size_;
    std::string frame_name_;
};

class TagBundleDescription
{
  public:
    std::map<int, int> id2idx_; // (id2idx_[<tag ID>]=<index in tags_>) mapping

    TagBundleDescription(std::string name) : name_(name) {}

    void addMemberTag(int id, double size, cv::Matx44d T_oi)
    {
        TagBundleMember member;
        member.id = id;
        member.size = size;
        member.T_oi = T_oi;
        tags_.push_back(member);
        id2idx_[id] = tags_.size() - 1;
    }

    std::string name() const { return name_; }
    // Get IDs of bundle member tags
    std::vector<int> bundleIds()
    {
        std::vector<int> ids;
        for (unsigned int i = 0; i < tags_.size(); i++)
        {
            ids.push_back(tags_[i].id);
        }
        return ids;
    }
    // Get sizes of bundle member tags
    std::vector<double> bundleSizes()
    {
        std::vector<double> sizes;
        for (unsigned int i = 0; i < tags_.size(); i++)
        {
            sizes.push_back(tags_[i].size);
        }
        return sizes;
    }
    int memberID(int tagID) { return tags_[id2idx_[tagID]].id; }
    double memberSize(int tagID) { return tags_[id2idx_[tagID]].size; }
    cv::Matx44d memberT_oi(int tagID) { return tags_[id2idx_[tagID]].T_oi; }

  private:
    // Bundle description
    std::string name_;
    std::vector<TagBundleMember> tags_;
};

class TagDetector
{
  public:
    // Detections sorting
    static int idComparison(const void *first, const void *second);

    // Remove detections of tags with the same ID
    void removeDuplicates();

    // AprilTags 2 code's attributes
    std::string family_;
    int border_;
    int threads_;
    double decimate_;
    double blur_;
    int refine_edges_;
    int refine_decode_;
    int refine_pose_;
    int debug_;
    int opt_method_;
    int tag_rotation_;
    int translation_index_;
    double direction_;

    // AprilTags 2 objects
    apriltag_family_t *tf_;
    apriltag_detector_t *td_;
    zarray_t *detections_;

    // Other members
    std::map<int, StandaloneTagDescription> standalone_tag_descriptions_;
    std::vector<TagBundleDescription> tag_bundle_descriptions_;
    bool run_quietly_;
    bool publish_tf_;

    std::string camera_which_;
    std::string camera_tf_frame_;


  public:
    TagDetector(ros::NodeHandle &pnh);
    ~TagDetector();

    double x_pos_now_;
    double z_distance_;
    int tag_id_;
    double distance_shift_;

    int missing_detection_count_;
    cv::KalmanFilter kf_pos_;

    double dt_;           // time between measurements (1/FPS)
    
    // Store standalone and bundle tag descriptions
    std::map<int, StandaloneTagDescription> parseStandaloneTags(
        XmlRpc::XmlRpcValue &standalone_tag_descriptions);
    std::vector<TagBundleDescription> parseTagBundles(
        XmlRpc::XmlRpcValue &tag_bundles);
    double xmlRpcGetDouble(
        XmlRpc::XmlRpcValue &xmlValue, std::string field) const;
    double xmlRpcGetDoubleWithDefault(
        XmlRpc::XmlRpcValue &xmlValue, std::string field,
        double defaultValue) const;

    bool findStandaloneTagDescription(
        int id, StandaloneTagDescription *&descriptionContainer,
        bool printWarning = true);

    geometry_msgs::PoseWithCovarianceStamped makeTagPose(
        const Eigen::Matrix4d &transform,
        const Eigen::Quaternion<double> rot_quaternion,
        const std_msgs::Header &header);

    // Detect tags in an image
    AprilTagDetectionArray detectTags(
        const cv_bridge::CvImagePtr &image,
        const sensor_msgs::CameraInfoConstPtr &camera_info);

    bool detectTags(
        const cv_bridge::CvImagePtr &image,
        const sensor_msgs::CameraInfoConstPtr &camera_info, localization_msgs::StopPoints &stopPoints);


    // Get the pose of the tag in the camera frame
    // Returns homogeneous transformation matrix [R,t;[0 0 0 1]] which
    // takes a point expressed in the tag frame to the same point
    // expressed in the camera frame. As usual, R is the (passive)
    // rotation from the tag frame to the camera frame and t is the
    // vector from the camera frame origin to the tag frame origin,
    // expressed in the camera frame.
    Eigen::Matrix4d getRelativeTransform(
        std::vector<cv::Point3d> objectPoints,
        std::vector<cv::Point2d> imagePoints,
        double fx, double fy, double cx, double cy) ;

    void addImagePoints(apriltag_detection_t *detection,
                        std::vector<cv::Point2d> &imagePoints) ;

    void addObjectPoints(double s, cv::Matx44d T_oi,
                         std::vector<cv::Point3d> &objectPoints) ;

    // Draw the detected tags' outlines and payload values on the image
    void drawDetections(cv_bridge::CvImagePtr image);

    void calcBoardCornerPositions(cv::Size boardSize, double squareSize, std::vector<cv::Point3d> &corners);
    bool detectChessboard(const cv_bridge::CvImagePtr &image,
                          const sensor_msgs::CameraInfoConstPtr &camera_info, Eigen::Matrix4d &T);

    void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt); 
    void fillMeasurements( cv::Mat &measurements,
                   const cv::Mat &translation_measured, const cv::Mat &rotation_measured);      
    void updateKalmanFilter(cv::KalmanFilter &KF, cv::Mat &measurement,
                     cv::Mat &translation_estimated, cv::Mat &rotation_estimated );  
    cv::Mat euler2rot(const cv::Mat &euler);
    cv::Mat rot2euler(const cv::Mat &rotationMatrix);                

};

} // namespace apriltags2_ros

#endif // APRILTAGS2_ROS_COMMON_FUNCTIONS_H

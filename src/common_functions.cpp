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
 */

#include "apriltags2_ros/common_functions.h"

namespace apriltags2_ros
{

TagDetector::TagDetector(ros::NodeHandle &pnh) : family_(getAprilTagOption<std::string>(pnh, "tag_family", "tag36h11")),
                                                 border_(getAprilTagOption<int>(pnh, "tag_border", 1)),
                                                 threads_(getAprilTagOption<int>(pnh, "tag_threads", 4)),
                                                 decimate_(getAprilTagOption<double>(pnh, "tag_decimate", 1.0)),
                                                 blur_(getAprilTagOption<double>(pnh, "tag_blur", 0.0)),
                                                 refine_edges_(getAprilTagOption<int>(pnh, "tag_refine_edges", 1)),
                                                 refine_decode_(getAprilTagOption<int>(pnh, "tag_refine_decode", 0)),
                                                 refine_pose_(getAprilTagOption<int>(pnh, "tag_refine_pose", 0)),
                                                 debug_(getAprilTagOption<int>(pnh, "tag_debug", 0)),
                                                 publish_tf_(getAprilTagOption<bool>(pnh, "publish_tf", false)),
                                                 dt_(0.062)
{
    XmlRpc::XmlRpcValue standalone_tag_descriptions;
    x_pos_now_ = 100.0;
    if (!pnh.getParam("standalone_tags", standalone_tag_descriptions))
    {
        ROS_WARN("No april tags specified");
    }
    else
    {
        try
        {
            standalone_tag_descriptions_ =
                parseStandaloneTags(standalone_tag_descriptions);
        }
        catch (XmlRpc::XmlRpcException e)
        {
            // in case any of the asserts in parseStandaloneTags() fail
            ROS_ERROR_STREAM("Error loading standalone tag descriptions: " << e.getMessage().c_str());
        }
    }

    // parse tag bundle descriptions specified by user (stored on ROS parameter
    // server)
    XmlRpc::XmlRpcValue tag_bundle_descriptions;
    if (!pnh.getParam("tag_bundles", tag_bundle_descriptions))
    {
        ROS_WARN("No tag bundles specified");
    }
    else
    {
        try
        {
            tag_bundle_descriptions_ = parseTagBundles(tag_bundle_descriptions);
        }
        catch (XmlRpc::XmlRpcException e)
        {
            // In case any of the asserts in parseStandaloneTags() fail
            ROS_ERROR_STREAM("Error loading tag bundle descriptions: " << e.getMessage().c_str());
        }
    }

    // Define the tag family whose tags should be searched for in the camera
    // images
    if (family_ == "tag36h11")
    {
        tf_ = tag36h11_create();
    }
    else if (family_ == "tag36h10")
    {
        tf_ = tag36h10_create();
    }
    else if (family_ == "tag25h9")
    {
        tf_ = tag25h9_create();
    }
    else if (family_ == "tag25h7")
    {
        tf_ = tag25h7_create();
    }
    else if (family_ == "tag16h5")
    {
        tf_ = tag16h5_create();
    }
    else
    {
        ROS_WARN("Invalid tag family specified! Aborting");
        exit(1);
    }
    tf_->black_border = (uint32_t)border_;

    // Create the AprilTags 2 detector
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_);
    
    td_->quad_decimate = (float)decimate_;
    td_->quad_sigma = (float)blur_;
    td_->nthreads = threads_;
    td_->debug = debug_;
    td_->refine_edges = refine_edges_;
    td_->refine_decode = refine_decode_;
    td_->refine_pose = refine_pose_;

    // Get tf frame name to use for the camera
    if (!pnh.getParam("camera_frame", camera_tf_frame_))
    {
        ROS_WARN_STREAM("Camera frame not specified, using 'camera'");
        camera_tf_frame_ = "camera";
    }

    if (!pnh.getParam("which", camera_which_))
    {
        ROS_WARN_STREAM("Camera coordinate not specified, using 'right camera coordinate'");
        camera_which_ = "right";
    }

    if (!pnh.getParam("tag_rotation", tag_rotation_))
    {
        ROS_WARN_STREAM("tag rotation angle not specified, using '0 as anti-clockwise'");
        tag_rotation_ = 0;
    }
    
    if (!pnh.getParam("opt_method", opt_method_))
    {
        ROS_WARN_STREAM("opt_method not specified, setting 'direct using solvepnp method'");
        opt_method_ = 4;
    }

    std::cout << "parameter which_camera: " << camera_which_ << std::endl;
    std::cout << "parameter tag decimate: " << decimate_ << std::endl;
    std::cout << "parameter tag threads: " << threads_ << std::endl;

    distance_shift_ = 0.0;
    missing_detection_count_ = 0;

    direction_ = 1.0;
    switch(tag_rotation_)
    {
    case 0:
    {
        translation_index_ = 0;
        direction_ = -1.;
        break;
    }
    case 1:
    {
        translation_index_ = 1;
        direction_ = 1.;
        break;
    }
    case 2:
    {
        translation_index_ = 0;
        direction_ = 1.;
        break;
    }
    case 3:
    {
        translation_index_ = 1;
        direction_ = -1.;
        break;
    }
    default:
        break;
    }
    if (camera_which_.compare("right") == 0)
    {
        switch (tag_rotation_)
        {
        case 0:
        {
            translation_index_ = 0;
            direction_ = 1.;
            break;
        }
        case 1:
        {
            translation_index_ = 1;
            direction_ = -1.;
            break;
        }
        case 2:
        {
            translation_index_ = 0;
            direction_ = -1.;
            break;
        }
        case 3:
        {
            translation_index_ = 1;
            direction_ = 1.;
            break;
        }
        default:
            break;
        }
    }

    //x,y,z ---> world point
    //  [1  0  0  dt   0  0]
    //  [0  1  0  0   dt  0]
    //  [0  0  1  0   0  dt]
    //  [0  0  0  1   0   0]
    //  [0  0  0  0   1   0]
    //  [0  0  0  0   0   1]


    /* MEASUREMENT MODEL */
    //  [1 0 0 0 0 0]
    //  [0 1 0 0 0 0]
    //  [0 0 1 0 0 0]
    kf_pos_.init(6, 3, 0, CV_64F);               // init Kalman Filter
    cv::setIdentity(kf_pos_.processNoiseCov, cv::Scalar::all(1e-5));     // set process noise
    cv::setIdentity(kf_pos_.measurementNoiseCov, cv::Scalar::all(1e-4)); // set measurement noise
    cv::setIdentity(kf_pos_.errorCovPost, cv::Scalar::all(1));           // error covariance

    kf_pos_.transitionMatrix.at<double>(0, 3) = dt_;
    kf_pos_.transitionMatrix.at<double>(1, 4) = dt_;
    kf_pos_.transitionMatrix.at<double>(2, 5) = dt_;

    kf_pos_.measurementMatrix.at<double>(0, 0) = 1;  // x
    kf_pos_.measurementMatrix.at<double>(1, 1) = 1;  // y
    kf_pos_.measurementMatrix.at<double>(2, 2) = 1;  // z
}

// destructor
TagDetector::~TagDetector()
{
    // free memory associated with tag detector
    apriltag_detector_destroy(td_);

    // Free memory associated with the array of tag detections
    zarray_destroy(detections_);

    // free memory associated with tag family
    if (family_ == "tag36h11")
    {
        tag36h11_destroy(tf_);
    }
    else if (family_ == "tag36h10")
    {
        tag36h10_destroy(tf_);
    }
    else if (family_ == "tag25h9")
    {
        tag25h9_destroy(tf_);
    }
    else if (family_ == "tag25h7")
    {
        tag25h7_destroy(tf_);
    }
    else if (family_ == "tag16h5")
    {
        tag16h5_destroy(tf_);
    }
}

AprilTagDetectionArray TagDetector::detectTags(
    const cv_bridge::CvImagePtr &image,
    const sensor_msgs::CameraInfoConstPtr &camera_info)
{
    // Convert image to AprilTag code's format
    cv::Mat gray_image;
    cv::cvtColor(image->image, gray_image, CV_BGR2GRAY);
    image_u8_t apriltags2_image = {.width = gray_image.cols,
                                   .height = gray_image.rows,
                                   .stride = gray_image.cols,
                                   .buf = gray_image.data};

    // Get camera intrinsic properties
    double fx = camera_info->K[0]; // focal length in camera x-direction [px]
    double fy = camera_info->K[4]; // focal length in camera y-direction [px]
    double cx = camera_info->K[2]; // optical center x-coordinate [px]
    double cy = camera_info->K[5]; // optical center y-coordinate [px]

    // Run AprilTags 2 algorithm on the image
    detections_ = apriltag_detector_detect(td_, &apriltags2_image);

    // Restriction: any tag ID can appear at most once in the scene. Thus, get all
    // the tags visible in the scene and remove any tags with IDs of which there
    // are multiple in the scene
    removeDuplicates();

    // Compute the estimated translation and rotation individually for each
    // detected tag
    AprilTagDetectionArray tag_detection_array;
    std::vector<std::string> detection_names;
    tag_detection_array.header = image->header;
    std::map<std::string, std::vector<cv::Point3d>> bundleObjectPoints;
    std::map<std::string, std::vector<cv::Point2d>> bundleImagePoints;
    for (int i = 0; i < zarray_size(detections_); i++)
    {
        // Get the i-th detected tag
        apriltag_detection_t *detection;
        zarray_get(detections_, i, &detection);

        // Bootstrap this for loop to find this tag's description amongst
        // the tag bundles. If found, add its points to the bundle's set of
        // object-image corresponding points (tag corners) for cv::solvePnP.
        // Don't yet run cv::solvePnP on the bundles, though, since we're still in
        // the process of collecting all the object-image corresponding points
        int tagID = detection->id;
        bool is_part_of_bundle = false;
        for (unsigned int j = 0; j < tag_bundle_descriptions_.size(); j++)
        {
            // Iterate over the registered bundles
            TagBundleDescription bundle = tag_bundle_descriptions_[j];

            if (bundle.id2idx_.find(tagID) != bundle.id2idx_.end())
            {
                // This detected tag belongs to the j-th tag bundle (its ID was found in
                // the bundle description)
                is_part_of_bundle = true;
                std::string bundleName = bundle.name();

                //===== Corner points in the world frame coordinates
                double s = bundle.memberSize(tagID) / 2;
                addObjectPoints(s, bundle.memberT_oi(tagID),
                                bundleObjectPoints[bundleName]);

                //===== Corner points in the image frame coordinates
                addImagePoints(detection, bundleImagePoints[bundleName]);
            }
        }

        // Find this tag's description amongst the standalone tags
        // Print warning when a tag was found that is neither part of a
        // bundle nor standalone (thus it is a tag in the environment
        // which the user specified no description for, or Apriltags
        // misdetected a tag (bad ID or a false positive)).
        StandaloneTagDescription *standaloneDescription;
        if (!findStandaloneTagDescription(tagID, standaloneDescription,
                                          !is_part_of_bundle))
        {
            continue;
        }

        //=================================================================
        // The remainder of this for loop is concerned with standalone tag
        // poses!
        double tag_size = standaloneDescription->size();

        // Get estimated tag pose in the camera frame.
        //
        // Note on frames:
        // The raw AprilTags 2 uses the following frames:
        //   - camera frame: looking from behind the camera (like a
        //     photographer), x is right, y is up and z is towards you
        //     (i.e. the back of camera)
        //   - tag frame: looking straight at the tag (oriented correctly),
        //     x is right, y is down and z is away from you (into the tag).
        // But we want:
        //   - camera frame: looking from behind the camera (like a
        //     photographer), x is right, y is down and z is straight
        //     ahead
        //   - tag frame: looking straight at the tag (oriented correctly),
        //     x is right, y is up and z is towards you (out of the tag).
        // Using these frames together with cv::solvePnP directly avoids
        // AprilTag 2's frames altogether.
        // TODO solvePnP[Ransac] better?
        std::vector<cv::Point3d> standaloneTagObjectPoints;
        std::vector<cv::Point2d> standaloneTagImagePoints;
        addObjectPoints(tag_size / 2, cv::Matx44d::eye(), standaloneTagObjectPoints);
        addImagePoints(detection, standaloneTagImagePoints);
        Eigen::Matrix4d transform = getRelativeTransform(standaloneTagObjectPoints,
                                                         standaloneTagImagePoints,
                                                         fx, fy, cx, cy);
        Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
        Eigen::Quaternion<double> rot_quaternion(rot);

        geometry_msgs::PoseWithCovarianceStamped tag_pose =
            makeTagPose(transform, rot_quaternion, image->header);

        // Add the detection to the back of the tag detection array
        AprilTagDetection tag_detection;
        tag_detection.pose = tag_pose;
        //    tag_detection.pose_inv = tag_pose_inv;
        tag_detection.id.push_back(detection->id);
        tag_detection.size.push_back(tag_size);
        tag_detection_array.detections.push_back(tag_detection);
        detection_names.push_back(standaloneDescription->frame_name());
    }

    // =================================================================
    //Estimate bundle origin pose for each bundle in which at least one
    //member tag was detected

    for (unsigned int j = 0; j < tag_bundle_descriptions_.size(); j++)
    {
        // Get bundle name
        std::string bundleName = tag_bundle_descriptions_[j].name();

        std::map<std::string,
                 std::vector<cv::Point3d>>::iterator it =
            bundleObjectPoints.find(bundleName);
        if (it != bundleObjectPoints.end())
        {
            // Some member tags of this bundle were detected, get the bundle's
            // position!
            TagBundleDescription &bundle = tag_bundle_descriptions_[j];

            Eigen::Matrix4d transform =
                getRelativeTransform(bundleObjectPoints[bundleName],
                                     bundleImagePoints[bundleName], fx, fy, cx, cy);
            Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
            Eigen::Quaternion<double> rot_quaternion(rot);

            geometry_msgs::PoseWithCovarianceStamped bundle_pose =
                makeTagPose(transform, rot_quaternion, image->header);

            // Add the detection to the back of the tag detection array
            AprilTagDetection tag_detection;
            tag_detection.pose = bundle_pose;
            tag_detection.id = bundle.bundleIds();
            tag_detection.size = bundle.bundleSizes();
            tag_detection_array.detections.push_back(tag_detection);
            detection_names.push_back(bundle.name());
        }
    }

    // If set, publish the transform /tf topic
    // if (publish_tf_)
    // {
    //     for (unsigned int i = 0; i < tag_detection_array.detections.size(); i++)
    //     {
    //         geometry_msgs::PoseStamped pose;
    //         pose.pose = tag_detection_array.detections[i].pose.pose.pose;
    //         pose.header = tag_detection_array.detections[i].pose.header;
    //         tf::Stamped<tf::Transform> tag_transform;
    //         tf::poseStampedMsgToTF(pose, tag_transform);
    //         tf_pub_.sendTransform(tf::StampedTransform(tag_transform,
    //                                                    tag_transform.stamp_,
    //                                                    camera_tf_frame_,
    //                                                    detection_names[i]));
    //     }
    // }

    return tag_detection_array;
}

bool TagDetector::detectTags(
    const cv_bridge::CvImagePtr &image,
    const sensor_msgs::CameraInfoConstPtr &camera_info,
    localization_msgs::StopPoints &stopPoints)
{

    // Get camera intrinsic properties
    double fx = camera_info->K[0]; // focal length in camera x-direction [px]
    double fy = camera_info->K[4]; // focal length in camera y-direction [px]
    double cx = camera_info->K[2]; // optical center x-coordinate [px]
    double cy = camera_info->K[5]; // optical center y-coordinate [px]

    cv::Mat cameraMatrix(3, 3, CV_64FC1);
    cameraMatrix.setTo(0);
    cameraMatrix.at<double>(0, 0) = camera_info->K[0];
    cameraMatrix.at<double>(1, 1) = camera_info->K[4];
    cameraMatrix.at<double>(0, 2) = camera_info->K[2];
    cameraMatrix.at<double>(1, 2) = camera_info->K[5];
    cameraMatrix.at<double>(2, 2) = 1.0;

    //setup distortion vector:
    cv::Mat distCoeffs(5, 1, CV_64FC1);
    distCoeffs.at<double>(0, 0) = camera_info->D[0];
    distCoeffs.at<double>(1, 0) = camera_info->D[1];
    distCoeffs.at<double>(2, 0) = camera_info->D[2];
    distCoeffs.at<double>(3, 0) = camera_info->D[3];
    distCoeffs.at<double>(4, 0) = camera_info->D[4];

    // Convert image to AprilTag code's format
    cv::Mat gray_image, refineGrayCopy;
    cv::Mat origin_image;
    origin_image = image->image.clone();
    cv::cvtColor(image->image, refineGrayCopy, CV_BGR2GRAY);
    cv::undistort(refineGrayCopy, gray_image, cameraMatrix, distCoeffs);

    image_u8_t apriltags2_image = {.width = gray_image.cols,
                                   .height = gray_image.rows,
                                   .stride = gray_image.cols,
                                   .buf = gray_image.data};

    bool flag = false;
    stopPoints.header = image->header;

    // Run AprilTags 2 algorithm on the image
    detections_ = apriltag_detector_detect(td_, &apriltags2_image);

    // Restriction: any tag ID can appear at most once in the scene. Thus, get all
    // the tags visible in the scene and remove any tags with IDs of which there
    // are multiple in the scene
    removeDuplicates();

    // Compute the estimated translation and rotation individually for each
    // detected tag
    AprilTagDetectionArray tag_detection_array;
    std::vector<std::string> detection_names;
    tag_detection_array.header = image->header;
    std::map<std::string, std::vector<cv::Point3d>> bundleObjectPoints;
    std::map<std::string, std::vector<cv::Point2d>> bundleImagePoints;

    if(missing_detection_count_ >= 20)
    {
        kf_pos_.init(6, 3, 0, CV_64F);               // init Kalman Filter
        cv::setIdentity(kf_pos_.processNoiseCov, cv::Scalar::all(1e-5));     // set process noise
        cv::setIdentity(kf_pos_.measurementNoiseCov, cv::Scalar::all(1e-4)); // set measurement noise
        cv::setIdentity(kf_pos_.errorCovPost, cv::Scalar::all(1));           // error covariance

        kf_pos_.transitionMatrix.at<double>(0, 3) = dt_;
        kf_pos_.transitionMatrix.at<double>(1, 4) = dt_;
        kf_pos_.transitionMatrix.at<double>(2, 5) = dt_;

        kf_pos_.measurementMatrix.at<double>(0, 0) = 1;  // x
        kf_pos_.measurementMatrix.at<double>(1, 1) = 1;  // y
        kf_pos_.measurementMatrix.at<double>(2, 2) = 1;  // z
        missing_detection_count_ = 20;
    }

    for (int i = 0; i < zarray_size(detections_); i++)
    {
        // Get the i-th detected tag
        apriltag_detection_t *detection;
        zarray_get(detections_, i, &detection);

        // Bootstrap this for loop to find this tag's description amongst
        // the tag bundles. If found, add its points to the bundle's set of
        // object-image corresponding points (tag corners) for cv::solvePnP.
        // Don't yet run cv::solvePnP on the bundles, though, since we're still in
        // the process of collecting all the object-image corresponding points
        int tagID = detection->id;
        bool is_part_of_bundle = false;
        std::cout << "tag_id: " << tagID << std::endl;

        // Find this tag's description amongst the standalone tags
        // Print warning when a tag was found that is neither part of a
        // bundle nor standalone (thus it is a tag in the environment
        // which the user specified no description for, or Apriltags
        // misdetected a tag (bad ID or a false positive)).

        StandaloneTagDescription *standaloneDescription;
        if (!findStandaloneTagDescription(tagID, standaloneDescription,
                                          !is_part_of_bundle))
        {
            continue;
        }

        //=================================================================
        // The remainder of this for loop is concerned with standalone tag
        // poses!
        missing_detection_count_ = 0;
        
        double tag_size = standaloneDescription->size();

        std::vector<cv::Point3d> standaloneTagObjectPoints;
        std::vector<cv::Point2d> standaloneTagImagePoints;

        addObjectPoints(tag_size / 2, cv::Matx44d::eye(), standaloneTagObjectPoints);
        addImagePoints(detection, standaloneTagImagePoints);

        Eigen::Matrix4d transform = getRelativeTransform(standaloneTagObjectPoints,
                                                         standaloneTagImagePoints, //refinecorners2, //standaloneTagImagePoints,
                                                         fx, fy, cx, cy);
        std::cout << "translation_index:" << translation_index_ << std::endl;

        if (opt_method_ == DIRECT_USING_SOLVER_PNP)
        {

            Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
            Eigen::Matrix3d rot_inv = rot.inverse();
            Eigen::Vector3d translation = transform.block(0, 3, 3, 1);
            Eigen::Vector3d translation_inv = -1. * rot_inv * translation;

            //using kalman filter
            cv::Mat measurements(3, 1, CV_64F);
            measurements.at<double>(0) = translation_inv(0);
            measurements.at<double>(1) = translation_inv(1);
            measurements.at<double>(2) = translation_inv(2);
            cv::Mat prediction = kf_pos_.predict();
            //The "correct" phase that is going to use the predicted value and our measurement
            cv::Mat estimated = kf_pos_.correct(measurements);
            std::cout << "id: " << detection->id << "," << "dis: " << estimated.at<double>(translation_index_) << "," << "z: " << estimated.at<double>(2) << std::endl;
            x_pos_now_ = estimated.at<double>(translation_index_) - distance_shift_;
            z_distance_ = estimated.at<double>(2);

            //std::cout << "id: " << detection->id << "," << "dis: " << translation_inv(translation_index_) << "," << "z: " << translation_inv(2) << std::endl;
            //x_pos_now_ = translation_inv(translation_index_) - distance_shift_;
            //z_distance_ = translation_inv(2);

            localization_msgs::StopPoint stopPoint;
            stopPoint.distance_shift = direction_ * x_pos_now_;
            stopPoint.distance_vertical = z_distance_;
            stopPoint.id = detection->id;
            stopPoint.type = 1; //apriltag
            stopPoints.stop_points.push_back(stopPoint);
            flag = true;
            return flag;
        }
    }
    missing_detection_count_++;
    return flag;
}

int TagDetector::idComparison(const void *first, const void *second)
{
    int id1 = ((apriltag_detection_t *)first)->id;
    int id2 = ((apriltag_detection_t *)second)->id;
    return (id1 < id2) ? -1 : ((id1 == id2) ? 0 : 1);
}

void TagDetector::removeDuplicates()
{
    zarray_sort(detections_, &idComparison);
    int count = 0;
    bool duplicate_detected = false;
    while (true)
    {
        if (count > zarray_size(detections_) - 1)
        {
            // The entire detection set was parsed
            return;
        }
        apriltag_detection_t *detection;
        zarray_get(detections_, count, &detection);
        int id_current = detection->id;
        // Default id_next value of -1 ensures that if the last detection
        // is a duplicated tag ID, it will get removed
        int id_next = -1;
        if (count < zarray_size(detections_) - 1)
        {
            zarray_get(detections_, count + 1, &detection);
            id_next = detection->id;
        }
        if (id_current == id_next || (id_current != id_next && duplicate_detected))
        {
            duplicate_detected = true;
            // Remove the current tag detection from detections array
            int shuffle = 0;
            zarray_remove_index(detections_, count, shuffle);
            if (id_current != id_next)
            {
                ROS_WARN_STREAM("Pruning tag ID " << id_current << " because it "
                                                                   "appears more than once in the image.");
                duplicate_detected = false; // Reset
            }
            continue;
        }
        else
        {
            count++;
        }
    }
}

void TagDetector::addObjectPoints(
    double s, cv::Matx44d T_oi, std::vector<cv::Point3d> &objectPoints) 
{
    // Add to object point vector the tag corner coordinates in the bundle frame
    // Going counterclockwise starting from the bottom left corner
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, -s, 0, 1));
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, -s, 0, 1));
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, s, 0, 1));
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, s, 0, 1));
}

void TagDetector::addImagePoints(
    apriltag_detection_t *detection,
    std::vector<cv::Point2d> &imagePoints) 
{
    // Add to image point vector the tag corners in the image frame
    // Going counterclockwise starting from the bottom left corner
    double tag_x[4] = {-1, 1, 1, -1};
    double tag_y[4] = {1, 1, -1, -1}; // Negated because AprilTag tag local
                                      // frame has y-axis pointing DOWN
                                      // while we use the tag local frame
                                      // with y-axis pointing UP

    
    for (int i = 0; i < 4; i++)
    {
        // Homography projection taking tag local frame coordinates to image pixels
        double im_x, im_y;
        homography_project(detection->H, tag_x[i], tag_y[i], &im_x, &im_y);
        
        //ROS_INFO("image_point: %f, %f", tag_x[i],tag_y[i]);
        imagePoints.push_back(cv::Point2d(im_x, im_y));
    }
}

Eigen::Matrix4d TagDetector::getRelativeTransform(
    std::vector<cv::Point3d> objectPoints,
    std::vector<cv::Point2d> imagePoints,
    double fx, double fy, double cx, double cy) 
{
    // perform Perspective-n-Point camera pose estimation using the
    // above 3D-2D point correspondences
    cv::Mat rvec, tvec;
    cv::Matx33d cameraMatrix(fx, 0, cx,
                             0, fy, cy,
                             0, 0, 1);
    cv::Vec4f distCoeffs(0, 0, 0, 0); // distortion coefficients
    // TODO Perhaps something like SOLVEPNP_EPNP would be faster? Would
    // need to first check WHAT is a bottleneck in this code, and only
    // do this if PnP solution is the bottleneck.
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

    //  cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, 400,
    //            1.0, 0.99, cv::noArray(), cv::SOLVEPNP_EPNP);

    cv::Matx33d R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d wRo;
    wRo << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);

    Eigen::Matrix4d T; // homogeneous transformation matrix
    T.topLeftCorner(3, 3) = wRo;
    T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
    T.row(3) << 0, 0, 0, 1;

#if OUTPUT_DEBUG_INFO_TO_FILE
     Eigen::Quaternion<double> quaternion_ori(wRo);
     std::ofstream os4;
     os4.open("/home/xd/quaternion_ori.txt", std::ios::out | std::ios::app);
     os4 << quaternion_ori.w() << "," << quaternion_ori.x() << "," << quaternion_ori.y() << ","
         << quaternion_ori.z() << std::endl;
     os4.close();
#endif
     return T;
}

geometry_msgs::PoseWithCovarianceStamped TagDetector::makeTagPose(
    const Eigen::Matrix4d &transform,
    const Eigen::Quaternion<double> rot_quaternion,
    const std_msgs::Header &header)
{
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header = header;
    //===== Position and orientation
    pose.pose.pose.position.x = transform(0, 3);
    pose.pose.pose.position.y = transform(1, 3);
    pose.pose.pose.position.z = transform(2, 3);
    pose.pose.pose.orientation.x = rot_quaternion.x();
    pose.pose.pose.orientation.y = rot_quaternion.y();
    pose.pose.pose.orientation.z = rot_quaternion.z();
    pose.pose.pose.orientation.w = rot_quaternion.w();
    return pose;
}

void TagDetector::drawDetections(cv_bridge::CvImagePtr image)
{
    for (int i = 0; i < zarray_size(detections_); i++)
    {
        apriltag_detection_t *det;
        zarray_get(detections_, i, &det);

        // Check if this ID is present in config/tags.yaml
        // Check if is part of a tag bundle
        int tagID = det->id;
        bool is_part_of_bundle = false;
        for (unsigned int j = 0; j < tag_bundle_descriptions_.size(); j++)
        {
            TagBundleDescription bundle = tag_bundle_descriptions_[j];
            if (bundle.id2idx_.find(tagID) != bundle.id2idx_.end())
            {
                is_part_of_bundle = true;
                break;
            }
        }
        // If not part of a bundle, check if defined as a standalone tag
        StandaloneTagDescription *standaloneDescription;
        if (!is_part_of_bundle &&
            !findStandaloneTagDescription(tagID, standaloneDescription, false))
        {
            // Neither a standalone tag nor part of a bundle, so this is a "rogue"
            // tag, skip it.
            continue;
        }

        // Draw tag outline with edge colors green, blue, blue, red
        // (going counter-clockwise, starting from lower-left corner in
        // tag coords). cv::Scalar(Blue, Green, Red) format for the edge
        // colors!
        line(image->image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
             cv::Point((int)det->p[1][0], (int)det->p[1][1]),
             cv::Scalar(0, 0xff, 0)); // green
        line(image->image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
             cv::Point((int)det->p[3][0], (int)det->p[3][1]),
             cv::Scalar(0, 0, 0xff)); // red
        line(image->image, cv::Point((int)det->p[1][0], (int)det->p[1][1]),
             cv::Point((int)det->p[2][0], (int)det->p[2][1]),
             cv::Scalar(0xff, 0, 0)); // blue
        line(image->image, cv::Point((int)det->p[2][0], (int)det->p[2][1]),
             cv::Point((int)det->p[3][0], (int)det->p[3][1]),
             cv::Scalar(0xff, 0, 0)); // blue

        // Print tag ID in the middle of the tag
        std::stringstream ss;
        ss << det->id;
        cv::String text = ss.str();
        int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontscale = 0.5;
        int baseline;
        cv::Size textsize = cv::getTextSize(text, fontface,
                                            fontscale, 2, &baseline);
        cv::putText(image->image, text,
                    cv::Point((int)(det->c[0] - textsize.width / 2),
                              (int)(det->c[1] + textsize.height / 2)),
                    fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
    }
}

// Parse standalone tag descriptions
std::map<int, StandaloneTagDescription> TagDetector::parseStandaloneTags(
    XmlRpc::XmlRpcValue &standalone_tags)
{
    // Create map that will be filled by the function and returned in the end
    std::map<int, StandaloneTagDescription> descriptions;
    // Ensure the type is correct
    ROS_ASSERT(standalone_tags.getType() == XmlRpc::XmlRpcValue::TypeArray);
    // Loop through all tag descriptions
    for (int32_t i = 0; i < standalone_tags.size(); i++)
    {

        // i-th tag description
        XmlRpc::XmlRpcValue &tag_description = standalone_tags[i];

        // Assert the tag description is a struct
        ROS_ASSERT(tag_description.getType() ==
                   XmlRpc::XmlRpcValue::TypeStruct);
        // Assert type of field "id" is an int
        ROS_ASSERT(tag_description["id"].getType() ==
                   XmlRpc::XmlRpcValue::TypeInt);
        // Assert type of field "size" is a double
        ROS_ASSERT(tag_description["size"].getType() ==
                   XmlRpc::XmlRpcValue::TypeDouble);

        int id = (int)tag_description["id"]; // tag id
        // Tag size (square, side length in meters)
        double size = (double)tag_description["size"];

        // Custom frame name, if such a field exists for this tag
        std::string frame_name;
        if (tag_description.hasMember("name"))
        {
            // Assert type of field "name" is a string
            ROS_ASSERT(tag_description["name"].getType() ==
                       XmlRpc::XmlRpcValue::TypeString);
            frame_name = (std::string)tag_description["name"];
        }
        else
        {
            std::stringstream frame_name_stream;
            frame_name_stream << "tag_" << id;
            frame_name = frame_name_stream.str();
        }

        StandaloneTagDescription description(id, size, frame_name);
        ROS_INFO_STREAM("Loaded tag config: " << id << ", size: " << size << ", frame_name: " << frame_name.c_str());
        // Add this tag's description to map of descriptions
        descriptions.insert(std::make_pair(id, description));
    }

    return descriptions;
}

// parse tag bundle descriptions
std::vector<TagBundleDescription> TagDetector::parseTagBundles(
    XmlRpc::XmlRpcValue &tag_bundles)
{
    std::vector<TagBundleDescription> descriptions;
    ROS_ASSERT(tag_bundles.getType() == XmlRpc::XmlRpcValue::TypeArray);

    // Loop through all tag bundle descritions
    for (int32_t i = 0; i < tag_bundles.size(); i++)
    {
        ROS_ASSERT(tag_bundles[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        // i-th tag bundle description
        XmlRpc::XmlRpcValue &bundle_description = tag_bundles[i];

        std::string bundleName;
        if (bundle_description.hasMember("name"))
        {
            ROS_ASSERT(bundle_description["name"].getType() ==
                       XmlRpc::XmlRpcValue::TypeString);
            bundleName = (std::string)bundle_description["name"];
        }
        else
        {
            std::stringstream bundle_name_stream;
            bundle_name_stream << "bundle_" << i;
            bundleName = bundle_name_stream.str();
        }
        TagBundleDescription bundle_i(bundleName);
        ROS_INFO("Loading tag bundle '%s'", bundle_i.name().c_str());

        ROS_ASSERT(bundle_description["layout"].getType() ==
                   XmlRpc::XmlRpcValue::TypeArray);
        XmlRpc::XmlRpcValue &member_tags = bundle_description["layout"];

        // Loop through each member tag of the bundle
        for (int32_t j = 0; j < member_tags.size(); j++)
        {
            ROS_ASSERT(member_tags[j].getType() == XmlRpc::XmlRpcValue::TypeStruct);
            XmlRpc::XmlRpcValue &tag = member_tags[j];

            ROS_ASSERT(tag["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
            int id = tag["id"];

            ROS_ASSERT(tag["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            double size = tag["size"];

            // Make sure that if this tag was specified also as standalone,
            // then the sizes match
            StandaloneTagDescription *standaloneDescription;
            if (findStandaloneTagDescription(id, standaloneDescription, false))
            {
                ROS_ASSERT(size == standaloneDescription->size());
            }

            // Get this tag's pose with respect to the bundle origin
            double x = xmlRpcGetDoubleWithDefault(tag, "x", 0.);
            double y = xmlRpcGetDoubleWithDefault(tag, "y", 0.);
            double z = xmlRpcGetDoubleWithDefault(tag, "z", 0.);
            double qw = xmlRpcGetDoubleWithDefault(tag, "qw", 1.);
            double qx = xmlRpcGetDoubleWithDefault(tag, "qx", 0.);
            double qy = xmlRpcGetDoubleWithDefault(tag, "qy", 0.);
            double qz = xmlRpcGetDoubleWithDefault(tag, "qz", 0.);
            Eigen::Quaterniond q_tag(qw, qx, qy, qz);
            q_tag.normalize();
            Eigen::Matrix3d R_oi = q_tag.toRotationMatrix();

            // Build the rigid transform from tag_j to the bundle origin
            cv::Matx44d T_mj(R_oi(0, 0), R_oi(0, 1), R_oi(0, 2), x,
                             R_oi(1, 0), R_oi(1, 1), R_oi(1, 2), y,
                             R_oi(2, 0), R_oi(2, 1), R_oi(2, 2), z,
                             0, 0, 0, 1);

            // Register the tag member
            bundle_i.addMemberTag(id, size, T_mj);
            ROS_INFO_STREAM(" " << j << ") id: " << id << ", size: " << size << ", "
                                << "p = [" << x << "," << y << "," << z << "], "
                                << "q = [" << qw << "," << qx << "," << qy << ","
                                << qz << "]");
        }
        descriptions.push_back(bundle_i);
    }
    return descriptions;
}

double TagDetector::xmlRpcGetDouble(XmlRpc::XmlRpcValue &xmlValue,
                                    std::string field) const
{
    ROS_ASSERT((xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
               (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
    if (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        int tmp = xmlValue[field];
        return (double)tmp;
    }
    else
    {
        return xmlValue[field];
    }
}

double TagDetector::xmlRpcGetDoubleWithDefault(XmlRpc::XmlRpcValue &xmlValue,
                                               std::string field,
                                               double defaultValue) const
{
    if (xmlValue.hasMember(field))
    {
        ROS_ASSERT((xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
                   (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
        if (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
            int tmp = xmlValue[field];
            return (double)tmp;
        }
        else
        {
            return xmlValue[field];
        }
    }
    else
    {
        return defaultValue;
    }
}

bool TagDetector::findStandaloneTagDescription(
    int id, StandaloneTagDescription *&descriptionContainer, bool printWarning)
{
    std::map<int, StandaloneTagDescription>::iterator description_itr =
        standalone_tag_descriptions_.find(id);
    if (description_itr == standalone_tag_descriptions_.end())
    {
        if (printWarning)
        {
            ROS_WARN_THROTTLE(10.0, "Requested description of standalone tag ID [%d],"
                                    " but no description was found...",
                              id);
        }
        return false;
    }
    descriptionContainer = &(description_itr->second);
    return true;
}

void TagDetector::calcBoardCornerPositions(cv::Size boardSize, double squareSize, std::vector<cv::Point3d> &corners)
{
    corners.clear();
    for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
            corners.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
}

bool TagDetector::detectChessboard(const cv_bridge::CvImagePtr &image,
                                   const sensor_msgs::CameraInfoConstPtr &camera_info, Eigen::Matrix4d &T)
{
    // Convert image to AprilTag code's format
    cv::Mat gray_image;
    cv::cvtColor(image->image, gray_image, CV_BGR2GRAY);

    double fx = camera_info->K[0]; // focal length in camera x-direction [px]
    double fy = camera_info->K[4]; // focal length in camera y-direction [px]
    double cx = camera_info->K[2]; // optical center x-coordinate [px]
    double cy = camera_info->K[5]; // optical center y-coordinate [px]

    cv::Mat rvec, tvec;
    cv::Matx33d cameraMatrix(fx, 0, cx,
                             0, fy, cy,
                             0, 0, 1);
    cv::Vec4f distCoeffs(0, 0, 0, 0); // distortion coefficients

    cv::Size boardSize = cv::Size(7, 5);

    std::vector<cv::Point2d> imagePoints;
    bool found = cv::findChessboardCorners(image->image, boardSize, imagePoints,
                                           cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
                                               cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found)
    {

        cv::cornerSubPix(gray_image, imagePoints, boardSize,
                         cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        cv::drawChessboardCorners(image->image, boardSize, cv::Mat(imagePoints), found);

        std::vector<float> reprojErrs;

        std::vector<cv::Point3d> objectPoints;
        double squareSize = 11.2;
        calcBoardCornerPositions(boardSize, squareSize, objectPoints);

        if (objectPoints.size() != imagePoints.size())
        {
            //Find intrinsic and extrinsic camera parameters
            //      double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
            //                                       distCoeffs, rvecs, tvecs, s.flag|CALIB_FIX_K4|CALIB_FIX_K5);
            return false;
        }

        // TODO Perhaps something like SOLVEPNP_EPNP would be faster? Would
        // need to first check WHAT is a bottleneck in this code, and only
        // do this if PnP solution is the bottleneck.
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
        cv::Matx33d R;
        cv::Rodrigues(rvec, R);
        Eigen::Matrix3d wRo;
        wRo << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);

        // homogeneous transformation matrix

        T.topLeftCorner(3, 3) = wRo;
        T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
        T.row(3) << 0, 0, 0, 1;
        ROS_INFO_STREAM(
            "tvec = [" << tvec.at<double>(0) << "," << tvec.at<double>(1) << "," << tvec.at<double>(2) << "]");
        return true;
    }
    else
        return false;
}

void TagDetector::initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
{
    KF.init(nStates, nMeasurements, nInputs, CV_64F);               // init Kalman Filter
    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));     // set process noise
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-4)); // set measurement noise
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));           // error covariance
                                                                    /* DYNAMIC MODEL */
                                                                    //pose, v, a
    //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]

    // KF.transitionMatrix.at<double>(0, 3) = dt;
    // KF.transitionMatrix.at<double>(1, 4) = dt;
    // KF.transitionMatrix.at<double>(2, 5) = dt;
    // KF.transitionMatrix.at<double>(3, 6) = dt;
    // KF.transitionMatrix.at<double>(4, 7) = dt;
    // KF.transitionMatrix.at<double>(5, 8) = dt;
    // KF.transitionMatrix.at<double>(0, 6) = 0.5 * pow(dt, 2);
    // KF.transitionMatrix.at<double>(1, 7) = 0.5 * pow(dt, 2);
    // KF.transitionMatrix.at<double>(2, 8) = 0.5 * pow(dt, 2);
    // // orientation
    // KF.transitionMatrix.at<double>(9, 12) = dt;
    // KF.transitionMatrix.at<double>(10, 13) = dt;
    // KF.transitionMatrix.at<double>(11, 14) = dt;
    // KF.transitionMatrix.at<double>(12, 15) = dt;
    // KF.transitionMatrix.at<double>(13, 16) = dt;
    // KF.transitionMatrix.at<double>(14, 17) = dt;
    // KF.transitionMatrix.at<double>(9, 15) = 0.5 * pow(dt, 2);
    // KF.transitionMatrix.at<double>(10, 16) = 0.5 * pow(dt, 2);
    // KF.transitionMatrix.at<double>(11, 17) = 0.5 * pow(dt, 2);
    /* MEASUREMENT MODEL */
    //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
    // KF.measurementMatrix.at<double>(0, 0) = 1;  // x
    // KF.measurementMatrix.at<double>(1, 1) = 1;  // y
    // KF.measurementMatrix.at<double>(2, 2) = 1;  // z
    // KF.measurementMatrix.at<double>(3, 9) = 1;  // roll
    // KF.measurementMatrix.at<double>(4, 10) = 1; // pitch
    // KF.measurementMatrix.at<double>(5, 11) = 1; // yaw

    
    //pose, v
    //  [1 0 0 dt  0  0   0  0  0   0   0   0  0  0]
    //  [0 1 0  0 dt  0   0  0  0   0   0   0  0  0]
    //  [0 0 1  0  0 dt   0  0  0   0   0   0  0  0]
    //  [0 0 0  1  0  0   0  0  0   0   0   0  0  0]
    //  [0 0 0  0  1  0   0  0  0   0   0   0  0  0]
    //  [0 0 0  0  0  1   0  0  0   0   0   0  0  0]
    //  [0 0 0  0  0  0   1  0  0   0  dt   0  0  0]
    //  [0 0 0  0  0  0   0  1  0   0   0  dt  0  0]
    //  [0 0 0  0  0  0   0  0  1   0   0  0  dt  0]
    //  [0 0 0  0  0  0   0  0  0   1   0   0  0 dt]
    //  [0 0 0  0  0  0   0  0  0   0   1   0  0  0]
    //  [0 0 0  0  0  0   0  0  0   0   0   1  0  0]
    //  [0 0 0  0  0  0   0  0  0   0   0   0  1  0]
    //  [0 0 0  0  0  0   0  0  0   0   0   0  0  1]

    /* MEASUREMENT MODEL */
    //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 1 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 1 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 1 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0]

        //pose, v
    //  [1 0 0 dt  0  0   0  0  0   0]
    //  [0 1 0  0 dt  0   0  0  0   0]
    //  [0 0 1  0  0 dt   0  0  0   0]
    //  [0 0 0  1  0  0   0  0  0   0]
    //  [0 0 0  0  1  0   0  0  0   0]
    //  [0 0 0  0  0  1   0  0  0   0]
    //  [0 0 0  0  0  0   1  0  0   0]
    //  [0 0 0  0  0  0   0  1  0   0]
    //  [0 0 0  0  0  0   0  0  1   0]
    //  [0 0 0  0  0  0   0  0  0   1]

    /* MEASUREMENT MODEL */
    //  [1 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 1 0 0 0]
    //  [0 0 0 0 0 0 0 1 0 0]
    //  [0 0 0 0 0 0 0 0 1 0]
    //  [0 0 0 0 0 0 0 0 0 1]



    //pose, v ->>> euler angle
    //  [1 0 0 dt  0  0   0  0  0   0   0   0]
    //  [0 1 0  0 dt  0   0  0  0   0   0   0]
    //  [0 0 1  0  0 dt   0  0  0   0   0   0]
    //  [0 0 0  1  0  0   0  0  0   0   0   0]
    //  [0 0 0  0  1  0   0  0  0   0   0   0]
    //  [0 0 0  0  0  1   0  0  0   0   0   0]
    //  [0 0 0  0  0  0   1  0  0   dt  0   0]
    //  [0 0 0  0  0  0   0  1  0   0   dt  0]
    //  [0 0 0  0  0  0   0  0  1   0   0  dt]
    //  [0 0 0  0  0  0   0  0  0   1   0   0]
    //  [0 0 0  0  0  0   0  0  0   0   1   0]
    //  [0 0 0  0  0  0   0  0  0   0   0   1]


    /* MEASUREMENT MODEL */
    //  [1 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 1 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 1 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 1 0 0 0]



      //x,y ---> image point
    //  [1  0  dt   0]
    //  [0  1  0   dt]
    //  [0  0  1    0]
    //  [0  0  0    1]

    /* MEASUREMENT MODEL */
    //  [1 0 0 0]
    //  [0 1 0 0]

    KF.transitionMatrix.at<double>(0, 2) = dt;
    KF.transitionMatrix.at<double>(1, 3) = dt;

    KF.measurementMatrix.at<double>(0, 0) = 1;  // x
    KF.measurementMatrix.at<double>(1, 1) = 1;  // y
}

void TagDetector::fillMeasurements( cv::Mat &measurements,
                   const cv::Mat &translation_measured, const cv::Mat &rotation_measured)
{
    // Convert rotation matrix to euler angles

    // Eigen::Matrix3d wRo;
    // wRo << rotation_measured.at<double>(0, 0), rotation_measured.at<double>(0, 1), rotation_measured.at<double>(0, 2), rotation_measured.at<double>(1, 0), rotation_measured.at<double>(1, 1), 
    //                               rotation_measured.at<double>(1, 2), rotation_measured.at<double>(2, 0), rotation_measured.at<double>(2, 1), rotation_measured.at<double>(2, 2);
    // Eigen::Quaternion<double> rot_quaternion(wRo);
    cv::Mat measured_eulers(3, 1, CV_64F);
    measured_eulers = rot2euler(rotation_measured);
    //Set measurement to predict
    measurements.at<double>(0) = translation_measured.at<double>(0); // x
    measurements.at<double>(1) = translation_measured.at<double>(1); // y
    measurements.at<double>(2) = translation_measured.at<double>(2); // z
    measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
    measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
    measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw
    // measurements.at<double>(3) = rot_quaternion.w();      // quaternion.w
    // measurements.at<double>(4) = rot_quaternion.x();      // quaternion.x
    // measurements.at<double>(5) = rot_quaternion.y();      // quaternion.y
    // measurements.at<double>(6) = rot_quaternion.z();      // quaternion.z
 }

void TagDetector::updateKalmanFilter( cv::KalmanFilter &KF, cv::Mat &measurement,
                     cv::Mat &translation_estimated, cv::Mat &rotation_estimated )
{
    // First predict, to update the internal statePre variable
    cv::Mat prediction = KF.predict();
    // The "correct" phase that is going to use the predicted value and our measurement
    cv::Mat estimated = KF.correct(measurement);
    // Estimated translation
    translation_estimated.at<double>(0) = estimated.at<double>(0);
    translation_estimated.at<double>(1) = estimated.at<double>(1);
    translation_estimated.at<double>(2) = estimated.at<double>(2);
    // Estimated euler angles
    // cv::Mat eulers_estimated(3, 1, CV_64F);
    // eulers_estimated.at<double>(0) = estimated.at<double>(9);
    // eulers_estimated.at<double>(1) = estimated.at<double>(10);
    // eulers_estimated.at<double>(2) = estimated.at<double>(11);

    cv::Mat eulers_estimated(3, 1, CV_64F);
    eulers_estimated.at<double>(0) = estimated.at<double>(6);
    eulers_estimated.at<double>(1) = estimated.at<double>(7);
    eulers_estimated.at<double>(2) = estimated.at<double>(8);

    rotation_estimated = euler2rot(eulers_estimated);

    // Eigen::Quaternion<double> quaternion_rotation(estimated.at<double>(6), estimated.at<double>(7), 
    //                                             estimated.at<double>(8), estimated.at<double>(9));
    
    // // Convert estimated quaternion to rotation matrix
    // cv::Mat rotationMatrix(3, 3, CV_64F);
    // Eigen::Matrix3d wRo = quaternion_rotation.toRotationMatrix();
    // rotationMatrix.at<double>(0, 0) = wRo(0, 0);
    // rotationMatrix.at<double>(0, 1) = wRo(0, 1);
    // rotationMatrix.at<double>(0, 2) = wRo(0, 2);
    // rotationMatrix.at<double>(1, 0) = wRo(1, 0);
    // rotationMatrix.at<double>(1, 1) = wRo(1, 1);
    // rotationMatrix.at<double>(1, 2) = wRo(1, 2);
    // rotationMatrix.at<double>(2, 0) = wRo(2, 0);
    // rotationMatrix.at<double>(2, 1) = wRo(2, 1);
    // rotationMatrix.at<double>(2, 2) = wRo(2, 2);
    // rotation_estimated = rotationMatrix;
}

cv::Mat TagDetector::rot2euler(const cv::Mat &rotationMatrix)
{
    cv::Mat euler(3, 1, CV_64F);

    double m00 = rotationMatrix.at<double>(0, 0);
    double m02 = rotationMatrix.at<double>(0, 2);
    double m10 = rotationMatrix.at<double>(1, 0);
    double m11 = rotationMatrix.at<double>(1, 1);
    double m12 = rotationMatrix.at<double>(1, 2);
    double m20 = rotationMatrix.at<double>(2, 0);
    double m22 = rotationMatrix.at<double>(2, 2);

    double bank, attitude, heading;

    // Assuming the angles are in radians.
    if (m10 > 0.998)
    { // singularity at north pole
        bank = 0;
        attitude = CV_PI / 2;
        heading = atan2(m02, m22);
    }
    else if (m10 < -0.998)
    { // singularity at south pole
        bank = 0;
        attitude = -CV_PI / 2;
        heading = atan2(m02, m22);
    }
    else
    {
        bank = atan2(-m12, m11);
        attitude = asin(m10);
        heading = atan2(-m20, m00);
    }

    euler.at<double>(0) = bank;
    euler.at<double>(1) = attitude;
    euler.at<double>(2) = heading;

    return euler;
}

// Converts a given Euler angles to Rotation Matrix
// Convention used is Y-Z-X Tait-Bryan angles
// Reference:
// https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm
cv::Mat TagDetector::euler2rot(const cv::Mat &euler)
{
    cv::Mat rotationMatrix(3, 3, CV_64F);

    double bank = euler.at<double>(0);
    double attitude = euler.at<double>(1);
    double heading = euler.at<double>(2);

    // Assuming the angles are in radians.
    double ch = cos(heading);
    double sh = sin(heading);
    double ca = cos(attitude);
    double sa = sin(attitude);
    double cb = cos(bank);
    double sb = sin(bank);

    double m00, m01, m02, m10, m11, m12, m20, m21, m22;

    m00 = ch * ca;
    m01 = sh * sb - ch * sa * cb;
    m02 = ch * sa * sb + sh * cb;
    m10 = sa;
    m11 = ca * cb;
    m12 = -ca * sb;
    m20 = -sh * ca;
    m21 = sh * sa * cb + ch * sb;
    m22 = -sh * sa * sb + ch * cb;

    rotationMatrix.at<double>(0, 0) = m00;
    rotationMatrix.at<double>(0, 1) = m01;
    rotationMatrix.at<double>(0, 2) = m02;
    rotationMatrix.at<double>(1, 0) = m10;
    rotationMatrix.at<double>(1, 1) = m11;
    rotationMatrix.at<double>(1, 2) = m12;
    rotationMatrix.at<double>(2, 0) = m20;
    rotationMatrix.at<double>(2, 1) = m21;
    rotationMatrix.at<double>(2, 2) = m22;

    return rotationMatrix;
}

} // namespace apriltags2_ros

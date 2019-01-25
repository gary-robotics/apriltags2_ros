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

#include "apriltags2_ros/continuous_detector.h"
#include "../include/apriltags2_ros/continuous_detector.h"
#include "../include/apriltags2_ros/common_functions.h"
//#include <image_transport.h>
#include <fstream>

namespace apriltags2_ros {

    ContinuousDetector::ContinuousDetector(ros::NodeHandle &nh,
                                           ros::NodeHandle &pnh) :
            tag_detector_(pnh),
            draw_tag_detections_image_(false),
//        getAprilTagOption<bool>(pnh, "publish_tag_detections_image", false)),
            it_(nh) {
//  camera_image_subscriber_ = nh.subscribe("image_rect", 1, &ContinuousDetector::imageCallback, this);
        camera_image_subscriber_ = it_.subscribeCamera("image_rect", 1,
                                                       &ContinuousDetector::imageCallback, this);

        pnc_vehicle_info_sub_ = nh.subscribe("/pnc_msgs/vehicle_info", 1, &ContinuousDetector::do_pnc_vehicle_info_Msg, this);
        center_vehicle_target_sub_ = nh.subscribe("/center_msgs/vehicle_target", 1, &ContinuousDetector::do_center_vehicle_target_Msg, this);


        time_duration_ = 0.01;
        detect_flag_ = false;
        z_distance__ = 0.0;
        velocity_ = 0.0;

        tag_detections_publisher_ =
                nh.advertise<localization_msgs::StopPoints>("tag_localization", 1);
        tag_detections_publisher2_ =
                nh.advertise<AprilTagPosition>("tag_position", 1);

        tag_ack_publisher_ =
                nh.advertise<center_msgs::Ack>("tag_ack", 1);

        timer_ = nh.createTimer(ros::Duration(time_duration_), &ContinuousDetector::timerCallback, this);        

        if (draw_tag_detections_image_) {
            tag_detections_image_publisher_ = it_.advertise("tag_detections_image", 1);
        }

    }

    void ContinuousDetector::timerCallback(const ros::TimerEvent &event) {

         if (detect_flag_) {
            mutex_.lock();
            for (std::size_t i = 0; i < stopPoints_.stop_points.size(); i++) {
                stopPoints_.stop_points[i].distance_shift -= velocity_ * time_duration_;  // inspvax 100hz
            }
            mutex_.unlock();
            tag_detections_publisher_.publish(stopPoints_);
        }
    }

    void ContinuousDetector::do_pnc_vehicle_info_Msg(const pnc_msgs::VehicleInfo& msg) {

        double v = msg.vehicle_speed;
//        std::cout << "vechile speed:" << v << std::endl;
        if(fabs(v) < 0.2) 
        {
            velocity_ = 0.0;
        }
    }

    void ContinuousDetector::do_center_vehicle_target_Msg(const center_msgs::VehicleTarget& msg){

        tag_detector_.tag_id_ = msg.stop_tag_id;
        tag_detector_.distance_shift_ = msg.tag_stop_shift;
        center_msgs::Ack ack_msg;
        //ack_msg.header = msg.header;
        ack_msg.header.stamp = ros::Time::now();
        ack_msg.chid = msg.chid;
        ack_msg.recv_packet_id = msg.send_packet_id;
        tag_ack_publisher_.publish(ack_msg);
    }

    void ContinuousDetector::imageCallback(
            const sensor_msgs::ImageConstPtr &image_rect,
            const sensor_msgs::CameraInfoConstPtr &camera_info) {
        // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
        // AprilTags 2 on the iamge

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now(); //Just for time measurement
        try {
            cv_image_ = cv_bridge::toCvCopy(image_rect,
                                            sensor_msgs::image_encodings::BGR8);//BGR8
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        mutex_.lock();
        stopPoints_.stop_points.clear();
        mutex_.unlock();
        detect_flag_ = tag_detector_.detectTags(cv_image_, camera_info, stopPoints_);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double time_duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

        if (detect_flag_) {
            mutex_.lock();

            for (std::size_t i = 0; i < stopPoints_.stop_points.size(); i++) {
                stopPoints_.stop_points[i].distance_shift -= velocity_ * (0.05 + time_duration /
                                                                                 1000000.0);  // 图像采集频率为20hz，所以增加50毫秒延时,tag 检测时间为time_duration
            }
            mutex_.unlock();
        }
        ROS_INFO("image process FPS = %f", 1000000.0 / time_duration); //Just for time measurement
    }

} // namespace apriltags2_ros

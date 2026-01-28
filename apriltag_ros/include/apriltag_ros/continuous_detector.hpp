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
 ** continuous_detector.h ******************************************************
 *
 * Wrapper class of TagDetector class which calls TagDetector::detectTags on
 * each newly arrived image published by a camera.
 *
 * $Revision: 1.0 $
 * $Date: 2017/12/17 13:25:52 $
 * $Author: dmalyuta $
 *
 * Originator:        Danylo Malyuta, JPL
 ******************************************************************************/

#ifndef APRILTAG_ROS_CONTINUOUS_DETECTOR_HPP
#define APRILTAG_ROS_CONTINUOUS_DETECTOR_HPP

// C++
#include <memory>
#include <mutex>
#include <unordered_map>

#include "apriltag_ros/common_functions.hpp"
#include "apriltag_ros/composition_visibility.h"

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "std_msgs/msg/bool.hpp"

#include <isaac_ros_managed_nitros/managed_nitros_subscriber.hpp>
#include <isaac_ros_nitros_image_type/nitros_image_view.hpp>

// Own
#include "apriltag_ros_interfaces/msg/april_tag_detection.hpp"
#include "apriltag_ros_interfaces/msg/april_tag_detection_array.hpp"

#include "apriltag_ros/camera_component.hpp"
#include "lpv_interfaces/msg/apriltag_observer.hpp"

namespace apriltag_ros
{

class ContinuousDetector
{
    public:
        using DiagnosticArrayMsg = diagnostic_msgs::msg::DiagnosticArray;
        using DiagnosticStatusMsg = diagnostic_msgs::msg::DiagnosticStatus;
        using ApriltagToggleMsg = lpv_interfaces::msg::ApriltagObserver;
        using AprilTagDetectionArray = apriltag_ros_interfaces::msg::AprilTagDetectionArray;

    	COMPOSITION_PUBLIC 
        explicit ContinuousDetector(const rclcpp::NodeOptions & options);
        COMPOSITION_PUBLIC
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
        get_node_base_interface() const;

        void ImageCallback(const nvidia::isaac_ros::nitros::NitrosImageView & view);

        void CameraInfoCallback(
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

        void ApriltagToggleCallback(
            const ApriltagToggleMsg::ConstSharedPtr & msg);
            
        bool detection_enabled;

        rclcpp::Node::SharedPtr nh_;
        rclcpp::QoS custom_qos_;

        std::mutex detection_mutex_;
        std::shared_ptr<TagDetector> tag_detector_;
        bool draw_tag_detections_image_;
        cv_bridge::CvImagePtr cv_image_;

        std::string camera_position;
        std::shared_ptr<uint32_t> target_id;
        std::unordered_map<CameraPosition, std::shared_ptr<CameraComponent>> camera_map;
        std::shared_ptr<std::unordered_map<CameraPosition, bool>> target_tag_detected_map;
        std::vector<CameraPosition> cameras;

        bool tag_detected;
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
            nvidia::isaac_ros::nitros::NitrosImageView>> nitros_sub_;
        std::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::CameraSubscriber camera_image_subscriber_;
        rclcpp::Subscription<ApriltagToggleMsg>::SharedPtr apriltag_toggle_sub_;
        image_transport::Publisher tag_detections_image_publisher_;
        rclcpp::Publisher<apriltag_ros_interfaces::msg::AprilTagDetectionArray>::SharedPtr tag_detections_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_reset_publisher_;

};

} // namespace apriltag_ros

#endif // APRILTAG_ROS_CONTINUOUS_DETECTOR_HPP

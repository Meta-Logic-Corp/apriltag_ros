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

#include "apriltag_ros/continuous_detector.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


using namespace apriltag_ros;

ContinuousDetector::ContinuousDetector(const rclcpp::NodeOptions & options)
: nh_(std::make_shared<rclcpp::Node>("apriltag_node", options)), custom_qos_(1)
{
    using std::placeholders::_1;
    rclcpp::uninstall_signal_handlers();

    // Declare and get parameters
    draw_tag_detections_image_ = nh_->declare_parameter<bool>("publish_tag_detections_image", false);
    std::string tag_detections_image_topic = nh_->declare_parameter<std::string>("tag_detections_image_topic", "tag_detections_image");
    std::string image_topic = nh_->declare_parameter<std::string>("image_topic", "color/image_raw");
    int queue_size = nh_->declare_parameter<int>("queue_size", 1);
    std::string tag_detections_topic = nh_->declare_parameter<std::string>("tag_detections_topic", "tag_detections");
    std::string state_diag_topic = nh_->declare_parameter<std::string>("state_diag_topic", "state_diag");
    std::string apriltag_toggle_topic = nh_->declare_parameter<std::string>("apriltag_toggle_topic", "apriltag_toggle");
    camera_position = nh_->declare_parameter<std::string>("camera_position", "camera_position");
    tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(nh_));
    detection_enabled = false;
    RCLCPP_INFO(nh_->get_logger(), "Starting apriltag_ros ContinuousDetector node with image topic: %s", image_topic.c_str());

    cameras = {
        CameraPosition::FRONT,
        CameraPosition::BACK,
        CameraPosition::LEFT,
        CameraPosition::RIGHT
    };

    target_id = std::make_shared<uint32_t>(ApriltagToggleMsg::TAG36_H11_0);

    target_tag_detected_map = std::make_shared<std::unordered_map<CameraPosition, bool>>();
    target_tag_detected_map->insert({CameraPosition::FRONT, false});
    target_tag_detected_map->insert({CameraPosition::BACK, false});
    target_tag_detected_map->insert({CameraPosition::LEFT, false});
    target_tag_detected_map->insert({CameraPosition::RIGHT, false});

    camera_map.insert({CameraPosition::FRONT,
         std::make_shared<CameraComponent>(nh_, CameraPosition::FRONT, tag_detector_, target_tag_detected_map, target_id)});
    camera_map.insert({CameraPosition::BACK,
         std::make_shared<CameraComponent>(nh_, CameraPosition::BACK, tag_detector_, target_tag_detected_map, target_id)});
    camera_map.insert({CameraPosition::LEFT,
         std::make_shared<CameraComponent>(nh_, CameraPosition::LEFT, tag_detector_, target_tag_detected_map, target_id)});
    camera_map.insert({CameraPosition::RIGHT,
         std::make_shared<CameraComponent>(nh_, CameraPosition::RIGHT, tag_detector_, target_tag_detected_map, target_id)});
    // Image_transport
    it_ = std::shared_ptr<image_transport::ImageTransport>(
        new image_transport::ImageTransport(nh_));

    camera_reset_publisher_ = nh_->create_publisher<std_msgs::msg::Bool>(
        "/front/camera_reset", 10);

    apriltag_toggle_sub_ = nh_->create_subscription<ApriltagToggleMsg>(apriltag_toggle_topic, 10,
        std::bind(&ContinuousDetector::ApriltagToggleCallback, this , std::placeholders::_1));

    if (draw_tag_detections_image_)
    {
        tag_detections_image_publisher_ = it_->advertise(tag_detections_image_topic, 1);
    }

}

void ContinuousDetector::CameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
    camera_info_ = msg;
}

void ContinuousDetector::ApriltagToggleCallback(
    const ApriltagToggleMsg::ConstSharedPtr& msg)
{
    // Check if the state is ready to detect tags
    if (detection_enabled == false && msg->spin_observer == true)
    {
        detection_enabled = msg->spin_observer;
        *target_id = msg->tag_id;
        RCLCPP_INFO(nh_->get_logger(), "Set target tag ID to %d", *target_id);
        for (auto const& camera : cameras){
            camera_map[camera]->detection_enabled = true;
        }
        tag_detected = false;
    }
    else if (detection_enabled == true && msg->spin_observer == false)
    {
        detection_enabled = false;
        for (auto const& camera : cameras){
            camera_map[camera]->detection_enabled = false;
            if (target_tag_detected_map->at(camera)) {
                tag_detected = true;
            }
            target_tag_detected_map->at(camera) = false;
            camera_map[camera]->tag_detected = false;
        }
        if (!tag_detected && camera_position == msg->camera_id){
            RCLCPP_WARN(nh_->get_logger(), "No tags detected, restarting");
            camera_reset_publisher_->publish(std_msgs::msg::Bool());
        }
    }
}

void ContinuousDetector::ImageCallback (
    const nvidia::isaac_ros::nitros::NitrosImageView & view)
{
    // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
    // AprilTag 2 on the iamge
    if (!detection_enabled)
    {
        RCLCPP_DEBUG(nh_->get_logger(), "Detection is disabled, skipping image processing.");
        return;
    }
    if (!camera_info_)
    {
        RCLCPP_WARN(nh_->get_logger(), "No camera info received yet, skipping image processing.");
        return;
    }
    try
    {
        sensor_msgs::msg::Image image_rect;
        image_rect.header.frame_id = view.GetFrameId();
        image_rect.header.stamp.sec = view.GetTimestampSeconds();
        image_rect.header.stamp.nanosec = view.GetTimestampNanoseconds();
        image_rect.height = view.GetHeight();
        image_rect.width = view.GetWidth();
        image_rect.encoding = view.GetEncoding();
        image_rect.step = view.GetSizeInBytes() / view.GetHeight();

        image_rect.data.resize(view.GetSizeInBytes());
        cudaMemcpy(image_rect.data.data(), view.GetGpuData(), view.GetSizeInBytes(), cudaMemcpyDefault);
        cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect.encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(nh_->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Publish detected tags in the image by AprilTag 2
    AprilTagDetectionArray apriltag_msg = tag_detector_->detectTags(cv_image_, camera_info_);
    tag_detections_publisher_->publish(apriltag_msg);
    if (apriltag_msg.detections.size() > 0){
        tag_detected = true;
    }
    // Publish the camera image overlaid by outlines of the detected tags and
    // their payload values
    if (draw_tag_detections_image_)
    {
        tag_detector_->drawDetections(cv_image_);
        tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
    }
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
ContinuousDetector::get_node_base_interface() const
{
    return this->nh_->get_node_base_interface();
}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_ros::ContinuousDetector)
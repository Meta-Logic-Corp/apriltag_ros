// C++
#include <memory>
#include <mutex>

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

#include "lpv_interfaces/msg/apriltag_observer.hpp"

namespace apriltag_ros
{

enum class CameraPosition
{
    FRONT,
    BACK,
    LEFT,
    RIGHT
};

class CameraComponent
{
public:
    using AprilTagDetectionArray = apriltag_ros_interfaces::msg::AprilTagDetectionArray;
    CameraComponent(
        std::shared_ptr<rclcpp::Node>,
        CameraPosition,
        std::shared_ptr<TagDetector>
    );

    void ImageCallback(const nvidia::isaac_ros::nitros::NitrosImageView & view);

    void CameraInfoCallback(
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

    bool detection_enabled = false;
    bool tag_detected = false;

private:
    rclcpp::Node::SharedPtr nh_;

    std::mutex detection_mutex_;
    std::shared_ptr<TagDetector> tag_detector_;
    bool draw_tag_detections_image_;
    cv_bridge::CvImagePtr cv_image_;

    CameraPosition camera_position_;
    
    std::unordered_map<std::string, std::string> dir_map;
    std::unordered_map<std::string, sensor_msgs::msg::CameraInfo::ConstSharedPtr> camera_info_map;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
        nvidia::isaac_ros::nitros::NitrosImageView>> nitros_sub_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    rclcpp::Publisher<apriltag_ros_interfaces::msg::AprilTagDetectionArray>::SharedPtr tag_detections_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_reset_publisher_;
};

}
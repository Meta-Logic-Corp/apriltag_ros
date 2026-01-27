#include "apriltag_ros/camera_component.hpp"

using namespace apriltag_ros;

CameraComponent::CameraComponent(
    std::shared_ptr<rclcpp::Node> nh,
    CameraPosition camera_position,
    std::shared_ptr<TagDetector> tag_detector) :
    nh_(std::move(nh)),
     camera_position_(camera_position),
     tag_detector_(std::move(tag_detector))
    {
    std::string tag_detections_topic = nh_->get_parameter("tag_detections_topic").as_string();
    std::string image_topic = nh_->get_parameter("image_topic").as_string();

    std::unordered_map<CameraPosition, std::string> camera_position_map = {
        {CameraPosition::FRONT, "front"},
        {CameraPosition::BACK, "back"},
        {CameraPosition::LEFT, "left"},
        {CameraPosition::RIGHT, "right"}
    };

    nitros_sub_ = std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
        nvidia::isaac_ros::nitros::NitrosImageView>>(
        nh_.get(), std::string("/")  + camera_position_map.at(camera_position_).c_str() + image_topic,
        nvidia::isaac_ros::nitros::nitros_image_rgb8_t::supported_type_name,
        std::bind(&CameraComponent::ImageCallback, this,
        std::placeholders::_1));
    camera_info_sub_ = nh_->create_subscription<sensor_msgs::msg::CameraInfo>(
      std::string("/") + camera_position_map.at(camera_position_).c_str() + image_topic + std::string("/camera_info"), 10,
      std::bind(&CameraComponent::CameraInfoCallback, this, std::placeholders::_1));

    tag_detections_publisher_ = nh_->create_publisher<
        apriltag_ros_interfaces::msg::AprilTagDetectionArray>(
        std::string("/") + camera_position_map.at(camera_position_).c_str() + std::string("/") + tag_detections_topic, 10);
    }

void CameraComponent::CameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
    camera_info_ = msg;
}

void CameraComponent::ImageCallback (
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
}

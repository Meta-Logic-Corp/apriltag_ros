#include "apriltag_ros/camera_component.hpp"

using namespace apriltag_ros;

CameraComponent::CameraComponent(
    std::shared_ptr<rclcpp::Node> nh,
    CameraPosition camera_position,
    std::shared_ptr<TagDetector> tag_detector,
    std::shared_ptr<std::unordered_map<CameraPosition, bool>> target_tag_detected_map,
    std::shared_ptr<uint32_t> target_id) :
    nh_(std::move(nh)),
    camera_position_(camera_position),
    tag_detector_(std::move(tag_detector)),
    target_tag_detected_map_(std::move(target_tag_detected_map)),
    target_id_(std::move(target_id))
    {
    std::string tag_detections_topic = nh_->get_parameter("tag_detections_topic").as_string();
    std::string image_topic = nh_->get_parameter("image_topic").as_string();

    camera_position_map.insert({CameraPosition::FRONT, "front"});
    camera_position_map.insert({CameraPosition::BACK, "back"});
    camera_position_map.insert({CameraPosition::LEFT, "left"});
    camera_position_map.insert({CameraPosition::RIGHT, "right"});

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

float CameraComponent::getLuminance(
    const cv_bridge::CvImagePtr& image)
{
    RCLCPP_WARN(nh_->get_logger(), "IDENTIFIER: LUMINANCE CALCULATED");
    cv::Mat gray_image;
    if (image->encoding == "mono8") {
        gray_image = image->image;
    } else if (image->encoding == "bgr8") {
        cv::cvtColor(image->image, gray_image, cv::COLOR_BGR2GRAY);
    } else if (image->encoding == "rgb8") {
        cv::cvtColor(image->image, gray_image, cv::COLOR_RGB2GRAY);
    } else {
        RCLCPP_WARN(nh_->get_logger(), "Unsupported encoding: %s", image->encoding.c_str());
        return 0.0f;
    }

    image_u8_t apriltag_image = { .width = gray_image.cols,
                                    .height = gray_image.rows,
                                    .stride = gray_image.cols,
                                    .buf = gray_image.data
    };

    const size_t pixel_count = apriltag_image.height * apriltag_image.width;
    if (pixel_count == 0) {
        return 0.0f;
    }

    const uint8_t* data = apriltag_image.buf;

    uint64_t sum = 0;
    for (size_t i = 0; i < pixel_count; ++i) {
        sum += data[i];
    }

    float mean_brightness =
        static_cast<float>(sum) / static_cast<float>(pixel_count);

    return mean_brightness / 255.0f;
}

void CameraComponent::ImageCallback (
    const nvidia::isaac_ros::nitros::NitrosImageView & view)
{
    // RCLCPP_WARN(nh_->get_logger(), "IDENTIFIER: IMAGE CALLBACK HIT");
    // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
    // AprilTag 2 on the iamge
    if (!detection_enabled)
    {
        RCLCPP_DEBUG(nh_->get_logger(), "Detection is disabled, skipping image processing.");
        return;
    }
    for (const auto& [position, target_tag_detected] : *target_tag_detected_map_){
        if (target_tag_detected && position != camera_position_){
            RCLCPP_WARN(nh_->get_logger(), "Tag already detected by %s camera, skipping image processing.", camera_position_map.at(position).c_str());
            return;
        }
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
    if (!apriltag_msg.detections.empty()) {
            float exposure = getLuminance(cv_image_);
            for (auto & detection : apriltag_msg.detections) {
                detection.exposure = exposure;
            }
        }
    // RCLCPP_WARN(nh_->get_logger(), "IDENTIFIER: TAG DETECTION PUBLISHED");
    tag_detections_publisher_->publish(apriltag_msg);
    if (apriltag_msg.detections.size() > 0){
        tag_detected = true;
        for (const auto& detection : apriltag_msg.detections) {
            if (detection.id[0] == *target_id_) {
                RCLCPP_INFO(nh_->get_logger(), "Target tag ID %d detected by %s camera.", detection.id[0],camera_position_map.at(camera_position_).c_str());
                target_tag_detected_map_->at(camera_position_) = true;
            }
        }
    }
}

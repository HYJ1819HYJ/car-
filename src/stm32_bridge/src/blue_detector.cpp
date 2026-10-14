#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <stm32_bridge/BlackLineObservation.h>
#include <stm32_bridge/BlueTargetObservation.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

/**
 * 红色A4纸终点检测节点
 * 
 * 订阅: /usb_cam/image_raw
 * 发布: /goal_detected (bool标记 - 检测到终点)
 * 
 * 红色A4纸在HSV空间:
 *   H: 170~179 或 0~10 (跨越色相零点)
 *   当前目标：红色A4纸
 */

// 保留历史节点名、话题和消息类型，兼容导航与云台；默认检测红色。
class BlueDetector {
public:
  BlueDetector() : nh_("~") {
    nh_.param<std::string>("image_topic", image_topic_, "/usb_cam/image_raw");
    nh_.param<double>("ratio_threshold", ratio_threshold_, 0.0003);
    nh_.param<double>("min_contour_ratio", min_contour_ratio_, 0.00025);
    nh_.param<double>("min_fill_ratio", min_fill_ratio_, 0.40);
    nh_.param<int>("min_pixel_width", min_pixel_width_, 8);
    nh_.param<int>("min_pixel_height", min_pixel_height_, 10);
    nh_.param<double>("min_aspect", min_aspect_, 0.25);
    nh_.param<double>("max_aspect", max_aspect_, 1.70);
    nh_.param<int>("confirm_frames", confirm_frames_, 3);
    nh_.param<int>("clear_frames", clear_frames_, 6);
    nh_.param<int>("h_min", h_min_, 170);
    nh_.param<int>("h_max", h_max_, 10);
    nh_.param<int>("s_min", s_min_, 80);
    nh_.param<int>("v_min", v_min_, 80);
    nh_.param<bool>("show_debug", show_debug_, false);
    nh_.param<bool>("black_line_enabled", black_line_enabled_, true);
    nh_.param<int>("black_v_max", black_v_max_, 70);
    nh_.param<double>("black_roi_top", black_roi_top_, 0.45);
    nh_.param<double>("black_min_length_ratio", black_min_length_ratio_, 0.18);
    nh_.param<int>("black_hough_threshold", black_hough_threshold_, 40);
    nh_.param<int>("black_max_line_gap", black_max_line_gap_, 20);
    nh_.param<int>("black_confirm_frames", black_confirm_frames_, 4);
    nh_.param<int>("black_clear_frames", black_clear_frames_, 6);

    confirm_frames_ = std::max(1, confirm_frames_);
    clear_frames_ = std::max(1, clear_frames_);
    min_pixel_width_ = std::max(2, min_pixel_width_);
    min_pixel_height_ = std::max(2, min_pixel_height_);
    ratio_threshold_ = std::max(0.0, ratio_threshold_);
    min_contour_ratio_ = std::max(0.0, min_contour_ratio_);
    min_fill_ratio_ = std::max(0.0, std::min(1.0, min_fill_ratio_));
    min_aspect_ = std::max(0.05, min_aspect_);
    max_aspect_ = std::max(min_aspect_, max_aspect_);
    h_min_ = std::max(0, std::min(179, h_min_));
    h_max_ = std::max(0, std::min(179, h_max_));
    s_min_ = std::max(0, std::min(255, s_min_));
    v_min_ = std::max(0, std::min(255, v_min_));
    black_v_max_ = std::max(0, std::min(255, black_v_max_));
    black_roi_top_ = std::max(0.0, std::min(0.90, black_roi_top_));
    black_min_length_ratio_ = std::max(0.05, std::min(1.0, black_min_length_ratio_));
    black_hough_threshold_ = std::max(5, black_hough_threshold_);
    black_max_line_gap_ = std::max(0, black_max_line_gap_);
    black_confirm_frames_ = std::max(1, black_confirm_frames_);
    black_clear_frames_ = std::max(1, black_clear_frames_);

    img_sub_ = nh_.subscribe(image_topic_, 5, &BlueDetector::imgCb, this);
    goal_pub_ = nh_.advertise<std_msgs::Bool>("/goal_detected", 10);
    ratio_pub_ = nh_.advertise<std_msgs::Float32>("/blue_ratio", 10);
    observation_pub_ = nh_.advertise<stm32_bridge::BlueTargetObservation>(
        "/blue_target/observation", 10);
    black_line_pub_ = nh_.advertise<stm32_bridge::BlackLineObservation>(
        "/black_line/observation", 10);

    ROS_INFO("Red-goal detector: ratio>=%.3f contour>=%.3f confirm=%d frames",
             ratio_threshold_, min_contour_ratio_, confirm_frames_);
  }

private:
  void imgCb(const sensor_msgs::Image::ConstPtr& msg) {
    try {
      auto cvp = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat hsv;
      cv::cvtColor(cvp->image, hsv, cv::COLOR_BGR2HSV);

      // 红色 HSV 范围
      cv::Scalar lo(h_min_, s_min_, v_min_);
      cv::Scalar hi(h_max_, 255, 255);
      cv::Mat mask;
      if (h_min_ <= h_max_) {
        cv::inRange(hsv, lo, hi, mask);
      } else {
        // 红色跨越 HSV 色相零点：合并 [h_min,179] 与 [0,h_max]。
        cv::Mat low_hue_mask;
        cv::inRange(hsv, lo, cv::Scalar(179, 255, 255), mask);
        cv::inRange(hsv, cv::Scalar(0, s_min_, v_min_), hi, low_hue_mask);
        cv::bitwise_or(mask, low_hue_mask, mask);
      }

      // 形态学去噪
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
      cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

      const double image_area = static_cast<double>(mask.rows) * mask.cols;
      const double ratio = static_cast<double>(cv::countNonZero(mask)) / image_area;

      // 单看红色像素占比容易被红色杂物误触发。必须先在所有轮廓中
      // 筛出尺寸、宽高比和填充率像 A4 纸的候选，再选最大的合格候选。
      // 避免较大的背景色块遮蔽真正的纸张。
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      double largest_area = 0.0;
      cv::Rect largest_rect;
      double selected_fill_ratio = 0.0;
      double selected_aspect = 0.0;
      for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        const cv::Rect rect = cv::boundingRect(contour);
        const double rect_area = static_cast<double>(rect.area());
        const double fill = rect_area > 0.0 ? area / rect_area : 0.0;
        const double aspect = rect.height > 0
            ? static_cast<double>(rect.width) / rect.height : 0.0;
        const bool valid_shape = rect.width >= min_pixel_width_ &&
            rect.height >= min_pixel_height_ &&
            aspect >= min_aspect_ && aspect <= max_aspect_ &&
            fill >= min_fill_ratio_;
        if (valid_shape && area > largest_area) {
          largest_area = area;
          largest_rect = rect;
          selected_fill_ratio = fill;
          selected_aspect = aspect;
        }
      }

      const double contour_ratio = largest_area / image_area;
      const double fill_ratio = selected_fill_ratio;
      const double aspect = selected_aspect;
      const bool large_enough = largest_rect.width >= min_pixel_width_ &&
          largest_rect.height >= min_pixel_height_;
      const bool a4_like = aspect >= min_aspect_ && aspect <= max_aspect_;
      const bool candidate = ratio >= ratio_threshold_ &&
          contour_ratio >= min_contour_ratio_ &&
          fill_ratio >= min_fill_ratio_ && large_enough && a4_like;

      if (candidate) {
        candidate_frames_ = std::min(confirm_frames_, candidate_frames_ + 1);
        clear_count_ = 0;
        if (candidate_frames_ >= confirm_frames_) detected_ = true;
      } else {
        candidate_frames_ = 0;
        clear_count_ = std::min(clear_frames_, clear_count_ + 1);
        if (clear_count_ >= clear_frames_) detected_ = false;
      }

      std_msgs::Bool msg_out;
      msg_out.data = detected_;
      goal_pub_.publish(msg_out);

      std_msgs::Float32 ratio_msg;
      ratio_msg.data = static_cast<float>(ratio);
      ratio_pub_.publish(ratio_msg);

      stm32_bridge::BlueTargetObservation observation;
      observation.header = msg->header;
      // /goal_detected 保留迟滞，便于诊断；云台和定位必须只接收本帧仍然
      // 存在的有效矩形，否则短暂丢帧时 (0,0) 旧框会让云台猛转。
      observation.detected = detected_ && candidate;
      observation.image_width = static_cast<uint32_t>(mask.cols);
      observation.image_height = static_cast<uint32_t>(mask.rows);
      observation.center_x = static_cast<float>(largest_rect.x + 0.5 * largest_rect.width);
      observation.center_y = static_cast<float>(largest_rect.y + 0.5 * largest_rect.height);
      observation.pixel_width = static_cast<float>(largest_rect.width);
      observation.pixel_height = static_cast<float>(largest_rect.height);
      observation.blue_ratio = static_cast<float>(ratio);
      observation.contour_ratio = static_cast<float>(contour_ratio);
      observation.fill_ratio = static_cast<float>(fill_ratio);
      observation_pub_.publish(observation);

      // 黑色终点线只在图像下部地面 ROI 内检测。使用长直线而不是黑色
      // 像素面积，降低阴影、黑色障碍物和墙角造成的误触发。
      cv::Mat black_mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
      cv::Vec4i best_line(0, 0, 0, 0);
      double best_line_length = 0.0;
      bool raw_black_line = false;
      if (black_line_enabled_) {
        const int roi_y = static_cast<int>(std::round(black_roi_top_ * hsv.rows));
        const cv::Rect roi(0, roi_y, hsv.cols, hsv.rows - roi_y);
        cv::Mat roi_mask;
        cv::inRange(hsv(roi), cv::Scalar(0, 0, 0),
                    cv::Scalar(179, 255, black_v_max_), roi_mask);
        cv::Mat black_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(roi_mask, roi_mask, cv::MORPH_CLOSE, black_kernel);
        roi_mask.copyTo(black_mask(roi));

        std::vector<cv::Vec4i> lines;
        const double minimum_length = black_min_length_ratio_ * hsv.cols;
        cv::HoughLinesP(roi_mask, lines, 1.0, CV_PI / 180.0,
                        black_hough_threshold_, minimum_length, black_max_line_gap_);
        for (const cv::Vec4i& line : lines) {
          const double dx = static_cast<double>(line[2] - line[0]);
          const double dy = static_cast<double>(line[3] - line[1]);
          const double length = std::hypot(dx, dy);
          if (length > best_line_length) {
            best_line_length = length;
            best_line = cv::Vec4i(line[0], line[1] + roi_y,
                                  line[2], line[3] + roi_y);
          }
        }
        raw_black_line = best_line_length >= minimum_length;
      }

      if (raw_black_line) {
        black_candidate_frames_ = std::min(black_confirm_frames_,
                                           black_candidate_frames_ + 1);
        black_clear_count_ = 0;
        last_black_line_ = best_line;
        last_black_line_length_ = best_line_length;
        if (black_candidate_frames_ >= black_confirm_frames_) black_detected_ = true;
      } else {
        black_candidate_frames_ = 0;
        black_clear_count_ = std::min(black_clear_frames_, black_clear_count_ + 1);
        if (black_clear_count_ >= black_clear_frames_) black_detected_ = false;
      }

      stm32_bridge::BlackLineObservation line_observation;
      line_observation.header = msg->header;
      line_observation.detected = black_detected_;
      const double line_dx = static_cast<double>(last_black_line_[2] - last_black_line_[0]);
      const double line_dy = static_cast<double>(last_black_line_[3] - last_black_line_[1]);
      line_observation.confidence = static_cast<float>(std::min(
          1.0, last_black_line_length_ / std::max(1, hsv.cols)));
      line_observation.center_x = 0.5f * (last_black_line_[0] + last_black_line_[2]);
      line_observation.center_y = 0.5f * (last_black_line_[1] + last_black_line_[3]);
      line_observation.angle = static_cast<float>(std::atan2(line_dy, line_dx));
      line_observation.pixel_length = static_cast<float>(last_black_line_length_);
      line_observation.x1 = static_cast<float>(last_black_line_[0]);
      line_observation.y1 = static_cast<float>(last_black_line_[1]);
      line_observation.x2 = static_cast<float>(last_black_line_[2]);
      line_observation.y2 = static_cast<float>(last_black_line_[3]);
      black_line_pub_.publish(line_observation);

      if (show_debug_) {
        if (largest_rect.area() > 0) {
          cv::rectangle(cvp->image, largest_rect,
                        candidate ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
        }
        if (black_detected_) {
          cv::line(cvp->image,
                   cv::Point(last_black_line_[0], last_black_line_[1]),
                   cv::Point(last_black_line_[2], last_black_line_[3]),
                   cv::Scalar(0, 255, 255), 3);
        }
        cv::imshow("Red Mask", mask);
        cv::imshow("Black Line Mask", black_mask);
        cv::imshow("Red Detector", cvp->image);
        cv::waitKey(1);
      }

      if (observation.detected) {
        ROS_WARN_THROTTLE(1.0,
                          "Red goal confirmed: ratio=%.3f contour=%.3f fill=%.2f",
                          ratio, contour_ratio, fill_ratio);
      } else if (candidate) {
        ROS_INFO_THROTTLE(1.0, "Red candidate: %d/%d ratio=%.3f",
                          candidate_frames_, confirm_frames_, ratio);
      }
    } catch (const std::exception& e) {
      ROS_WARN_THROTTLE(5.0, "Image processing error: %s", e.what());
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber img_sub_;
  ros::Publisher goal_pub_;
  ros::Publisher ratio_pub_;
  ros::Publisher observation_pub_;
  ros::Publisher black_line_pub_;

  std::string image_topic_;
  double ratio_threshold_{0.0003};
  double min_contour_ratio_{0.00025};
  double min_fill_ratio_{0.40};
  int min_pixel_width_{8};
  int min_pixel_height_{10};
  double min_aspect_{0.25};
  double max_aspect_{1.70};
  int confirm_frames_{3};
  int clear_frames_{6};
  int h_min_{170};
  int h_max_{10};
  int s_min_{80};
  int v_min_{80};
  int candidate_frames_{0};
  int clear_count_{0};
  bool detected_{false};
  bool show_debug_{false};
  bool black_line_enabled_{true};
  int black_v_max_{70};
  double black_roi_top_{0.45};
  double black_min_length_ratio_{0.18};
  int black_hough_threshold_{40};
  int black_max_line_gap_{20};
  int black_confirm_frames_{4};
  int black_clear_frames_{6};
  int black_candidate_frames_{0};
  int black_clear_count_{0};
  bool black_detected_{false};
  cv::Vec4i last_black_line_{0, 0, 0, 0};
  double last_black_line_length_{0.0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "blue_detector");
  BlueDetector node;
  ros::spin();
  return 0;
}

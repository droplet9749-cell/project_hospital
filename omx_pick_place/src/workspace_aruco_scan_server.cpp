#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/videoio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "omx_pick_place/msg/box_detection.hpp"
#include "omx_pick_place/srv/scan_workspace.hpp"
#include "omx_pick_place/srv/pose_to_joint_execute.hpp"

#include "rclcpp/executors/multi_threaded_executor.hpp"
using namespace std::chrono_literals;

struct GraspRule
{
  double offset_x{0.0};
  double offset_y{0.0};
  double offset_z{0.0};
  double yaw_offset_deg{0.0};
};

struct ViewPlan
{
  geometry_msgs::msg::Pose desired_camera_pose_link0;
  geometry_msgs::msg::Point desired_ee_position_link0;
  double tool_pitch_sum{1.0};
  double wrist_angle{0.0};
};

class WorkspaceArucoScanServer : public rclcpp::Node
{
public:
  using ScanWorkspace = omx_pick_place::srv::ScanWorkspace;
  using BoxDetection = omx_pick_place::msg::BoxDetection;
  using PoseSrv = omx_pick_place::srv::PoseToJointExecute;

  WorkspaceArucoScanServer()
  : Node("workspace_aruco_scan_server"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameters();
    load_parameters();
    load_grasp_rules();

    client_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::QoS qos(10);
   

    pose_client_ = this->create_client<PoseSrv>(
      pose_service_name_,
      qos,
      client_cbg_);

    scan_srv_ = this->create_service<ScanWorkspace>(
      "/scan_workspace_aruco",
      std::bind(
        &WorkspaceArucoScanServer::handle_scan, this,
        std::placeholders::_1, std::placeholders::_2));

    init_camera();
    init_aruco();

    RCLCPP_INFO(get_logger(), "workspace_aruco_scan_server started.");
  }

  ~WorkspaceArucoScanServer() override
  {
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

private:
  rclcpp::Service<ScanWorkspace>::SharedPtr scan_srv_;
  rclcpp::Client<PoseSrv>::SharedPtr pose_client_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  cv::VideoCapture cap_;

  int camera_device_index_{0};
  int camera_width_{640};
  int camera_height_{480};

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  std::string arm_base_frame_;
  std::string ee_frame_;
  std::string camera_frame_;
  std::string pose_service_name_;
  std::string eval_link_name_;

  int pose_execute_mode_{5};


  double marker_length_m_{0.04};
  double ik_timeout_sec_{5.0};
  double move_duration_sec_{3.0};
  double tool_pitch_tolerance_{0.01};
  double wrist_tolerance_{0.0};
  double position_tolerance_{0.01};
  double default_view_pitch_rad_{1.05};
  double default_camera_standoff_m_{0.22};

  std::string aruco_dictionary_name_{"DICT_4X4_50"};
  std::vector<int64_t> box_ids_;
  std::map<int, GraspRule> grasp_rules_;

  rclcpp::CallbackGroup::SharedPtr client_cbg_;

private:
  void declare_parameters()
  {
    this->declare_parameter<std::string>("arm_base_frame", "link0");
    this->declare_parameter<std::string>("ee_frame", "end_effector_link");
    this->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");

    this->declare_parameter<std::string>("pose_service_name", "/pose_to_joint_execute");
    this->declare_parameter<int>("pose_execute_mode", 5);
    this->declare_parameter<double>("ik_timeout_sec", 5.0);
    this->declare_parameter<double>("move_duration_sec", 3.0);

    this->declare_parameter<std::string>("eval_link_name", "end_effector_link");
    this->declare_parameter<double>("tool_pitch_tolerance", 0.01);
    this->declare_parameter<double>("wrist_tolerance", 0.0);
    this->declare_parameter<double>("position_tolerance", 0.01);

    this->declare_parameter<double>("default_view_pitch_rad", 1.05);
    this->declare_parameter<double>("default_camera_standoff_m", 0.22);

    this->declare_parameter<int>("camera_device_index", 0);
    this->declare_parameter<int>("camera_width", 640);
    this->declare_parameter<int>("camera_height", 480);

    this->declare_parameter<double>("camera_fx", 615.0);
    this->declare_parameter<double>("camera_fy", 615.0);
    this->declare_parameter<double>("camera_cx", 320.0);
    this->declare_parameter<double>("camera_cy", 240.0);

    this->declare_parameter<double>("dist_k1", 0.0);
    this->declare_parameter<double>("dist_k2", 0.0);
    this->declare_parameter<double>("dist_p1", 0.0);
    this->declare_parameter<double>("dist_p2", 0.0);
    this->declare_parameter<double>("dist_k3", 0.0);

    this->declare_parameter<std::string>("aruco_dictionary", "DICT_4X4_50");
    this->declare_parameter<double>("marker_length_m", 0.04);

    this->declare_parameter<std::vector<int64_t>>("box_ids", std::vector<int64_t>{});
  }

  void load_parameters()
  {
    arm_base_frame_ = this->get_parameter("arm_base_frame").as_string();
    ee_frame_ = this->get_parameter("ee_frame").as_string();
    camera_frame_ = this->get_parameter("camera_frame").as_string();

    pose_service_name_ = this->get_parameter("pose_service_name").as_string();
    pose_execute_mode_ = this->get_parameter("pose_execute_mode").as_int();
    ik_timeout_sec_ = this->get_parameter("ik_timeout_sec").as_double();
    move_duration_sec_ = this->get_parameter("move_duration_sec").as_double();

    eval_link_name_ = this->get_parameter("eval_link_name").as_string();
    tool_pitch_tolerance_ = this->get_parameter("tool_pitch_tolerance").as_double();
    wrist_tolerance_ = this->get_parameter("wrist_tolerance").as_double();
    position_tolerance_ = this->get_parameter("position_tolerance").as_double();

    default_view_pitch_rad_ = this->get_parameter("default_view_pitch_rad").as_double();
    default_camera_standoff_m_ = this->get_parameter("default_camera_standoff_m").as_double();

        camera_device_index_ = this->get_parameter("camera_device_index").as_int();
    camera_width_ = this->get_parameter("camera_width").as_int();
    camera_height_ = this->get_parameter("camera_height").as_int();

    const double fx = this->get_parameter("camera_fx").as_double();
    const double fy = this->get_parameter("camera_fy").as_double();
    const double cx = this->get_parameter("camera_cx").as_double();
    const double cy = this->get_parameter("camera_cy").as_double();

    const double k1 = this->get_parameter("dist_k1").as_double();
    const double k2 = this->get_parameter("dist_k2").as_double();
    const double p1 = this->get_parameter("dist_p1").as_double();
    const double p2 = this->get_parameter("dist_p2").as_double();
    const double k3 = this->get_parameter("dist_k3").as_double();


    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0);

    dist_coeffs_ = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);

    aruco_dictionary_name_ = this->get_parameter("aruco_dictionary").as_string();
    marker_length_m_ = this->get_parameter("marker_length_m").as_double();

    box_ids_ = this->get_parameter("box_ids").as_integer_array();
  }

  void load_grasp_rules()
  {
    for (const auto & id64 : box_ids_) {
      int id = static_cast<int>(id64);
      GraspRule rule;
      rule.offset_x = declare_or_get_double("box_" + std::to_string(id) + ".grasp_offset_x", 0.0);
      rule.offset_y = declare_or_get_double("box_" + std::to_string(id) + ".grasp_offset_y", 0.0);
      rule.offset_z = declare_or_get_double("box_" + std::to_string(id) + ".grasp_offset_z", 0.0);
      rule.yaw_offset_deg = declare_or_get_double("box_" + std::to_string(id) + ".yaw_offset_deg", 0.0);
      grasp_rules_[id] = rule;
    }
  }

  double declare_or_get_double(const std::string & name, double default_value)
  {
    if (!this->has_parameter(name)) {
      this->declare_parameter<double>(name, default_value);
    }
    return this->get_parameter(name).as_double();
  }

  void init_camera()
  {
    cap_.open(camera_device_index_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      throw std::runtime_error("failed to open RGB camera");
    }
    bool ok_fourcc = cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    bool ok_w = cap_.set(cv::CAP_PROP_FRAME_WIDTH, camera_width_);
    bool ok_h = cap_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height_);
    bool ok_fps = cap_.set(cv::CAP_PROP_FPS, 30);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    RCLCPP_INFO(
      get_logger(),
      "camera set results: fourcc=%d width=%d height=%d fps=%d",
      ok_fourcc, ok_w, ok_h, ok_fps);
  }

  void init_aruco()
  {
	 dictionary_ = get_aruco_dictionary(aruco_dictionary_name_);
 	 detector_params_ = cv::aruco::DetectorParameters::create();
  }

  cv::Ptr<cv::aruco::Dictionary> get_aruco_dictionary(const std::string & name)
  {
    if (name == "DICT_4X4_50") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    if (name == "DICT_4X4_100") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    if (name == "DICT_5X5_50") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    if (name == "DICT_6X6_50") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  }

  void handle_scan(
    const std::shared_ptr<ScanWorkspace::Request> req,
    std::shared_ptr<ScanWorkspace::Response> res)
  {
    try {
      geometry_msgs::msg::Point workspace_center =
        compute_workspace_center(req->workspace_min, req->workspace_max);

      double standoff = req->camera_standoff_m > 0.0 ?
        req->camera_standoff_m : default_camera_standoff_m_;

      ViewPlan plan = make_workspace_view_plan(workspace_center, standoff);
      res->camera_target_pose_arm = plan.desired_camera_pose_link0;

      if (!move_camera_with_execute5(plan)) {
        res->success = false;
        res->message = "failed to move camera with execute_mode=5";
        return;
      }

      double settle = req->settle_time_sec > 0.0 ? req->settle_time_sec : 0.6;
      rclcpp::sleep_for(
  	std::chrono::duration_cast<std::chrono::nanoseconds>(
    	  std::chrono::duration<double>(settle)
        )
      );

      
      cv::Mat color_clone;
      if (!grab_frame(color_clone)) {
        res->success = false;
        res->message = "failed to capture RGB frame";
        return;
      }
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      std::vector<std::vector<cv::Point2f>> rejected;
      cv::aruco::detectMarkers(color_clone,dictionary_,corners,ids,detector_params_,rejected);

      if (ids.empty()) {
        res->success = true;
        res->message = "no aruco markers detected";
        return;
      }

      geometry_msgs::msg::TransformStamped tf_link0_from_cam =
        tf_buffer_.lookupTransform(arm_base_frame_, camera_frame_, tf2::TimePointZero);

      std::vector<cv::Vec3d> rvecs, tvecs;
      estimate_marker_poses(corners, rvecs, tvecs);

      for (size_t i = 0; i < ids.size(); ++i) {
        BoxDetection box;
        box.id = ids[i];
        box.valid = false;
        box.note = "";

        const cv::Vec3d & rvec = rvecs[i];
        const cv::Vec3d & tvec = tvecs[i];

        if (!std::isfinite(tvec[0]) || !std::isfinite(tvec[1]) || !std::isfinite(tvec[2]) || tvec[2] <= 0.0) {
          box.note = "invalid pose";
          res->boxes.push_back(box);
          continue;
        }

        box.depth_m = static_cast<float>(tvec[2]);

        geometry_msgs::msg::PointStamped p_cam, p_link0;
        p_cam.header.frame_id = camera_frame_;
        p_cam.point.x = tvec[0];
        p_cam.point.y = tvec[1];
        p_cam.point.z = tvec[2];

        tf2::doTransform(p_cam, p_link0, tf_link0_from_cam);

        box.box_center_link0 = p_link0.point;

        double yaw = estimate_marker_yaw_from_rvec(rvec);
        if (grasp_rules_.count(box.id)) {
          yaw += grasp_rules_[box.id].yaw_offset_deg * M_PI / 180.0;
        }
        box.yaw_rad = static_cast<float>(normalize_angle(yaw));

        geometry_msgs::msg::Point grasp = box.box_center_link0;
        if (grasp_rules_.count(box.id)) {
          const auto & rule = grasp_rules_[box.id];
          grasp.x += rule.offset_x;
          grasp.y += rule.offset_y;
          grasp.z += rule.offset_z;
        }
        box.grasp_point_link0 = grasp;

        if (!point_in_workspace(
              box.grasp_point_link0,
              req->workspace_min,
              req->workspace_max,
              0.0))
        {
          box.note = "detected but grasp point outside workspace";
          res->boxes.push_back(box);
          continue;
        }

        box.valid = true;
        box.note = "ok";
        res->boxes.push_back(box);
      }

      if (req->capture_debug_image) {
        draw_debug(color_clone, corners, ids, rvecs, tvecs);
        cv::imwrite("/tmp/scan_workspace_aruco_debug.png", color_clone);
      }

      res->success = true;
      res->message = "scan completed";
    } catch (const std::exception & e) {
      res->success = false;
      res->message = std::string("exception: ") + e.what();
    }
  }
  void estimate_marker_poses(
    const std::vector<std::vector<cv::Point2f>> & corners,
    std::vector<cv::Vec3d> & rvecs,
    std::vector<cv::Vec3d> & tvecs)
  {
    rvecs.clear();
    tvecs.clear();

    const double half = marker_length_m_ * 0.5;
    std::vector<cv::Point3f> obj_pts = {
      cv::Point3f(-half,  half, 0.0f),
      cv::Point3f( half,  half, 0.0f),
      cv::Point3f( half, -half, 0.0f),
      cv::Point3f(-half, -half, 0.0f)
    };

    cv::Mat zero_dist = cv::Mat::zeros(4, 1, CV_64F);
    
    for (const auto & c : corners) {

      cv::Vec3d rvec, tvec;

      bool ok = cv::solvePnP(
        obj_pts,
        c,
        camera_matrix_,
        dist_coeffs_,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE_SQUARE);

      if (!ok) {
        rvec = cv::Vec3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
        tvec = cv::Vec3d(std::numeric_limits<double>::quiet_NaN(), 0.0, -1.0);
      }

      rvecs.push_back(rvec);
      tvecs.push_back(tvec);
    }
  }
  double estimate_marker_yaw_from_rvec(const cv::Vec3d & rvec) const
  {
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);

    tf2::Matrix3x3 R(
      R_cv.at<double>(0, 0), R_cv.at<double>(0, 1), R_cv.at<double>(0, 2),
      R_cv.at<double>(1, 0), R_cv.at<double>(1, 1), R_cv.at<double>(1, 2),
      R_cv.at<double>(2, 0), R_cv.at<double>(2, 1), R_cv.at<double>(2, 2));

    double roll, pitch, yaw;
    R.getRPY(roll, pitch, yaw);
    return yaw;
  }
  bool grab_frame(cv::Mat & frame)
  {
    for (int i = 0; i < 5; ++i) {
      cap_ >> frame;
      if (!frame.empty()) {
        return true;
      }
      rclcpp::sleep_for(50ms);
    }
    return false;
  }

  geometry_msgs::msg::Point compute_workspace_center(
    const geometry_msgs::msg::Point & min_pt,
    const geometry_msgs::msg::Point & max_pt) const
  {
    geometry_msgs::msg::Point c;
    c.x = 0.5 * (min_pt.x + max_pt.x);
    c.y = 0.5 * (min_pt.y + max_pt.y);
    c.z = 0.5 * (min_pt.z + max_pt.z);
    return c;
  }

  ViewPlan make_workspace_view_plan(
    const geometry_msgs::msg::Point & target_link0,
    double standoff)
  {
    ViewPlan vp;

    vp.wrist_angle = 0.0;
    vp.tool_pitch_sum = default_view_pitch_rad_;
    // camera optical forward가 workspace 중심을 향한다고 가정한 시선벡터
    tf2::Vector3 forward(
      std::cos(vp.tool_pitch_sum) * std::cos(vp.wrist_angle),
      std::cos(vp.tool_pitch_sum) * std::sin(vp.wrist_angle),
      -std::sin(vp.tool_pitch_sum));
    forward.normalize();

    tf2::Vector3 target(target_link0.x, target_link0.y, target_link0.z);
    tf2::Vector3 cam_pos = target - standoff * forward;

    // 디버깅/응답용 camera 목표 pose
    vp.desired_camera_pose_link0.position.x = cam_pos.x();
    vp.desired_camera_pose_link0.position.y = cam_pos.y();
    vp.desired_camera_pose_link0.position.z = cam_pos.z();

    tf2::Quaternion q_cam;
    {
      // 보기용 pose이므로 대략적인 look-at quaternion만 구성
      tf2::Vector3 z_axis = forward;
      tf2::Vector3 world_up(0.0, 0.0, 1.0);
      if (std::fabs(z_axis.dot(world_up)) > 0.95) {
        world_up = tf2::Vector3(0.0, 1.0, 0.0);
      }
      tf2::Vector3 x_axis = world_up.cross(z_axis).normalized();
      tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();

      tf2::Matrix3x3 R(
        x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z());

      R.getRotation(q_cam);
      q_cam.normalize();
    }
    vp.desired_camera_pose_link0.orientation = tf2::toMsg(q_cam);

    // execute_mode=5는 orientation target 대신
    // tool_pitch_sum / wrist_angle 제약으로 eval_link 자세를 결정하므로
    // 그 자세에서 camera가 cam_pos에 오도록 EE 위치를 역산
    geometry_msgs::msg::TransformStamped tf_ee_from_cam =
      tf_buffer_.lookupTransform(ee_frame_, camera_frame_, tf2::TimePointZero);

    tf2::Transform T_ee_cam;
    tf2::fromMsg(tf_ee_from_cam.transform, T_ee_cam);

    // wrist_angle: base z축 yaw
    // tool_pitch_sum: 아래쪽 + 이므로 conventional Ry에서는 음수 회전으로 모델링
    tf2::Matrix3x3 R_ee =
      tf2::Matrix3x3(tf2::Quaternion(tf2::Vector3(0, 0, 1), vp.wrist_angle)) *
      tf2::Matrix3x3(tf2::Quaternion(tf2::Vector3(0, 1, 0), -vp.tool_pitch_sum));

    tf2::Vector3 p_ee_cam = T_ee_cam.getOrigin();  // ee frame에서 본 camera 위치
    tf2::Vector3 p_ee_cam_in_link0 = R_ee * p_ee_cam;
    tf2::Vector3 ee_pos = cam_pos - p_ee_cam_in_link0;

    vp.desired_ee_position_link0.x = ee_pos.x();
    vp.desired_ee_position_link0.y = ee_pos.y();
    vp.desired_ee_position_link0.z = ee_pos.z();

    return vp;
  }

  bool move_camera_with_execute5(const ViewPlan & vp)
  {
    if (!pose_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(get_logger(), "pose service not available: %s", pose_service_name_.c_str());
      return false;
    }

    auto req = std::make_shared<PoseSrv::Request>();
    req->target_pose.header.frame_id = arm_base_frame_;
    req->target_pose.pose.position = vp.desired_ee_position_link0;

    // execute_mode=5에서는 orientation을 직접 쓰지 않더라도 형식상 채움
    req->target_pose.pose.orientation.x = 0.0;
    req->target_pose.pose.orientation.y = 0.0;
    req->target_pose.pose.orientation.z = 0.0;
    req->target_pose.pose.orientation.w = 1.0;

    req->execute_mode = pose_execute_mode_;
    req->ik_timeout_sec = ik_timeout_sec_;
    req->move_duration_sec = move_duration_sec_;

    req->tool_pitch_sum = vp.tool_pitch_sum;
    req->wrist_angle = vp.wrist_angle;
    req->position_tolerance = position_tolerance_;
    req->eval_link_name = eval_link_name_;

    req->tool_pitch_tolerance = tool_pitch_tolerance_;
    req->wrist_tolerance = wrist_tolerance_;
    RCLCPP_INFO(
      get_logger(),
      "execute5 request: frame=%s pos=(%.6f, %.6f, %.6f) ori=(%.3f, %.3f, %.3f, %.3f) "
      "mode=%d ik=%.3f move=%.3f pitch=%.6f pitch_tol=%.6f wrist=%.6f wrist_tol=%.6f "
      "pos_tol=%.6f eval=%s",
      req->target_pose.header.frame_id.c_str(),
      req->target_pose.pose.position.x,
      req->target_pose.pose.position.y,
      req->target_pose.pose.position.z,
      req->target_pose.pose.orientation.x,
      req->target_pose.pose.orientation.y,
      req->target_pose.pose.orientation.z,
      req->target_pose.pose.orientation.w,
      req->execute_mode,
      req->ik_timeout_sec,
      req->move_duration_sec,
      req->tool_pitch_sum,
      req->tool_pitch_tolerance,
      req->wrist_angle,
      req->wrist_tolerance,
      req->position_tolerance,
      req->eval_link_name.c_str());
    auto future = pose_client_->async_send_request(req);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (rclcpp::ok()) {
      auto status = future.wait_for(std::chrono::milliseconds(50));
      if (status == std::future_status::ready) {
        break;
      }
      if (std::chrono::steady_clock::now() > deadline) {
        throw std::runtime_error("pose_to_joint_execute call timeout");
      }
    }

    if (!rclcpp::ok()) {
      throw std::runtime_error("pose_to_joint_execute interrupted (rclcpp not ok)");
    }

    const auto resp = future.get();
    if (!resp->success) {
      throw std::runtime_error(std::string("pose_to_joint_execute failed: ") + resp->message);
    }

    return resp->success;
  }

  double normalize_angle(double a) const
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  bool point_in_workspace(
    const geometry_msgs::msg::Point & p,
    const geometry_msgs::msg::Point & min_pt,
    const geometry_msgs::msg::Point & max_pt,
    double margin) const
  {
    return (p.x >= min_pt.x - margin && p.x <= max_pt.x + margin &&
            p.y >= min_pt.y - margin && p.y <= max_pt.y + margin &&
            p.z >= min_pt.z - margin && p.z <= max_pt.z + margin);
  }

  void draw_debug(
    cv::Mat & image,
    const std::vector<std::vector<cv::Point2f>> & corners,
    const std::vector<int> & ids,
    const std::vector<cv::Vec3d> & rvecs,
    const std::vector<cv::Vec3d> & tvecs)
  {
    cv::aruco::drawDetectedMarkers(image, corners, ids);

    for (size_t i = 0; i < ids.size(); ++i) {
      cv::drawFrameAxes(image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], marker_length_m_ * 0.5);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<WorkspaceArucoScanServer>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  
  
  rclcpp::shutdown();
  return 0;
}


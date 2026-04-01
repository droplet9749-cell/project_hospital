#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <stdexcept>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// arm service
#include "omx_pick_place/srv/pose_to_joint_execute.hpp"

// scan service + msg
#include "omx_pick_place/srv/scan_workspace.hpp"
#include "omx_pick_place/msg/box_detection.hpp"

// orchestration srv
#include "omx_pick_place/srv/execute_seq.hpp"

using namespace std::chrono_literals;

class ExecuteSeqNode : public rclcpp::Node
{
public:
  using ExecuteSeq = omx_pick_place::srv::ExecuteSeq;
  using PoseSrv = omx_pick_place::srv::PoseToJointExecute;
  using ScanSrv = omx_pick_place::srv::ScanWorkspace;
  using BoxDetection = omx_pick_place::msg::BoxDetection;

  ExecuteSeqNode()
  : Node("execute_seq_node")
  {
    arm_base_frame_ = this->declare_parameter<std::string>("arm_base_frame", "link0");
    pose_service_name_ = this->declare_parameter<std::string>("pose_service_name", "/pose_to_joint_execute");
    scan_service_name_ = this->declare_parameter<std::string>("scan_service_name", "/scan_workspace_aruco");

    pick_execute_mode_ = this->declare_parameter<int>("pick_execute_mode", 5);
    ik_timeout_sec_ = this->declare_parameter<double>("ik_timeout_sec", 5.0);
    move_duration_sec_ = this->declare_parameter<double>("move_duration_sec", 3.0);

    eval_link_name_ = this->declare_parameter<std::string>("eval_link_name", "end_effector_link");
    tool_pitch_sum_ = this->declare_parameter<double>("tool_pitch_sum", 1.20);
    tool_pitch_tolerance_ = this->declare_parameter<double>("tool_pitch_tolerance", 0.05);
    wrist_tolerance_ = this->declare_parameter<double>("wrist_tolerance", 0.10);
    position_tolerance_ = this->declare_parameter<double>("position_tolerance", 0.01);

    // pick stages
    pregrasp_offset_z_ = this->declare_parameter<double>("pregrasp_offset_z", 0.05);
    lift_offset_z_ = this->declare_parameter<double>("lift_offset_z", 0.10);

    settle_time_sec_ = this->declare_parameter<double>("settle_time_sec", 0.6);
    capture_debug_image_ = this->declare_parameter<bool>("capture_debug_image", false);

    // wrist 계산 관련
    use_radial_compensation_ = this->declare_parameter<bool>("use_radial_compensation", true);
    wrist_yaw_offset_rad_ = this->declare_parameter<double>("wrist_yaw_offset_rad", 0.0);
    radial_alignment_gain_ = this->declare_parameter<double>("radial_alignment_gain", 1.0);

    simulate_gripper_only_ = this->declare_parameter<bool>("simulate_gripper_only", true);

    client_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    pose_client_ = this->create_client<PoseSrv>(
      pose_service_name_,
      rmw_qos_profile_services_default,
      client_cbg_);

    scan_client_ = this->create_client<ScanSrv>(
      scan_service_name_,
      rmw_qos_profile_services_default,
      client_cbg_);

    exec_srv_ = this->create_service<ExecuteSeq>(
      "/execute_seq",
      std::bind(&ExecuteSeqNode::handle_execute, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "execute_seq_node started");
  }

private:
  rclcpp::Service<ExecuteSeq>::SharedPtr exec_srv_;
  rclcpp::Client<PoseSrv>::SharedPtr pose_client_;
  rclcpp::Client<ScanSrv>::SharedPtr scan_client_;
  rclcpp::CallbackGroup::SharedPtr client_cbg_;

  std::string arm_base_frame_;
  std::string pose_service_name_;
  std::string scan_service_name_;
  std::string eval_link_name_;

  int pick_execute_mode_{5};
  double ik_timeout_sec_{5.0};
  double move_duration_sec_{3.0};

  double tool_pitch_sum_{1.20};
  double tool_pitch_tolerance_{0.05};
  double wrist_tolerance_{0.10};
  double position_tolerance_{0.01};

  double pregrasp_offset_z_{0.05};
  double lift_offset_z_{0.10};

  double settle_time_sec_{0.6};
  bool capture_debug_image_{false};

  bool use_radial_compensation_{true};
  double wrist_yaw_offset_rad_{0.0};
  double radial_alignment_gain_{1.0};

  bool simulate_gripper_only_{true};

private:
  void handle_execute(
    const std::shared_ptr<ExecuteSeq::Request> req,
    std::shared_ptr<ExecuteSeq::Response> res)
  {
    initialize_response(res);

    try {
      if (req->execute_seq == "A") {
        run_sequence_A(req, res);
        return;
      }

      res->success = false;
      res->message = "Unsupported execute_seq. Add new run_sequence_X().";
    } catch (const std::exception & e) {
      res->success = false;
      res->message = std::string("exception: ") + e.what();
    }
  }

  void initialize_response(std::shared_ptr<ExecuteSeq::Response> res)
  {
    res->success = false;
    res->message.clear();
    res->picked_box_id = -1;
    res->picked_point_link0.x = 0.0;
    res->picked_point_link0.y = 0.0;
    res->picked_point_link0.z = 0.0;
  }

  void run_sequence_A(
    const std::shared_ptr<ExecuteSeq::Request> req,
    std::shared_ptr<ExecuteSeq::Response> res)
  {
    auto scan_resp = call_scan_workspace(
      req->workspace_min,
      req->workspace_max,
      req->camera_standoff_m > 0.0 ? req->camera_standoff_m : 0.0,
      settle_time_sec_,
      capture_debug_image_);

    if (!scan_resp->success) {
      res->message = std::string("scan failed: ") + scan_resp->message;
      return;
    }

    auto target_box_opt = select_target_box(scan_resp->boxes, req->target_box_id);
    if (!target_box_opt.has_value()) {
      res->message = "No valid target box found from scan response";
      return;
    }

    const auto & box = target_box_opt.value();
    res->picked_box_id = box.id;
    res->picked_point_link0 = box.grasp_point_link0;

    if (!execute_pick_from_box(box)) {
      res->message = "Sequence A failed while executing pick";
      return;
    }

    res->success = true;
    res->message = "Sequence A completed";
  }

  std::shared_ptr<ScanSrv::Response> call_scan_workspace(
    const geometry_msgs::msg::Point & workspace_min,
    const geometry_msgs::msg::Point & workspace_max,
    double camera_standoff_m,
    double settle_time_sec,
    bool capture_debug_image)
  {
    if (!scan_client_->wait_for_service(2s)) {
      throw std::runtime_error("scan service not available: " + scan_service_name_);
    }

    auto req = std::make_shared<ScanSrv::Request>();
    req->workspace_min = workspace_min;
    req->workspace_max = workspace_max;
    req->camera_standoff_m = static_cast<float>(camera_standoff_m);
    req->settle_time_sec = static_cast<float>(settle_time_sec);
    req->capture_debug_image = capture_debug_image;

    auto future = scan_client_->async_send_request(req);

    if (future.wait_for(40s) != std::future_status::ready) {
      throw std::runtime_error("scan service timeout");
    }

    return future.get();
  }

  std::optional<BoxDetection> select_target_box(
    const std::vector<BoxDetection> & boxes,
    int target_box_id)
  {
    // ScanWorkspace 응답의 boxes[]를 그대로 파싱
    // 각 원소는 workspace_aruco_scan_server.cpp에서
    // id / valid / yaw_rad / grasp_point_link0 / box_center_link0 / note 등을 채워 반환됨
    for (const auto & box : boxes) {
      RCLCPP_INFO(
        get_logger(),
        "scan box: id=%d valid=%d grasp=(%.4f, %.4f, %.4f) center=(%.4f, %.4f, %.4f) yaw=%.4f note=%s",
        box.id,
        box.valid ? 1 : 0,
        box.grasp_point_link0.x, box.grasp_point_link0.y, box.grasp_point_link0.z,
        box.box_center_link0.x, box.box_center_link0.y, box.box_center_link0.z,
        static_cast<double>(box.yaw_rad),
        box.note.c_str());
    }

    if (target_box_id >= 0) {
      for (const auto & box : boxes) {
        if (box.id == target_box_id && box.valid) {
          return box;
        }
      }
      return std::nullopt;
    }

    for (const auto & box : boxes) {
      if (box.valid) {
        return box;
      }
    }

    return std::nullopt;
  }

  bool execute_pick_from_box(const BoxDetection & box)
  {
    // grasp point
    geometry_msgs::msg::Point grasp_pt = box.grasp_point_link0;

    // pregrasp: z + 0.05
    geometry_msgs::msg::Point pregrasp_pt = grasp_pt;
    pregrasp_pt.z += pregrasp_offset_z_;

    // lift
    geometry_msgs::msg::Point lift_pt = grasp_pt;
    lift_pt.z += lift_offset_z_;

    const double wrist_angle = compute_pick_wrist_angle(box);

    RCLCPP_INFO(
      get_logger(),
      "pick target box_id=%d grasp=(%.4f, %.4f, %.4f) yaw_rad=%.4f wrist_angle=%.4f",
      box.id,
      grasp_pt.x, grasp_pt.y, grasp_pt.z,
      static_cast<double>(box.yaw_rad),
      wrist_angle);

    if (!open_gripper()) {
      RCLCPP_ERROR(get_logger(), "Failed to open gripper");
      return false;
    }

    // 1) pregrasp: grasp점 위 z+0.05
    if (!move_arm_to_point_constrained(pregrasp_pt, wrist_angle, "pregrasp")) {
      return false;
    }

    // 2) 수직 하강: x,y 유지 / z만 grasp로 내려감
    if (!move_arm_to_point_constrained(grasp_pt, wrist_angle, "vertical_descend")) {
      return false;
    }

    // 3) close
    if (!close_gripper()) {
      RCLCPP_ERROR(get_logger(), "Failed to close gripper");
      return false;
    }

    // 4) lift
    if (!move_arm_to_point_constrained(lift_pt, wrist_angle, "lift")) {
      return false;
    }

    return true;
  }

  double compute_pick_wrist_angle(const BoxDetection & box) const
  {
    // 1) 박스 자체 회전 정보
    const double box_yaw = normalize_angle(static_cast<double>(box.yaw_rad));

    // 2) grasp point가 base(link0) 기준 어느 방향에 있는지
    //    q1 seed가 atan2(y,x) 근처에서 잡히기 때문에,
    //    물체의 방사 방향(radial direction)을 같이 고려해
    //    wrist가 현재 물체 위치에 대해 자연스럽게 정렬되도록 함
    const double radial_dir = std::atan2(
      static_cast<double>(box.grasp_point_link0.y),
      static_cast<double>(box.grasp_point_link0.x));

    // 3) box yaw + radial compensation
    //    - yaw_rad만 쓰면 물체 자체 회전만 반영됨
    //    - grasp 위치(x,y)에 따른 접근 방향까지 반영하려면 radial_dir를 일부 섞어야 함
    double wrist = box_yaw + wrist_yaw_offset_rad_;

    if (use_radial_compensation_) {
      wrist += radial_alignment_gain_ * radial_dir;
    }

    return normalize_angle(wrist);
  }

  bool move_arm_to_point_constrained(
    const geometry_msgs::msg::Point & pt,
    double wrist_angle,
    const std::string & stage_name)
  {
    if (!pose_client_) {
      throw std::runtime_error("pose client not created");
    }

    if (!pose_client_->wait_for_service(2s)) {
      throw std::runtime_error("pose service not available: " + pose_service_name_);
    }

    auto req = std::make_shared<PoseSrv::Request>();

    req->target_pose.header.frame_id = arm_base_frame_;
    req->target_pose.pose.position = pt;

    // execute_mode=5에서는 orientation보다
    // tool_pitch_sum + wrist_angle + tolerances가 핵심
    req->target_pose.pose.orientation.x = 0.0;
    req->target_pose.pose.orientation.y = 0.0;
    req->target_pose.pose.orientation.z = 0.0;
    req->target_pose.pose.orientation.w = 1.0;

    // constrained direct controller
    req->execute_mode = 5;
    req->ik_timeout_sec = ik_timeout_sec_;
    req->move_duration_sec = move_duration_sec_;

    req->tool_pitch_sum = tool_pitch_sum_;
    req->wrist_angle = wrist_angle;
    req->position_tolerance = position_tolerance_;
    req->eval_link_name = eval_link_name_;
    req->tool_pitch_tolerance = tool_pitch_tolerance_;
    req->wrist_tolerance = wrist_tolerance_;

    RCLCPP_INFO(
      get_logger(),
      "[%s] execute_mode=5 pos=(%.4f, %.4f, %.4f), tool_pitch_sum=%.4f wrist_angle=%.4f",
      stage_name.c_str(),
      pt.x, pt.y, pt.z,
      req->tool_pitch_sum,
      req->wrist_angle);

    auto future = pose_client_->async_send_request(req);

    const auto deadline = std::chrono::steady_clock::now() + 30s;
    while (rclcpp::ok()) {
      auto status = future.wait_for(50ms);
      if (status == std::future_status::ready) {
        break;
      }
      if (std::chrono::steady_clock::now() > deadline) {
        throw std::runtime_error("pose_to_joint_execute timeout at stage: " + stage_name);
      }
    }

    if (!rclcpp::ok()) {
      throw std::runtime_error("pose_to_joint_execute interrupted");
    }

    auto resp = future.get();
    if (!resp->success) {
      RCLCPP_ERROR(
        get_logger(),
        "[%s] pose_to_joint_execute failed: %s",
        stage_name.c_str(),
        resp->message.c_str());
      return false;
    }

    return true;
  }

  bool open_gripper()
  {
    if (simulate_gripper_only_) {
      RCLCPP_INFO(get_logger(), "[gripper] simulated open");
      return true;
    }

    // TODO: 실제 gripper 제어로 교체
    return false;
  }

  bool close_gripper()
  {
    if (simulate_gripper_only_) {
      RCLCPP_INFO(get_logger(), "[gripper] simulated close");
      return true;
    }

    // TODO: 실제 gripper 제어로 교체
    return false;
  }

  double normalize_angle(double a) const
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ExecuteSeqNode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

#include <chrono>
#include <memory>
#include <string>
#include <stdexcept>
#include <atomic>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

// Gripper는 MoveIt으로 제어 (arm은 서비스로 위임)
#include <moveit/move_group_interface/move_group_interface.h>

// pose_to_joint_execute_server의 서비스 타입
#include "omx_pick_place/srv/pose_to_joint_execute.hpp"

using PoseToJointExecute = omx_pick_place::srv::PoseToJointExecute;

class PickPlaceNode : public rclcpp::Node
{
public:
  PickPlaceNode()
  : Node("omx_pick_place_node")
  {
    // 필요 시 시뮬 시간
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // arm 이동은 서비스로 요청
    pose_exec_client_ = this->create_client<PoseToJointExecute>("pose_to_joint_execute");

    // /tomato_target 메시지(=YOLO bbox)가 오면 실행 (데이터는 무시)
    tomato_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/tomato_target",
      rclcpp::QoS(10),
      std::bind(&PickPlaceNode::onTomatoTarget, this, std::placeholders::_1));

    // 완료 신호 publish
    mani_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/mani_command", 10);

    // ---- Pick/Place 목표를 파라미터로 받기 (기존 하드코드 기본값 유지) ----
    // 프레임
    this->declare_parameter<std::string>("target_frame", "link0");

    // 물체(픽) pose
    this->declare_parameter<std::vector<double>>(
      "pick.position", {0.09, -0.295, 0.1});
    this->declare_parameter<std::vector<double>>(
      "pick.orientation", {0.0, 0.0, 0.0, 1.0});

    // 바구니(플레이스) pose
    this->declare_parameter<std::vector<double>>(
      "place.position", {-0.25, -0.01, 0.05});
    this->declare_parameter<std::vector<double>>(
      "place.orientation", {0.0, 0.0, 0.0, 1.0});
  }

  void init()
  {
    // Gripper MoveIt 인터페이스만 사용
    gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "gripper");

    gripper_->setPlanningTime(5.0);
    gripper_->setMaxVelocityScalingFactor(0.3);

    RCLCPP_INFO(get_logger(), "PickPlaceNode initialized (gripper MoveIt + arm service client)");
  }

private:
  void onTomatoTarget(const std_msgs::msg::Float32MultiArray::SharedPtr /*msg*/)
  {
    // NOTE: MultiThreadedExecutor에서 콜백은 병렬 실행될 수 있으므로
    //       재진입 방지를 위해 atomic flag 사용.
    if (busy_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "Pick&Place is already running. Ignoring /tomato_target.");
      return;
    }

    // IMPORTANT:
    // /tomato_target 구독 콜백에서 MoveIt execute(), 서비스 호출 wait 등
    // 시간이 오래 걸리는 작업을 바로 수행하면, 같은 executor가 처리해야 하는
    // MoveIt action 응답(GoalResponse/Result) 콜백이 지연/꼬이면서
    // "unknown goal response" -> abort/timeout로 이어질 수 있습니다.
    // 그래서 실제 Pick&Place는 별도 스레드에서 실행하고, 콜백은 즉시 반환합니다.
    auto self = std::static_pointer_cast<PickPlaceNode>(shared_from_this());
    std::thread([self]() {
      bool ok = true;
      try {
        self->run_demo();
      } catch (const std::exception &e) {
        ok = false;
        RCLCPP_ERROR(self->get_logger(), "Pick&Place failed: %s", e.what());
      }

      std_msgs::msg::String out;
      out.data = ok ? "grap_done" : "failed";
      self->mani_cmd_pub_->publish(out);
      RCLCPP_INFO(self->get_logger(), "Published /mani_command: %s", out.data.c_str());

      self->busy_.store(false);
    }).detach();
  }

  // ---- Pick & Place 실행 ----
  void run_demo()
  {
    // 파라미터에서 pick/place pose 읽기
    const auto frame = this->get_parameter("target_frame").as_string();

    geometry_msgs::msg::PoseStamped pick = load_pose_from_params_("pick", frame);
    geometry_msgs::msg::PoseStamped place = load_pose_from_params_("place", frame);

    execute(pick, place);
  }

  geometry_msgs::msg::PoseStamped load_pose_from_params_(
    const std::string & prefix,
    const std::string & frame_id)
  {
    // prefix.position: [x,y,z]
    // prefix.orientation: [x,y,z,w]
    const std::string p_key = prefix + ".position";
    const std::string q_key = prefix + ".orientation";

    if (!this->has_parameter(p_key) || !this->has_parameter(q_key)) {
      throw std::runtime_error(
        "Missing parameters: '" + p_key + "' and/or '" + q_key + "'");
    }

    const auto p = this->get_parameter(p_key).as_double_array();
    const auto q = this->get_parameter(q_key).as_double_array();
    if (p.size() != 3) {
      throw std::runtime_error(
        "Parameter '" + p_key + "' must be [x,y,z] (size=3)");
    }
    if (q.size() != 4) {
      throw std::runtime_error(
        "Parameter '" + q_key + "' must be [x,y,z,w] (size=4)");
    }

    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = frame_id;
    out.pose.position.x = p[0];
    out.pose.position.y = p[1];
    out.pose.position.z = p[2];
    out.pose.orientation.x = q[0];
    out.pose.orientation.y = q[1];
    out.pose.orientation.z = q[2];
    out.pose.orientation.w = q[3];
    return out;
  }

  geometry_msgs::msg::PoseStamped pregrasp(geometry_msgs::msg::PoseStamped p)
  {
    p.pose.position.y += 0.08;
    return p;
  }

  geometry_msgs::msg::PoseStamped lift(geometry_msgs::msg::PoseStamped p)
  {
    p.pose.position.z += 0.15;
    return p;
  }
  geometry_msgs::msg::PoseStamped lower(geometry_msgs::msg::PoseStamped p)
  {
    p.pose.position.z -= 0.08;
    return p;
  }


  // ---- arm 이동: pose_to_joint_execute_server로 요청 ----
  void move_arm_via_service(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!pose_exec_client_) {
      throw std::runtime_error("pose_to_joint_execute client not created");
    }

    // 서비스 대기
    if (!pose_exec_client_->wait_for_service(std::chrono::seconds(2))) {
      throw std::runtime_error("pose_to_joint_execute service not available");
    }

    auto req = std::make_shared<PoseToJointExecute::Request>();
    req->target_pose = pose;

    // 서버 구현 기준:
    // 0: IK only, 1: MoveIt plan+execute, 2: Direct controller(FJT)
    req->execute_mode = 2;

    req->ik_timeout_sec = 5.0;
    req->move_duration_sec = 5.0;

    auto future = pose_exec_client_->async_send_request(req);

    // IMPORTANT:
    // rclcpp::spin_until_future_complete()는 임시 Executor를 만들고 add_node()를 시도해서
    // "Node has already been added to an executor" 예외가 날 수 있음.
    // main에서 MultiThreadedExecutor가 spin 중이므로 여기서는 wait_for만 한다.
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
  }

  // ---- gripper: 기존 MoveIt named target 사용 ----
  void gripper(const std::string & state)
  {
    if (!gripper_) return;

    gripper_->setNamedTarget(state);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = static_cast<bool>(gripper_->plan(plan));
    RCLCPP_INFO(get_logger(), "GRIPPER PLAN: %d", ok);

    if (!ok) {
      throw std::runtime_error("gripper plan failed");
    }

    auto ret = gripper_->execute(plan);
    RCLCPP_INFO(get_logger(), "GRIPPER EXEC RESULT: %d", ret.val);

    /*if (ret.val != moveit::core::MoveItErrorCode::SUCCESS) {
      throw std::runtime_error("gripper execute failed or timeout");
    }*/
  }

  void execute(geometry_msgs::msg::PoseStamped pick,
               geometry_msgs::msg::PoseStamped place)
  {
    RCLCPP_INFO(get_logger(), "START PICK & PLACE");

    gripper("open");

    move_arm_via_service(pregrasp(pick));
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    move_arm_via_service(pick);
    gripper("close");
    rclcpp::sleep_for(std::chrono::milliseconds(2000));
    move_arm_via_service(lower(pick));

    move_arm_via_service(place);
    gripper("open");
    move_arm_via_service(lift(place));

    RCLCPP_INFO(get_logger(), "DONE");
  }

private:
  rclcpp::Client<PoseToJointExecute>::SharedPtr pose_exec_client_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tomato_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mani_cmd_pub_;

  std::atomic<bool> busy_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PickPlaceNode>();
  node->init();

  // MoveIt/TF 초기화 대기 (필요 시)
  rclcpp::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(node->get_logger(), "Waiting for /tomato_target ...");

  // 콜백(구독/서비스 응답/액션 피드백 등)을 병렬 처리
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}


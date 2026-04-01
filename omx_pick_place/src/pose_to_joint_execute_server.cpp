#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

// 네 srv 패키지/이름에 맞게 include 경로가 달라짐
#include "omx_pick_place/srv/pose_to_joint_execute.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
//constrain ik
#include <moveit/robot_model/robot_model.h>
#include <limits>
#include <cmath>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

using PoseToJointExecute = omx_pick_place::srv::PoseToJointExecute;
using FJT = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FJT>;
class PoseToJointExecuteServer : public rclcpp::Node
{
public:
  PoseToJointExecuteServer()
  : Node("pose_to_joint_execute_server"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // MoveGroupInterface는 shared_from_this() 때문에 타이머 init 권장
    init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&PoseToJointExecuteServer::init_once, this));
    // FollowJointTrajectory action 이름 (moveit_controllers.yaml 기준 arm_controller/follow_joint_trajectory)
    fjt_action_name_ = this->declare_parameter<std::string>(
      "fjt_action_name", "/arm_controller/follow_joint_trajectory");

    fjt_client_ = rclcpp_action::create_client<FJT>(this, fjt_action_name_);

    RCLCPP_INFO(get_logger(), "Direct FJT action: %s", fjt_action_name_.c_str());
  }
  ~PoseToJointExecuteServer() override
  {
    if (mg_exec_ && mg_node_) {
      mg_exec_->cancel();
      mg_exec_->remove_node(mg_node_);
    }
    if (mg_spin_thread_.joinable()) {
      mg_spin_thread_.join();
    }
  }
private:
  void init_once()
  {
    init_timer_->cancel();

    //constrain ik
    constrained_eval_link_ =
      this->declare_parameter<std::string>("constrained_eval_link", "end_effector_link");

    constrained_default_pos_tol_ =
      this->declare_parameter<double>("constrained_default_pos_tol", 0.01);

    constrained_search_step_rad_ =
      this->declare_parameter<double>("constrained_search_step_rad", 0.05);


    group_name_ = this->declare_parameter<std::string>("move_group", "arm");

    // ★ MoveIt 전용 노드 생성 (중요)
    mg_node_ = std::make_shared<rclcpp::Node>("pose_to_joint_mg_node");

    // ★ 전용 executor + spin thread
    mg_exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    mg_exec_->add_node(mg_node_);
    mg_spin_thread_ = std::thread([this]() { mg_exec_->spin(); });

    // ★ MoveGroupInterface는 mg_node_로 만든다
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      mg_node_, group_name_);

    // action client는 서비스 노드(this)에 둬도 되고 mg_node_에 둬도 되는데
    // 보통 this에 둬도 OK. (아래는 기존대로)
    if (this->has_parameter("fjt_action_name")) {
      fjt_action_name_ = this->get_parameter("fjt_action_name").as_string();
    } else {
      fjt_action_name_ = this->declare_parameter<std::string>(
        "fjt_action_name", "/arm_controller/follow_joint_trajectory");
    }
    action_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    fjt_client_ = rclcpp_action::create_client<FJT>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      fjt_action_name_,
      action_cb_group_);
    move_group_->setPlanningTime(5.0);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);

    RCLCPP_INFO(get_logger(), "MoveGroup ready. group=%s planning_frame=%s ee=%s",
                move_group_->getName().c_str(),
                move_group_->getPlanningFrame().c_str(),
                move_group_->getEndEffectorLink().c_str());

    service_ = this->create_service<PoseToJointExecute>(
      "pose_to_joint_execute",
      std::bind(&PoseToJointExecuteServer::handle_request, this,
                std::placeholders::_1, std::placeholders::_2));
    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg){
        std::lock_guard<std::mutex> lk(js_mutex_);
        last_js_ = *msg;
      });
  }

  void handle_request(
    const std::shared_ptr<PoseToJointExecute::Request> req,
    std::shared_ptr<PoseToJointExecute::Response> res)
  {
    res->success = false;
    res->message.clear();
    res->joint_state = sensor_msgs::msg::JointState();

    if (!move_group_) {
      res->message = "MoveGroup not initialized yet";
      return;
    }

    geometry_msgs::msg::PoseStamped target_plan;
    if (!transformPoseToPlanningFrame(req->target_pose, target_plan)) {
      res->message = "TF transform to planning frame failed";
      return;
    }

    // -------------------------------
    // NEW: constrained mode 3/4/5
    // target_pose is interpreted as GRIPPER target pose
    // -------------------------------
    if (req->execute_mode == 3 || req->execute_mode == 4 || req->execute_mode == 5) {
      std::vector<std::string> joint_names = {"joint1","joint2","joint3","joint4","joint5"};
      std::vector<double> joint_values;

      const double pos_tol =
        (req->position_tolerance > 0.0) ? req->position_tolerance : constrained_default_pos_tol_;

      const std::string eval_link =
        req->eval_link_name.empty() ? constrained_eval_link_ : req->eval_link_name;

      const double pitch_tol =
        (req->tool_pitch_tolerance >= 0.0) ? req->tool_pitch_tolerance : 0.0;

      const double wrist_tol =
        (req->wrist_tolerance >= 0.0) ? req->wrist_tolerance : 0.0;

      std::string msg;
      if (!computeConstrainedJointValues(
            target_plan,
            req->tool_pitch_sum,
            pitch_tol,
            req->wrist_angle,
            wrist_tol,
            eval_link,
            pos_tol,
            joint_names,
            joint_values,
            msg))
      {
        res->message = msg.empty() ? "Constrained IK failed" : msg;
        return;
      }

      res->joint_state.header.stamp = now();
      res->joint_state.name = joint_names;
      res->joint_state.position = joint_values;

      if (req->execute_mode == 3) {
        res->success = true;
        res->message = "Constrained IK ok (mode=CONSTRAINED_IK_ONLY)";
        return;
      }

      if (req->execute_mode == 4) {
        std::string exec_msg;
        if (!planAndExecuteJointTarget(joint_values, exec_msg)) {
          res->message = exec_msg.empty() ? "MoveIt plan/execute failed" : exec_msg;
          return;
        }
        res->success = true;
        res->message = "Constrained IK ok, MoveIt executed (mode=CONSTRAINED_MOVEIT_PLAN_EXECUTE)";
        return;
      }

      if (req->execute_mode == 5) {
        const double duration = (req->move_duration_sec > 0.0) ? req->move_duration_sec : 2.0;
        std::string exec_msg;
        if (!directExecuteByFJT(joint_names, joint_values, duration, exec_msg)) {
          res->message = exec_msg.empty() ? "Direct controller execution failed" : exec_msg;
          return;
        }
        res->success = true;
        res->message = "Constrained IK ok, direct executed (mode=CONSTRAINED_DIRECT_CONTROLLER)";
        return;
      }
    }

    // -------------------------------
    // existing mode 0/1/2 unchanged
    // -------------------------------
    std::vector<std::string> joint_names;
    std::vector<double> joint_values;
    double ik_timeout = req->ik_timeout_sec > 0.0 ? req->ik_timeout_sec : 0.2;

    if (!computeIKJointValues(target_plan, joint_names, joint_values, ik_timeout)) {
      res->message = "IK failed. Try: increase ik_timeout_sec / relax orientation / check reachability";
      return;
    }

    // 기존 동작 유지
    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (joint_names[i] == "joint5") {
        joint_values[i] = 0.0;
        break;
      }
    }

    res->joint_state.header.stamp = now();
    res->joint_state.name = joint_names;
    res->joint_state.position = joint_values;

    if (req->execute_mode == 0) {
      res->success = true;
      res->message = "IK ok (mode=IK_ONLY)";
      return;
    }

    if (req->execute_mode == 1) {
      std::string msg;
      if (!planAndExecuteJointTarget(joint_values, msg)) {
        res->message = msg.empty() ? "MoveIt plan/execute failed" : msg;
        return;
      }
      res->success = true;
      res->message = "IK ok, MoveIt executed (mode=MOVEIT_PLAN_EXECUTE)";
      return;
    }

    if (req->execute_mode == 2) {
      double duration = (req->move_duration_sec > 0.0) ? req->move_duration_sec : 2.0;
      std::string msg;
      if (!directExecuteByFJT(joint_names, joint_values, duration, msg)) {
        res->message = msg.empty() ? "Direct controller execution failed" : msg;
        return;
      }
      res->success = true;
      res->message = "IK ok, direct executed (mode=DIRECT_CONTROLLER)";
      return;
    }

    res->message = "Invalid execute_mode (use 0,1,2,3,4,5)";
  }
  //constrain Ik
  bool computeConstrainedJointValues(
    const geometry_msgs::msg::PoseStamped& target_pose_in_planning_frame,
    double tool_pitch_sum,
    double tool_pitch_tolerance,
    double wrist_angle,
    double wrist_tolerance,
    const std::string& eval_link_name,
    double position_tolerance,
    std::vector<std::string>& out_joint_names,
    std::vector<double>& out_joint_values,
    std::string& out_msg)
  {
    const auto robot_model = move_group_->getRobotModel();
    const auto* jmg = robot_model->getJointModelGroup(move_group_->getName());
    if (!jmg) {
      out_msg = "JointModelGroup not found: " + move_group_->getName();
      return false;
    }

    if (!robot_model->hasLinkModel(eval_link_name)) {
      out_msg = "eval_link_name not found in RobotModel: " + eval_link_name;
      return false;
    }

    out_joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5"};

    const double px = target_pose_in_planning_frame.pose.position.x;
    const double py = target_pose_in_planning_frame.pose.position.y;
    const double pz = target_pose_in_planning_frame.pose.position.z;

    const double target_r = std::sqrt(px * px + py * py);
    const double target_z = pz;

    sensor_msgs::msg::JointState js_copy;
    {
      std::lock_guard<std::mutex> lk(js_mutex_);
      js_copy = last_js_;
    }

    auto get_bound = [&](const std::string& name, double fallback_min, double fallback_max) {
      double mn = fallback_min;
      double mx = fallback_max;
      try {
        const auto& vb = robot_model->getVariableBounds(name);
        mn = vb.min_position_;
        mx = vb.max_position_;
      } catch (...) {
      }
      return std::pair<double, double>{mn, mx};
    };

    auto clamp = [](double v, double lo, double hi) {
      return std::max(lo, std::min(hi, v));
    };

    auto normalize_angle = [](double a) {
      while (a > M_PI) a -= 2.0 * M_PI;
      while (a < -M_PI) a += 2.0 * M_PI;
      return a;
    };

    const auto [q1_min, q1_max] = get_bound("joint1", -3.13287,  M_PI);
    const auto [q2_min, q2_max] = get_bound("joint2", -2.07694,  1.57);
    const auto [q3_min, q3_max] = get_bound("joint3", -2.07694,  1.57);
    const auto [q4_min, q4_max] = get_bound("joint4", -1.72788,  1.72788);
    const auto [q5_min, q5_max] = get_bound("joint5", -1.72788,  1.72788);

    const double q1_seed = std::atan2(py, px);
    std::vector<double> q1_candidates = {
      clamp(normalize_angle(q1_seed), q1_min, q1_max),
      clamp(normalize_angle(q1_seed + M_PI), q1_min, q1_max)
    };

    const double q5_fixed = clamp(wrist_angle, q5_min, q5_max);

    RCLCPP_INFO(
      get_logger(),
      "[execute5][planar] target xyz=(%.6f, %.6f, %.6f) target_r=%.6f target_z=%.6f "
      "pitch=%.6f pitch_tol=%.6f wrist=%.6f wrist_tol=%.6f pos_tol=%.6f eval_link=%s",
      px, py, pz, target_r, target_z,
      tool_pitch_sum, tool_pitch_tolerance,
      wrist_angle, wrist_tolerance,
      position_tolerance, eval_link_name.c_str());

    RCLCPP_INFO(
      get_logger(),
      "[execute5][planar] bounds q1=[%.4f, %.4f] q2=[%.4f, %.4f] q3=[%.4f, %.4f] "
      "q4=[%.4f, %.4f] q5=[%.4f, %.4f] q1_seed=%.4f q1_alt=%.4f q5_fixed=%.4f",
      q1_min, q1_max, q2_min, q2_max, q3_min, q3_max, q4_min, q4_max, q5_min, q5_max,
      q1_candidates[0], q1_candidates[1], q5_fixed);

    auto make_state_from_seed = [&]() {
      moveit::core::RobotState state(robot_model);
      state.setToDefaultValues();

      if (!js_copy.name.empty() && js_copy.name.size() == js_copy.position.size()) {
        for (size_t i = 0; i < js_copy.name.size(); ++i) {
          try {
            (void)robot_model->getVariableIndex(js_copy.name[i]);
            state.setVariablePosition(js_copy.name[i], js_copy.position[i]);
          } catch (...) {
          }
        }
      }
      state.update();
      return state;
    };

    auto eval_state_rz_xyz = [&](double q1, double q2, double q3, double q5,
                                Eigen::Vector3d& p_out,
                                double& r_out,
                                double& z_out,
                                moveit::core::RobotState& state_out,
                                std::string& err) -> bool {
      moveit::core::RobotState state = make_state_from_seed();

      const double q4 = tool_pitch_sum - q2 - q3;
      if (q4 < q4_min || q4 > q4_max) {
        err = "q4 out of bounds";
        return false;
      }

      try {
        state.setVariablePosition("joint1", q1);
        state.setVariablePosition("joint2", q2);
        state.setVariablePosition("joint3", q3);
        state.setVariablePosition("joint4", q4);
        state.setVariablePosition("joint5", q5);
      } catch (const std::exception& e) {
        err = std::string("setVariablePosition failed: ") + e.what();
        return false;
      }

      state.update();

      if (!state.satisfiesBounds(jmg)) {
        err = "state violates bounds";
        return false;
      }

      const Eigen::Isometry3d& tf_eval = state.getGlobalLinkTransform(eval_link_name);
      p_out = tf_eval.translation();
      r_out = std::sqrt(p_out.x() * p_out.x() + p_out.y() * p_out.y());
      z_out = p_out.z();

      state_out = state;
      return true;
    };

    auto eval_error_vec = [&](double q1, double q2, double q3, double q5,
                              Eigen::Vector2d& e_out,
                              double& pos_err_out,
                              moveit::core::RobotState& state_out,
                              Eigen::Vector3d& p_out,
                              std::string& err) -> bool {
      double r = 0.0, z = 0.0;
      if (!eval_state_rz_xyz(q1, q2, q3, q5, p_out, r, z, state_out, err)) {
        return false;
      }

      e_out[0] = r - target_r;
      e_out[1] = z - target_z;

      const double ex = p_out.x() - px;
      const double ey = p_out.y() - py;
      const double ez = p_out.z() - pz;
      pos_err_out = std::sqrt(ex * ex + ey * ey + ez * ez);
      return true;
    };

    moveit::core::RobotState best_state(robot_model);
    best_state.setToDefaultValues();
    double best_score = std::numeric_limits<double>::infinity();
    bool found_any = false;

    double best_q1 = 0.0, best_q2 = 0.0, best_q3 = 0.0, best_q4 = 0.0, best_q5 = 0.0;
    double best_pos_err = std::numeric_limits<double>::infinity();
    double best_r_err = std::numeric_limits<double>::infinity();
    double best_z_err = std::numeric_limits<double>::infinity();

    auto get_js_pos = [&](const std::string& name, double fallback) {
      for (size_t i = 0; i < js_copy.name.size(); ++i) {
        if (js_copy.name[i] == name && i < js_copy.position.size()) {
          return js_copy.position[i];
        }
      }
      return fallback;
    };

    const double q2_seed_default = clamp(get_js_pos("joint2", 0.0), q2_min, q2_max);
    const double q3_seed_default = clamp(get_js_pos("joint3", 0.0), q3_min, q3_max);

    RCLCPP_INFO(
      get_logger(),
      "[execute5][planar] seed from joint_states q2=%.6f q3=%.6f",
      q2_seed_default, q3_seed_default);

    const int max_iters = 60;
    const double fd_eps = 1e-3;
    const double lambda = 1e-2;
    const double step_limit = 0.12;
    const double rz_tol = std::max(1e-4, position_tolerance * 0.5);

    for (size_t c = 0; c < q1_candidates.size(); ++c) {
      const double q1 = q1_candidates[c];

      double q2 = q2_seed_default;
      double q3 = q3_seed_default;

      {
        double q4 = tool_pitch_sum - q2 - q3;
        if (q4 < q4_min || q4 > q4_max) {
          const double q4_clamped = clamp(q4, q4_min, q4_max);
          const double desired_sum23 = tool_pitch_sum - q4_clamped;
          const double cur_sum23 = q2 + q3;
          const double delta = 0.5 * (desired_sum23 - cur_sum23);
          q2 = clamp(q2 + delta, q2_min, q2_max);
          q3 = clamp(q3 + delta, q3_min, q3_max);

          RCLCPP_WARN(
            get_logger(),
            "[execute5][planar] q1 candidate %zu seed corrected for q4 bound: "
            "raw_q4=%.6f clamped_q4=%.6f -> q2=%.6f q3=%.6f",
            c, q4, q4_clamped, q2, q3);
        }
      }

      RCLCPP_INFO(
        get_logger(),
        "[execute5][planar] start candidate %zu q1=%.6f q2_seed=%.6f q3_seed=%.6f q4_seed=%.6f q5=%.6f",
        c, q1, q2, q3, tool_pitch_sum - q2 - q3, q5_fixed);

      for (int iter = 0; iter < max_iters; ++iter) {
        moveit::core::RobotState cur_state(robot_model);
        Eigen::Vector3d p_cur;
        Eigen::Vector2d e;
        double pos_err = std::numeric_limits<double>::infinity();
        std::string err;

        if (!eval_error_vec(q1, q2, q3, q5_fixed, e, pos_err, cur_state, p_cur, err)) {
          RCLCPP_WARN(
            get_logger(),
            "[execute5][planar] cand=%zu iter=%d eval failed q=(%.5f, %.5f, %.5f, %.5f, %.5f): %s",
            c, iter,
            q1, q2, q3, tool_pitch_sum - q2 - q3, q5_fixed,
            err.c_str());

          q2 = clamp(q2 + 0.03, q2_min, q2_max);
          q3 = clamp(q3 - 0.03, q3_min, q3_max);
          continue;
        }

        const double q4 = tool_pitch_sum - q2 - q3;
        const double pitch_dev = std::abs((q2 + q3 + q4) - tool_pitch_sum);
        const double wrist_dev = std::abs(normalize_angle(q5_fixed - wrist_angle));
        const double score = pos_err + 0.02 * pitch_dev + 0.02 * wrist_dev;

        RCLCPP_INFO(
          get_logger(),
          "[execute5][planar] cand=%zu iter=%d q=(%.5f, %.5f, %.5f, %.5f, %.5f) "
          "eval_xyz=(%.6f, %.6f, %.6f) r=%.6f z=%.6f "
          "err_r=%.6f err_z=%.6f pos_err=%.6f score=%.6f "
          "pitch_dev=%.6f wrist_dev=%.6f",
          c, iter,
          q1, q2, q3, q4, q5_fixed,
          p_cur.x(), p_cur.y(), p_cur.z(),
          std::sqrt(p_cur.x() * p_cur.x() + p_cur.y() * p_cur.y()),
          p_cur.z(),
          e[0], e[1], pos_err, score,
          pitch_dev, wrist_dev);

        if (score < best_score) {
          best_score = score;
          best_state = cur_state;
          found_any = true;

          best_q1 = q1;
          best_q2 = q2;
          best_q3 = q3;
          best_q4 = q4;
          best_q5 = q5_fixed;
          best_pos_err = pos_err;
          best_r_err = e[0];
          best_z_err = e[1];

          RCLCPP_INFO(
            get_logger(),
            "[execute5][planar] new best cand=%zu iter=%d score=%.6f "
            "best_q=(%.5f, %.5f, %.5f, %.5f, %.5f) pos_err=%.6f err_r=%.6f err_z=%.6f",
            c, iter, best_score,
            best_q1, best_q2, best_q3, best_q4, best_q5,
            best_pos_err, best_r_err, best_z_err);
        }

        if (std::abs(e[0]) <= rz_tol &&
            std::abs(e[1]) <= rz_tol &&
            pos_err <= position_tolerance &&
            pitch_dev <= std::abs(tool_pitch_tolerance) + 1e-6 &&
            wrist_dev <= std::abs(wrist_tolerance) + 1e-6) {
          out_joint_values.clear();
          cur_state.copyJointGroupPositions(jmg, out_joint_values);

          RCLCPP_INFO(
            get_logger(),
            "[execute5][planar] success cand=%zu iter=%d final_q=(%.5f, %.5f, %.5f, %.5f, %.5f) "
            "pos_err=%.6f err_r=%.6f err_z=%.6f",
            c, iter, q1, q2, q3, q4, q5_fixed, pos_err, e[0], e[1]);

          out_msg = "Constrained IK solved by planar iterative method";
          return true;
        }

        Eigen::Matrix2d J;
        for (int col = 0; col < 2; ++col) {
          double q2p = q2;
          double q3p = q3;
          if (col == 0) q2p += fd_eps;
          else          q3p += fd_eps;

          moveit::core::RobotState tmp_state(robot_model);
          Eigen::Vector3d p_tmp;
          Eigen::Vector2d e_tmp;
          double pos_err_tmp = 0.0;
          std::string err_tmp;

          if (!eval_error_vec(q1, q2p, q3p, q5_fixed, e_tmp, pos_err_tmp, tmp_state, p_tmp, err_tmp)) {
            J(0, col) = 0.0;
            J(1, col) = 0.0;

            RCLCPP_WARN(
              get_logger(),
              "[execute5][planar] cand=%zu iter=%d fd failed col=%d q2p=%.5f q3p=%.5f : %s",
              c, iter, col, q2p, q3p, err_tmp.c_str());
            continue;
          }

          J(0, col) = (e_tmp[0] - e[0]) / fd_eps;
          J(1, col) = (e_tmp[1] - e[1]) / fd_eps;
        }

        const Eigen::Matrix2d A = J.transpose() * J + lambda * lambda * Eigen::Matrix2d::Identity();
        const Eigen::Vector2d g = J.transpose() * e;
        Eigen::Vector2d dq = -A.ldlt().solve(g);

        RCLCPP_INFO(
          get_logger(),
          "[execute5][planar] cand=%zu iter=%d J=[[%.6f, %.6f],[%.6f, %.6f]] "
          "g=(%.6f, %.6f) raw_dq=(%.6f, %.6f)",
          c, iter,
          J(0,0), J(0,1), J(1,0), J(1,1),
          g[0], g[1], dq[0], dq[1]);

        dq[0] = clamp(dq[0], -step_limit, step_limit);
        dq[1] = clamp(dq[1], -step_limit, step_limit);

        double q2_next = clamp(q2 + dq[0], q2_min, q2_max);
        double q3_next = clamp(q3 + dq[1], q3_min, q3_max);

        double q4_next = tool_pitch_sum - q2_next - q3_next;
        if (q4_next < q4_min || q4_next > q4_max) {
          const double q4_clamped = clamp(q4_next, q4_min, q4_max);
          const double desired_sum23 = tool_pitch_sum - q4_clamped;
          const double cur_sum23 = q2_next + q3_next;
          const double corr = 0.5 * (desired_sum23 - cur_sum23);

          RCLCPP_WARN(
            get_logger(),
            "[execute5][planar] cand=%zu iter=%d q4 correction before=(q2=%.5f q3=%.5f q4=%.5f) "
            "desired_sum23=%.5f cur_sum23=%.5f corr=%.5f",
            c, iter, q2_next, q3_next, q4_next, desired_sum23, cur_sum23, corr);

          q2_next = clamp(q2_next + corr, q2_min, q2_max);
          q3_next = clamp(q3_next + corr, q3_min, q3_max);
          q4_next = tool_pitch_sum - q2_next - q3_next;

          RCLCPP_WARN(
            get_logger(),
            "[execute5][planar] cand=%zu iter=%d q4 correction after=(q2=%.5f q3=%.5f q4=%.5f)",
            c, iter, q2_next, q3_next, q4_next);
        }

        RCLCPP_INFO(
          get_logger(),
          "[execute5][planar] cand=%zu iter=%d dq_clamped=(%.6f, %.6f) next_q=(%.5f, %.5f, %.5f)",
          c, iter, dq[0], dq[1], q2_next, q3_next, q4_next);

        if (std::abs(q2_next - q2) < 1e-5 && std::abs(q3_next - q3) < 1e-5) {
          RCLCPP_WARN(
            get_logger(),
            "[execute5][planar] cand=%zu iter=%d stagnation: q2/q3 update too small",
            c, iter);
          break;
        }

        q2 = q2_next;
        q3 = q3_next;
      }

      RCLCPP_WARN(
        get_logger(),
        "[execute5][planar] candidate %zu finished without exact solution",
        c);
    }

    if (found_any) {
      out_joint_values.clear();
      best_state.copyJointGroupPositions(jmg, out_joint_values);

      RCLCPP_WARN(
        get_logger(),
        "[execute5][planar] best candidate only: "
        "best_q=(%.5f, %.5f, %.5f, %.5f, %.5f) "
        "best_score=%.6f best_pos_err=%.6f best_err_r=%.6f best_err_z=%.6f",
        best_q1, best_q2, best_q3, best_q4, best_q5,
        best_score, best_pos_err, best_r_err, best_z_err);

      out_msg = "No exact solution within tolerance; returning best candidate from planar iterative solver";
      return false;
    }

    RCLCPP_ERROR(
      get_logger(),
      "[execute5][planar] no candidate found at all for target xyz=(%.6f, %.6f, %.6f)",
      px, py, pz);

    out_msg = "No constrained IK candidate found by planar iterative solver";
    return false;
  }
  bool transformPoseToPlanningFrame(
    const geometry_msgs::msg::PoseStamped& in_pose,
    geometry_msgs::msg::PoseStamped& out_pose)
  {
    const std::string planning_frame = move_group_->getPlanningFrame();

    if (in_pose.header.frame_id.empty()) {
      RCLCPP_ERROR(get_logger(), "Request pose frame_id is empty");
      return false;
    }

    if (in_pose.header.frame_id == planning_frame) {
      out_pose = in_pose;
      return true;
    }

    try {
      const auto tf = tf_buffer_.lookupTransform(
        planning_frame, in_pose.header.frame_id, tf2::TimePointZero);

      tf2::doTransform(in_pose, out_pose, tf);
      out_pose.header.frame_id = planning_frame;
      out_pose.header.stamp = now();
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "TF failed: %s", e.what());
      return false;
    }
  }

  bool computeIKJointValues(
    const geometry_msgs::msg::PoseStamped& target_in_planning_frame,
    std::vector<std::string>& out_joint_names,
    std::vector<double>& out_joint_values,
    double ik_timeout_sec)
  {
    const auto robot_model = move_group_->getRobotModel();
    const auto* jmg = robot_model->getJointModelGroup(move_group_->getName());
    if (!jmg) {
      RCLCPP_ERROR(get_logger(), "JointModelGroup not found: %s", move_group_->getName().c_str());
      return false;
    }
    moveit::core::RobotState state(robot_model);
    state.setToDefaultValues();

    // ★ 최근 joint_states를 seed로 반영
    sensor_msgs::msg::JointState js_copy;
    {
      std::lock_guard<std::mutex> lk(js_mutex_);
      js_copy = last_js_;
    }

    if (!js_copy.name.empty() && js_copy.name.size() == js_copy.position.size()) {
      // MoveIt의 variable order는 URDF/MoveIt에 의해 정해지므로
      // 이름 매칭으로 값을 넣어준다
      for (size_t i = 0; i < js_copy.name.size(); ++i) {
        const std::string& jn = js_copy.name[i];
        double pos = js_copy.position[i];
	
	try {
    	     (void)robot_model->getVariableIndex(jn);  // 없으면 예외 throw :contentReference[oaicite:2]{index=2}
             state.setVariablePosition(jn, pos);       // 존재하면 세팅
        } catch (const std::exception&) {
        // 모델에 없는 joint/variable 이름이면 무시
          continue;
        }
       }
       state.update();
    } else {
      RCLCPP_WARN(get_logger(), "No joint_states received yet -> using default seed.");
    }

    // IK
    bool ok = state.setFromIK(jmg, target_in_planning_frame.pose, ik_timeout_sec);
    if (!ok) return false;

    out_joint_names = jmg->getVariableNames();
    state.copyJointGroupPositions(jmg, out_joint_values);
    return true;
  }

  bool planAndExecuteJointTarget(const std::vector<double>& joint_values, std::string& out_msg)
  {
    move_group_->setJointValueTarget(joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_code = move_group_->plan(plan);
    if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
      out_msg = "Planning failed (MoveItErrorCode=" + std::to_string(plan_code.val) + ")";
      return false;
    }

    auto exec_code = move_group_->execute(plan);
    if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
      out_msg = "Execute failed (MoveItErrorCode=" + std::to_string(exec_code.val) + ")";
      return false;
    }

    return true;
  }
  bool directExecuteByFJT(const std::vector<std::string>& joint_names,
                          const std::vector<double>& joint_positions,
                          double duration_sec,
                          std::string& out_msg)
  {
    if (!fjt_client_) {
      out_msg = "FJT action client not initialized";
      return false;
    }

    if (joint_names.empty() || joint_positions.empty() || joint_names.size() != joint_positions.size()) {
      out_msg = "Invalid joint_names/positions";
      return false;
    }

    if (!fjt_client_->wait_for_action_server(std::chrono::seconds(2))) {
      out_msg = "FollowJointTrajectory action server not available: " + fjt_action_name_;
      return false;
    }

    FJT::Goal goal;
    goal.trajectory.joint_names = joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = joint_positions;
    p.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    goal.trajectory.points.push_back(p);

    // 동기적으로 결과 기다리기(서비스 요청-응답이라 깔끔)
    auto goal_future = fjt_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      out_msg = "Failed to send FJT goal (timeout)";
      return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      out_msg = "FJT goal was rejected by server";
      return false;
    }

    auto result_future = fjt_client_->async_get_result(goal_handle);
    // duration + 여유
    auto wait_time = std::chrono::milliseconds(static_cast<int>((duration_sec + 2.0) * 1000.0));
    if (result_future.wait_for(wait_time) != std::future_status::ready) {
    out_msg = "FJT result timeout";
    return false;
    }

    auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
      out_msg = "FJT failed (ResultCode=" + std::to_string(static_cast<int>(wrapped.code)) + ")";
      return false;
    }

    return true;
  }
private:
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::Service<PoseToJointExecute>::SharedPtr service_;

  rclcpp::Node::SharedPtr mg_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> mg_exec_;
  std::thread mg_spin_thread_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  sensor_msgs::msg::JointState last_js_;
  std::mutex js_mutex_;
  
  std::string group_name_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  rclcpp::CallbackGroup::SharedPtr action_cb_group_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Client<FJT>::SharedPtr fjt_client_;
  std::string fjt_action_name_;

  std::string constrained_eval_link_;
  double constrained_default_pos_tol_{0.01};
  double constrained_search_step_rad_{0.05};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseToJointExecuteServer>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}


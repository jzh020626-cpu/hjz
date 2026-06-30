#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <map>
#include <atomic>
#include <thread>
#include <mutex>
#include <optional>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/motor_command.hpp"
#include "base_interfaces_demo/msg/motor_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MathUtils {
public:
    static double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

class HuataiControlNode : public rclcpp::Node
{
public:
    HuataiControlNode()
    : Node("huatai_control_node"), poses_initialized_(false), motors_initialized_(false)
    {
        publisher1_ = create_publisher<base_interfaces_demo::msg::MotorCommand>("/huatai1_pos_spe_pd", 10);
        publisher2_ = create_publisher<base_interfaces_demo::msg::MotorCommand>("/huatai2_pos_spe_pd", 10);
        publisher3_ = create_publisher<base_interfaces_demo::msg::MotorCommand>("/huatai3_pos_spe_pd", 10);

        current_car_poses_.resize(3, Eigen::Affine3d::Identity());
        motor_zeros_.resize(3);
        motor_zeros_captured_.resize(3, false);

        current_local_pts_.resize(3, Eigen::Vector3d::Zero());
        grab_points_in_obj_.resize(3, Eigen::Vector3d::Zero());
        initial_tips_local_.resize(3, Eigen::Vector3d::Zero());

        motor_init_flags_ = {false, false, false};
        received_poses_.resize(4, false);
        theta_unwrapped_.resize(4, 0.0);
        prev_raw_theta_.resize(4, std::nullopt);

        // 刚体名称（用于诊断）
        rigid_names_ = {"Rigid17(Car1)", "Rigid14(Car2)", "Rigid15(Car3)", "Rigid8(Obj)"};

        init_motor_subscriptions();
        init_pose_subscriptions();

        // 外部指令接口 (Web UI)
        status_pub_ = create_publisher<std_msgs::msg::String>("/tiaozi_status", 10);
        cmd_sub_ = create_subscription<std_msgs::msg::String>("/tiaozi_cmd", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) { handle_external_cmd(msg); });

        init_preset_commands();

        RCLCPP_INFO(this->get_logger(), "=== 华台协同调姿控制节点启动 ===");
        RCLCPP_INFO(this->get_logger(), "所有位移基于 OptiTrack 动捕当前位姿");
        input_thread_ = std::thread(&HuataiControlNode::read_user_input, this);
    }

    ~HuataiControlNode() { if (input_thread_.joinable()) input_thread_.join(); }

private:
    // --- 配置 ---
    const double POSITION_TOLERANCE = 1.0;
    const double MAX_TRANS_SPEED = 10.0; // mm/s
    const double MAX_ROT_SPEED = 0.5;   // deg/s
    const double X_MIN = 1.0, X_MAX = 275.0, Y_MIN = 1.0, Y_MAX = 275.0, Z_MIN = 1.0, Z_MAX = 195.0;

    // --- 运动学 ---
    Eigen::Matrix3d euler_to_matrix(double r, double p, double y) {
        return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())).toRotationMatrix();
    }

    Eigen::Vector3d get_euler_stable(const Eigen::Matrix3d& R) {
        double r, p, y;
        p = -asin(R(2,0));
        if (cos(p) > 0.001) {
            r = atan2(R(2,1), R(2,2));
            y = atan2(R(1,0), R(0,0));
        } else {
            r = 0.0; y = atan2(-R(0,1), R(1,1));
        }
        return Eigen::Vector3d(r*180.0/M_PI, p*180.0/M_PI, y*180.0/M_PI);
    }

    double calculate_move_time(const Eigen::Affine3d& start, const Eigen::Affine3d& end) {
        double d_p = (end.translation() - start.translation()).norm();
        Eigen::Vector3d s_e = get_euler_stable(start.linear());
        Eigen::Vector3d e_e = get_euler_stable(end.linear());
        double d_r = std::max({std::abs(MathUtils::normalize_angle((e_e[0]-s_e[0])*M_PI/180.0)),
                               std::abs(MathUtils::normalize_angle((e_e[1]-s_e[1])*M_PI/180.0)),
                               std::abs(MathUtils::normalize_angle((e_e[2]-s_e[2])*M_PI/180.0))}) * 180.0/M_PI;
        return std::max({d_p / MAX_TRANS_SPEED, d_r / MAX_ROT_SPEED, 2.0});
    }

    // --- 电机状态回调 ---
    void handle_motor_status(int id, const base_interfaces_demo::msg::MotorStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (!motor_zeros_captured_[id]) {
            motor_zeros_[id] << msg->x, msg->y, msg->z;
            motor_zeros_captured_[id] = true;
            RCLCPP_INFO(this->get_logger(), "滑台 %d 零点: [%.2f, %.2f, %.2f]",
                        id + 1, msg->x, msg->y, msg->z);
        }

        current_local_pts_[id] << msg->x, msg->y, msg->z;

        if (!motor_init_flags_[id]) {
            motor_init_flags_[id] = true;
            motors_initialized_ = true;
            if (poses_initialized_) print_motor_status();
        }
    }

    // --- 位姿处理 ---
    Eigen::Affine3d process_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg, size_t idx) {
        double rx_rad = msg->pose.orientation.x * M_PI / 180.0;
        double ry_rad = -msg->pose.orientation.z * M_PI / 180.0;
        double rz_rad = msg->pose.orientation.y * M_PI / 180.0;
        if (!prev_raw_theta_[idx].has_value()) theta_unwrapped_[idx] = rz_rad;
        else theta_unwrapped_[idx] += MathUtils::normalize_angle(rz_rad - prev_raw_theta_[idx].value());
        prev_raw_theta_[idx] = rz_rad;

        Eigen::Affine3d t = Eigen::Affine3d::Identity();
        t.translate(Eigen::Vector3d(msg->pose.position.x, -msg->pose.position.z, msg->pose.position.y));
        t.linear() = euler_to_matrix(rx_rad, ry_rad, theta_unwrapped_[idx]);
        return t;
    }

    // --- 预设指令 ---
    void init_preset_commands() {
        preset_commands_ = {
            {0,    0,   0,   0,  0,  0,  1.0},
            {0,    0,  80,   0,  0,  0,  5.0},
            {0,    0,   0,  -4,  0,  0,  5.0},
            {0,    0,   0,   0, -4,  0,  5.0},
            {0,    0,   0,   0,  0, -4,  5.0},
            {0,    0,   0,   0,  0,  4,  5.0},
            {0,    0,   0,   0,  4,  0,  5.0},
            {0,    0,   0,   4,  0,  0,  5.0},
            {0,    0,   0,   0,  4,  0,  5.0},
            {50.0, 0,   0,   0,  0,  0,  5.0},
            {0,   50.0, 0,   0,  0,  0,  5.0},
        };
    }

    void execute_preset_commands() {
        RCLCPP_INFO(this->get_logger(), ">>> 执行预设序列 (共 %zu 条) <<<", preset_commands_.size());
        for (size_t i = 0; i < preset_commands_.size(); ++i) {
            const auto& cmd = preset_commands_[i];
            RCLCPP_INFO(this->get_logger(), "[%zu/%zu]", i + 1, preset_commands_.size());
            execute_tiaozi(cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        RCLCPP_INFO(this->get_logger(), ">>> 预设序列完成 <<<");
    }

    // --- 核心调姿 (基于动捕相对位移) ---
    void execute_tiaozi(double dx, double dy, double dz, double rx, double ry, double rz, double time_min) {
        if (!poses_initialized_) {
            RCLCPP_WARN(this->get_logger(), "等待位姿初始化...");
            return;
        }

        const double MAX_ANGLE_DEG = 1.0;
        auto clamp_angle = [&](double v) { return std::max(-MAX_ANGLE_DEG, std::min(MAX_ANGLE_DEG, v)); };
        rx = clamp_angle(rx); ry = clamp_angle(ry); rz = clamp_angle(rz);

        std::vector<int> active;
        for (int i = 0; i < 3; ++i) if (motor_zeros_captured_[i]) active.push_back(i);
        if (active.empty()) {
            RCLCPP_WARN(this->get_logger(), "无滑台就绪");
            publish_status_json("idle");
            return;
        }

        // 锁保护：读取位姿和计算目标必须在同一帧内
        std::lock_guard<std::mutex> lock(state_mutex_);

        try {
            // T_target = 当前动捕位姿 + 增量
            Eigen::Affine3d T_target = Eigen::Affine3d::Identity();
            T_target.translation() = obj_pose_curr_.translation() + Eigen::Vector3d(dx, dy, dz);
            T_target.linear() = euler_to_matrix(rx*M_PI/180.0, ry*M_PI/180.0, rz*M_PI/180.0) * obj_pose_curr_.linear();

            double move_time = std::max(calculate_move_time(obj_pose_curr_, T_target), time_min);
            publish_status_json("moving");

            // 诊断：当前物体位姿
            auto obj_pos = obj_pose_curr_.translation();
            auto obj_euler = get_euler_stable(obj_pose_curr_.linear());
            RCLCPP_INFO(this->get_logger(),
                ">>> 诊断: 物体当前位姿 Pos(%.1f,%.1f,%.1f) Rot(%.2f,%.2f,%.2f)deg | 目标增量 d(%.1f,%.1f,%.1f) dR(%.2f,%.2f,%.2f)deg",
                obj_pos.x(), obj_pos.y(), obj_pos.z(),
                obj_euler[0], obj_euler[1], obj_euler[2],
                dx, dy, dz, rx, ry, rz);

            std::vector<Eigen::Vector3d> target_m(3, Eigen::Vector3d::Zero());
            for (int i : active) {
                // 诊断：每个滑台的计算过程
                auto car_pos = current_car_poses_[i].translation();
                auto car_euler = get_euler_stable(current_car_poses_[i].linear());
                Eigen::Vector3d gp_world = obj_pose_curr_ * grab_points_in_obj_[i];
                Eigen::Vector3d gp_target_world = T_target * grab_points_in_obj_[i];
                Eigen::Vector3d P_tip_l = current_car_poses_[i].inverse() * gp_target_world;

                target_m[i] = (P_tip_l - initial_tips_local_[i]) + motor_zeros_[i];

                RCLCPP_INFO(this->get_logger(),
                    ">>> 滑台%d: carPos(%.1f,%.1f,%.1f) carRot(%.1f,%.1f,%.1f)deg",
                    i+1, car_pos.x(), car_pos.y(), car_pos.z(),
                    car_euler[0], car_euler[1], car_euler[2]);
                RCLCPP_INFO(this->get_logger(),
                    "    grab_world(%.1f,%.1f,%.1f) -> target_world(%.1f,%.1f,%.1f) -> tip_local(%.1f,%.1f,%.1f)",
                    gp_world.x(), gp_world.y(), gp_world.z(),
                    gp_target_world.x(), gp_target_world.y(), gp_target_world.z(),
                    P_tip_l.x(), P_tip_l.y(), P_tip_l.z());
                RCLCPP_INFO(this->get_logger(),
                    "    init_tip_local(%.1f,%.1f,%.1f) motor_zero(%.1f,%.1f,%.1f) -> target(%.1f,%.1f,%.1f)",
                    initial_tips_local_[i].x(), initial_tips_local_[i].y(), initial_tips_local_[i].z(),
                    motor_zeros_[i].x(), motor_zeros_[i].y(), motor_zeros_[i].z(),
                    target_m[i].x(), target_m[i].y(), target_m[i].z());

                if (target_m[i].x() < X_MIN-0.1 || target_m[i].x() > X_MAX+0.1 ||
                    target_m[i].y() < Y_MIN-0.1 || target_m[i].y() > Y_MAX+0.1 ||
                    target_m[i].z() < Z_MIN-0.1 || target_m[i].z() > Z_MAX+0.1) {
                    RCLCPP_WARN(this->get_logger(), "行程拦截: 滑台%d [%.1f,%.1f,%.1f]", i+1, target_m[i].x(), target_m[i].y(), target_m[i].z());
                    publish_status_json("error");
                    return;
                }
            }

            RCLCPP_INFO(this->get_logger(), ">>> D(%.1f,%.1f,%.1f) R(%.2f,%.2f,%.2f)deg | %d台 %.1fs",
                dx, dy, dz, rx, ry, rz, (int)active.size(), move_time);
            for (int i : active)
                RCLCPP_INFO(this->get_logger(), " 滑台%d: X%.2f Y%.2f Z%.2f", i+1, target_m[i].x(), target_m[i].y(), target_m[i].z());
            for (int i = 0; i < 3; ++i)
                if (!motor_zeros_captured_[i]) RCLCPP_WARN(this->get_logger(), " 滑台%d [未就绪] 跳过", i+1);

            std::vector<rclcpp::Publisher<base_interfaces_demo::msg::MotorCommand>::SharedPtr> pubs = {publisher1_, publisher2_, publisher3_};
            for (int i : active) {
                auto msg = base_interfaces_demo::msg::MotorCommand();
                msg.command_type = "position";
                msg.x = target_m[i].x(); msg.y = target_m[i].y(); msg.z = target_m[i].z();
                msg.time = move_time;
                msg.is_relative = false;
                pubs[i]->publish(msg);
            }

            wait_for_arrival(target_m, active, move_time + 1.5);

            auto p = obj_pose_curr_.translation();
            auto e = get_euler_stable(obj_pose_curr_.linear());
            RCLCPP_INFO(this->get_logger(), ">>> 完成: Pos(%.2f,%.2f,%.2f) Rot(%.2f,%.2f,%.2f)", p.x(), p.y(), p.z(), e[0], e[1], e[2]);

            publish_status_json("idle");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "异常: %s", e.what());
            publish_status_json("error");
        }
    }

    // --- 外部指令处理 (Web UI) ---
    void handle_external_cmd(const std_msgs::msg::String::SharedPtr msg) {
        std::string payload = msg->data;
        try {
            if (payload.find("\"preset\"") != std::string::npos) {
                execute_preset_commands();
                return;
            }
            std::vector<double> vals;
            std::vector<const char*> keys = {"\"dx\"", "\"dy\"", "\"dz\"", "\"rx\"", "\"ry\"", "\"rz\"", "\"time\""};
            for (const char* key : keys) {
                size_t pos = payload.find(key);
                if (pos == std::string::npos) { vals.clear(); break; }
                size_t colon = payload.find(':', pos);
                size_t end = payload.find_first_of(",}\n\r \t", colon+1);
                std::string num = payload.substr(colon+1, end-colon-1);
                vals.push_back(std::stod(num));
            }
            if (vals.size() == 7) {
                execute_tiaozi(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6]);
            } else {
                RCLCPP_WARN(this->get_logger(), "无效指令: %s", payload.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "指令解析失败: %s", e.what());
        }
    }

    // --- 状态发布 (JSON) ---
    void publish_status_json(const std::string& state) {
        auto p = obj_pose_curr_.translation();
        auto e = get_euler_stable(obj_pose_curr_.linear());
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "{"
           << "\"obj_x\":" << p.x() << ",\"obj_y\":" << p.y() << ",\"obj_z\":" << p.z() << ","
           << "\"obj_roll\":" << e[0] << ",\"obj_pitch\":" << e[1] << ",\"obj_yaw\":" << e[2] << ","
           << "\"poses_ready\":["
           << (received_poses_[0]?"true":"false") << ","
           << (received_poses_[1]?"true":"false") << ","
           << (received_poses_[2]?"true":"false") << ","
           << (received_poses_[3]?"true":"false") << "],"
           << "\"motor_ready\":["
           << (motor_zeros_captured_[0]?"true":"false") << ","
           << (motor_zeros_captured_[1]?"true":"false") << ","
           << (motor_zeros_captured_[2]?"true":"false") << "],"
           << "\"motor_x\":[" << current_local_pts_[0].x() << "," << current_local_pts_[1].x() << "," << current_local_pts_[2].x() << "],"
           << "\"motor_y\":[" << current_local_pts_[0].y() << "," << current_local_pts_[1].y() << "," << current_local_pts_[2].y() << "],"
           << "\"motor_z\":[" << current_local_pts_[0].z() << "," << current_local_pts_[1].z() << "," << current_local_pts_[2].z() << "],"
           << "\"state\":\"" << state << "\""
           << "}";
        auto out = std_msgs::msg::String();
        out.data = ss.str();
        status_pub_->publish(out);
    }

    // --- 初始化检查 ---
    void check_ready_internal() {
        if (poses_initialized_) return;

        // 显示哪些刚体已就绪
        int ready = 0;
        std::string missing;
        for (int i = 0; i < 4; ++i) {
            if (received_poses_[i]) ready++;
            else missing += rigid_names_[i] + " ";
        }

        if (ready < 4) {
            // 每 5 秒输出一次状态，避免刷屏
            static auto last_log = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - last_log).count() > 5.0) {
                RCLCPP_INFO(this->get_logger(), ">>> 动捕初始化等待: %d/4 刚体就绪, 缺失: %s <<<", ready, missing.c_str());
                last_log = now;
            }
            return;
        }

        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3d p_init_w(current_car_poses_[i].translation().x(),
                                     current_car_poses_[i].translation().y(),
                                     obj_pose_curr_.translation().z());
            grab_points_in_obj_[i] = obj_pose_curr_.inverse() * p_init_w;
            initial_tips_local_[i] = current_car_poses_[i].inverse() * p_init_w;
        }
        poses_initialized_ = true;
        auto p = obj_pose_curr_.translation();
        auto e = get_euler_stable(obj_pose_curr_.linear());
        RCLCPP_INFO(this->get_logger(), "=============================================");
        RCLCPP_INFO(this->get_logger(), "  位姿初始化完成 (OptiTrack 动捕)");
        RCLCPP_INFO(this->get_logger(), "  物体: Pos(%.2f,%.2f,%.2f) Rot(%.2f,%.2f,%.2f)", p.x(), p.y(), p.z(), e[0], e[1], e[2]);
        RCLCPP_INFO(this->get_logger(), "=============================================");
        print_motor_status();
    }

    void print_motor_status() {
        int ready = 0;
        for (int i = 0; i < 3; ++i) if (motor_zeros_captured_[i]) ready++;
        RCLCPP_INFO(this->get_logger(), "滑台: 1:%s 2:%s 3:%s (%d/3)",
            motor_zeros_captured_[0]?"ON":"OFF",
            motor_zeros_captured_[1]?"ON":"OFF",
            motor_zeros_captured_[2]?"ON":"OFF", ready);
    }

    void handle_pose_stamped(int id, const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (msg->pose.position.x <= -180.0) {
            std::string n = (id==0)?"Car1":(id==1)?"Car2":(id==2)?"Car3":"Obj";
            RCLCPP_ERROR(this->get_logger(), "刚体 [%s] 信号丢失!", n.c_str());
            rclcpp::shutdown(); exit(EXIT_FAILURE);
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (id < 3) current_car_poses_[id] = process_pose(msg, id);
        else obj_pose_curr_ = process_pose(msg, id);
        received_poses_[id] = true;
        check_ready_internal();
    }

    // --- 到位检测 ---
    bool wait_for_arrival(const std::vector<Eigen::Vector3d>& target, const std::vector<int>& active, double timeout) {
        auto start = std::chrono::steady_clock::now();
        double last_e = 999.0; int s_cnt = 0;
        while (rclcpp::ok()) {
            double m_e = 0;
            { std::lock_guard<std::mutex> lock(state_mutex_);
              for (int i : active) m_e = std::max(m_e, (current_local_pts_[i]-target[i]).norm()); }
            if (m_e <= POSITION_TOLERANCE) return true;
            if (std::abs(last_e - m_e) < 0.005) s_cnt++; else s_cnt = 0;
            last_e = m_e;
            if (s_cnt >= 12) return true;
            if (std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count() > timeout) return false;
            rclcpp::sleep_for(100ms);
        }
        return false;
    }

    void init_motor_subscriptions() {
        for(int i=0; i<3; ++i) {
            std::string t = "huatai" + std::to_string(i+1) + "_pos_spe_p";
            motor_subs_.push_back(create_subscription<base_interfaces_demo::msg::MotorStatus>(t, 10,
                [this, i](const base_interfaces_demo::msg::MotorStatus::SharedPtr msg) { handle_motor_status(i, msg); }));
        }
    }

    void init_pose_subscriptions() {
        std::vector<std::string> t = {"Rigid17/pose", "Rigid14/pose", "Rigid15/pose", "Rigid8/pose"};
        for(int i=0; i<4; ++i) {
            pose_subs_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(t[i], 10,
                [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { handle_pose_stamped(i, msg); }));
        }
    }

    // --- 终端交互 ---
    void read_user_input() {
        while (rclcpp::ok()) {
            if (!poses_initialized_) {
                // 显示缺失的刚体
                std::string missing;
                for (int i = 0; i < 4; ++i)
                    if (!received_poses_[i]) missing += rigid_names_[i] + " ";
                std::cout << "\r等待动捕: 缺失 [" << missing << "]    " << std::flush;
                publish_status_json("init");
                std::this_thread::sleep_for(1000ms);
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                publish_status_json("idle");
                auto pos = obj_pose_curr_.translation();
                auto e = get_euler_stable(obj_pose_curr_.linear());
                std::cout << "\n--- 物体位姿 [OptiTrack Rigid8] ---" << std::endl;
                std::cout << "Pos: (" << std::fixed << std::setprecision(2)
                          << pos.x() << ", " << pos.y() << ", " << pos.z() << ") mm" << std::endl;
                std::cout << "Rot: (" << e[0] << ", " << e[1] << ", " << e[2] << ") deg" << std::endl;
                std::cout << "滑台 1:" << (motor_zeros_captured_[0]?"ON":"OFF")
                          << " 2:" << (motor_zeros_captured_[1]?"ON":"OFF")
                          << " 3:" << (motor_zeros_captured_[2]?"ON":"OFF")
                          << " (" << (motor_zeros_captured_[0]+motor_zeros_captured_[1]+motor_zeros_captured_[2]) << "/3)" << std::endl;
                std::cout << "> dx dy dz rx ry rz min_t | p=预设 q=退出" << std::endl;
            }

            std::string input; std::getline(std::cin, input);
            if (input == "q") { rclcpp::shutdown(); return; }
            if (input == "p") { execute_preset_commands(); continue; }

            std::istringstream iss(input);
            std::vector<double> vals; double v;
            while (iss >> v) vals.push_back(v);
            if (vals.size() == 7) execute_tiaozi(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6]);
        }
    }

    // --- 成员变量 ---
    rclcpp::Publisher<base_interfaces_demo::msg::MotorCommand>::SharedPtr publisher1_, publisher2_, publisher3_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
    std::vector<rclcpp::Subscription<base_interfaces_demo::msg::MotorStatus>::SharedPtr> motor_subs_;
    std::vector<Eigen::Affine3d> current_car_poses_;
    Eigen::Affine3d obj_pose_curr_;
    std::vector<Eigen::Vector3d> motor_zeros_, grab_points_in_obj_, initial_tips_local_, current_local_pts_;
    std::vector<std::optional<double>> prev_raw_theta_;
    std::vector<double> theta_unwrapped_;
    std::vector<std::string> rigid_names_;
    std::vector<bool> motor_init_flags_, received_poses_, motor_zeros_captured_;
    std::vector<std::vector<double>> preset_commands_;
    std::atomic<bool> poses_initialized_, motors_initialized_;
    std::thread input_thread_;
    std::mutex state_mutex_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HuataiControlNode>());
    rclcpp::shutdown();
    return 0;
}

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <regex>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "base_interfaces_demo/msg/motor_command.hpp"
#include "base_interfaces_demo/msg/motor_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class MathUtils {
public:
    static double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

class HuataiControlNodeTcpModelFixedCars : public rclcpp::Node
{
public:
    HuataiControlNodeTcpModelFixedCars()
    : Node("huatai_control_node_tcp_model_fixed_cars"),
      poses_initialized_(false),
      motors_initialized_(false),
      car_poses_locked_(false)
    {
        publisher1_ = create_publisher<base_interfaces_demo::msg::MotorCommand>("/huatai1_pos_spe_pd", 10);
        publisher2_ = create_publisher<base_interfaces_demo::msg::MotorCommand>("/huatai2_pos_spe_pd", 10);
        publisher3_ = create_publisher<base_interfaces_demo::msg::MotorCommand>("/huatai3_pos_spe_pd", 10);

        current_car_poses_.resize(3, Eigen::Affine3d::Identity());
        initial_car_poses_.resize(3, Eigen::Affine3d::Identity());
        current_local_pts_.resize(3, Eigen::Vector3d::Zero());
        contact_points_in_obj_.resize(3, Eigen::Vector3d::Zero());
        tcp_bias_in_car_.resize(3, Eigen::Vector3d::Zero());

        motor_init_flags_ = {false, false, false};
        received_poses_.resize(4, false);
        theta_unwrapped_.resize(4, 0.0);
        prev_raw_theta_.resize(4, std::nullopt);
        pending_cmd_.resize(8, 0.0);
        pending_cmd_valid_ = false;

        declare_tcp_params();
        load_tcp_params();
        declare_car_pose_source_params();
        load_car_pose_source_params();
        declare_config_persistence_params();
        load_config_persistence_params();
        init_motor_subscriptions();
        init_pose_subscriptions();
        init_preset_commands();

        RCLCPP_INFO(this->get_logger(), "=== 协同调姿控制节点启动 (真实TCP模型-固定小车位姿版) ===");
        print_tcp_params();
        apply_configured_car_poses();
        print_car_pose_sources();
        input_thread_ = std::thread(&HuataiControlNodeTcpModelFixedCars::read_user_input, this);
    }

    ~HuataiControlNodeTcpModelFixedCars() {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    const double POSITION_ERROR_TOLERANCE = 2.0;
    const double ROTATION_ERROR_SUM_TOLERANCE = 0.1;
    const double PREINSERT_POSITION_TOLERANCE = 2.0;
    const double PREINSERT_ROTATION_ERROR_SUM_TOLERANCE = 0.03;
    const double MOTOR_ARRIVAL_TOLERANCE = 0.5;
    const double MOTOR_STABLE_ERROR_LIMIT = 0.8;
    const double COMMAND_SIGNIFICANT_DELTA = 1.0;
    const double MOTOR_NO_MOTION_THRESHOLD = 0.2;
    const double MAX_TRANS_SPEED = 10.0;
    const double MAX_ROT_SPEED = 0.5;
    const double GENERAL_CORRECTION_TIMEOUT = 20.0;
    const double PREINSERT_CORRECTION_TIMEOUT = 30.0;
    const double INSERTION_CORRECTION_TIMEOUT = 20.0;
    const double X_MIN = 1.0, X_MAX = 275.0, Y_MIN = 1.0, Y_MAX = 275.0, Z_MIN = 1.0, Z_MAX = 195.0;

    Eigen::Matrix3d euler_to_matrix(double r, double p, double y) {
        return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())).toRotationMatrix();
    }

    Eigen::Vector3d get_euler_stable(const Eigen::Matrix3d& R) {
        double r, p, y;
        p = -asin(R(2, 0));
        if (cos(p) > 0.001) {
            r = atan2(R(2, 1), R(2, 2));
            y = atan2(R(1, 0), R(0, 0));
        } else {
            r = 0.0;
            y = atan2(-R(0, 1), R(1, 1));
        }
        return Eigen::Vector3d(r * 180.0 / M_PI, p * 180.0 / M_PI, y * 180.0 / M_PI);
    }

    double calculate_move_time(const Eigen::Affine3d& start, const Eigen::Affine3d& end) {
        double d_p = (end.translation() - start.translation()).norm();
        Eigen::Vector3d s_e = get_euler_stable(start.linear());
        Eigen::Vector3d e_e = get_euler_stable(end.linear());
        double d_r = std::max({
            std::abs(MathUtils::normalize_angle((e_e[0] - s_e[0]) * M_PI / 180.0)),
            std::abs(MathUtils::normalize_angle((e_e[1] - s_e[1]) * M_PI / 180.0)),
            std::abs(MathUtils::normalize_angle((e_e[2] - s_e[2]) * M_PI / 180.0))
        }) * 180.0 / M_PI;
        return std::max({d_p / MAX_TRANS_SPEED, d_r / MAX_ROT_SPEED, 2.0});
    }

    Eigen::Affine3d process_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg, size_t idx) {
        double rx_rad = msg->pose.orientation.x * M_PI / 180.0;
        double ry_rad = -msg->pose.orientation.z * M_PI / 180.0;
        double rz_rad = msg->pose.orientation.y * M_PI / 180.0;
        if (!prev_raw_theta_[idx].has_value()) {
            theta_unwrapped_[idx] = rz_rad;
        } else {
            theta_unwrapped_[idx] += MathUtils::normalize_angle(rz_rad - prev_raw_theta_[idx].value());
        }
        prev_raw_theta_[idx] = rz_rad;

        Eigen::Affine3d t = Eigen::Affine3d::Identity();
        t.translate(Eigen::Vector3d(msg->pose.position.x, -msg->pose.position.z, msg->pose.position.y));
        t.linear() = euler_to_matrix(rx_rad, ry_rad, theta_unwrapped_[idx]);
        return t;
    }

    void declare_tcp_params() {
        declare_parameter<std::vector<double>>("tcp_bias_car1", {-143.03, -131.53, 686.69});
        declare_parameter<std::vector<double>>("tcp_bias_car2", {-131.37, -129.56, 674.48});
        declare_parameter<std::vector<double>>("tcp_bias_car3", {-136.64, -139.54, 672.39});
    }

    void declare_car_pose_source_params() {
        declare_parameter<std::string>("car1_pose_source", "mocap");
        declare_parameter<std::string>("car2_pose_source", "mocap");
        declare_parameter<std::string>("car3_pose_source", "mocap");

        declare_parameter<std::vector<double>>("car1_config_position", {0.0, 0.0, 0.0});
        declare_parameter<std::vector<double>>("car2_config_position", {0.0, 0.0, 0.0});
        declare_parameter<std::vector<double>>("car3_config_position", {0.0, 0.0, 0.0});

        declare_parameter<std::vector<double>>("car1_config_rpy_deg", {0.0, 0.0, 0.0});
        declare_parameter<std::vector<double>>("car2_config_rpy_deg", {0.0, 0.0, 0.0});
        declare_parameter<std::vector<double>>("car3_config_rpy_deg", {0.0, 0.0, 0.0});

        declare_parameter<std::string>("obj_pose_source", "mocap");
        declare_parameter<std::vector<double>>("obj_config_position", {0.0, 0.0, 0.0});
        declare_parameter<std::vector<double>>("obj_config_rpy_deg", {0.0, 0.0, 0.0});
    }

    void declare_config_persistence_params() {
        declare_parameter<std::string>(
            "config_yaml_output_path",
            "config/tiaozi_tcp_model_fixed_cars.yaml");
        declare_parameter<std::string>(
            "fallback_sequences_yaml_path",
            "config/tiaozi_tcp_model_fixed_cars_fallbacks.yaml");
        declare_parameter<std::vector<std::string>>(
            "fallback_stage_sequences",
            std::vector<std::string>{
                "242.0 209.6 70.10 222.4 222.1 76.3 254.7 169.0 163.5"
            });
        declare_parameter<double>("fallback_move_time_sec", 6.0);
        declare_parameter<int>("fallback_pause_ms", 500);
    }

    Eigen::Vector3d get_vec3_param(const std::string& name) {
        auto values = get_parameter(name).as_double_array();
        if (values.size() != 3) {
            throw std::runtime_error("Parameter '" + name + "' must have exactly 3 elements.");
        }
        return Eigen::Vector3d(values[0], values[1], values[2]);
    }

    void load_tcp_params() {
        tcp_bias_in_car_[0] = get_vec3_param("tcp_bias_car1");
        tcp_bias_in_car_[1] = get_vec3_param("tcp_bias_car2");
        tcp_bias_in_car_[2] = get_vec3_param("tcp_bias_car3");
    }

    std::string get_pose_source_param(const std::string& name) {
        std::string source = get_parameter(name).as_string();
        if (source != "mocap" && source != "config") {
            throw std::runtime_error("Parameter '" + name + "' must be 'mocap' or 'config'.");
        }
        return source;
    }

    void load_car_pose_source_params() {
        car_pose_sources_[0] = get_pose_source_param("car1_pose_source");
        car_pose_sources_[1] = get_pose_source_param("car2_pose_source");
        car_pose_sources_[2] = get_pose_source_param("car3_pose_source");

        configured_car_positions_[0] = get_vec3_param("car1_config_position");
        configured_car_positions_[1] = get_vec3_param("car2_config_position");
        configured_car_positions_[2] = get_vec3_param("car3_config_position");

        configured_car_rpy_deg_[0] = get_vec3_param("car1_config_rpy_deg");
        configured_car_rpy_deg_[1] = get_vec3_param("car2_config_rpy_deg");
        configured_car_rpy_deg_[2] = get_vec3_param("car3_config_rpy_deg");

        obj_pose_source_ = get_pose_source_param("obj_pose_source");
        configured_obj_position_ = get_vec3_param("obj_config_position");
        configured_obj_rpy_deg_ = get_vec3_param("obj_config_rpy_deg");
    }

    void load_config_persistence_params() {
        config_yaml_output_path_ = resolve_config_path(get_parameter("config_yaml_output_path").as_string());
        fallback_sequences_yaml_path_ = resolve_config_path(get_parameter("fallback_sequences_yaml_path").as_string());
        fallback_move_time_sec_ = get_parameter("fallback_move_time_sec").as_double();
        fallback_pause_ms_ = get_parameter("fallback_pause_ms").as_int();

        fallback_stage_sequences_.clear();
        auto sequence_strings = get_parameter("fallback_stage_sequences").as_string_array();
        for (size_t idx = 0; idx < sequence_strings.size(); ++idx) {
            std::istringstream iss(sequence_strings[idx]);
            std::vector<double> values;
            double v;
            while (iss >> v) {
                values.push_back(v);
            }
            if (values.size() != 9) {
                throw std::runtime_error(
                    "Parameter 'fallback_stage_sequences' entry " + std::to_string(idx) +
                    " must contain exactly 9 numbers.");
            }
            fallback_stage_sequences_.push_back({
                Eigen::Vector3d(values[0], values[1], values[2]),
                Eigen::Vector3d(values[3], values[4], values[5]),
                Eigen::Vector3d(values[6], values[7], values[8]),
            });
        }

        auto yaml_sequences = load_fallback_sequences_from_yaml(fallback_sequences_yaml_path_);
        if (!yaml_sequences.empty()) {
            fallback_stage_sequences_ = yaml_sequences;
            RCLCPP_INFO(
                this->get_logger(),
                "已从结构化回退配置文件加载 %zu 组回退位: %s",
                fallback_stage_sequences_.size(),
                fallback_sequences_yaml_path_.c_str());
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "未从结构化回退配置文件读取到有效回退位，继续使用参数 fallback_stage_sequences。");
        }
    }

    std::string resolve_config_path(const std::string& path) const {
        namespace fs = std::filesystem;
        if (path.empty()) {
            return path;
        }

        fs::path candidate(path);
        if (candidate.is_absolute()) {
            return candidate.string();
        }
        if (fs::exists(candidate)) {
            return candidate.string();
        }

        fs::path package_share = ament_index_cpp::get_package_share_directory("motor_httx_pos_spe");
        if (candidate.parent_path().empty()) {
            return (package_share / "config" / candidate).string();
        }
        return (package_share / candidate).string();
    }

    static std::string trim(const std::string& s) {
        const char* ws = " \t\r\n";
        const auto begin = s.find_first_not_of(ws);
        if (begin == std::string::npos) {
            return "";
        }
        const auto end = s.find_last_not_of(ws);
        return s.substr(begin, end - begin + 1);
    }

    static bool parse_bracket_vec3(const std::string& line, Eigen::Vector3d& out) {
        auto l = line.find('[');
        auto r = line.find(']');
        if (l == std::string::npos || r == std::string::npos || r <= l + 1) {
            return false;
        }
        std::string body = line.substr(l + 1, r - l - 1);
        for (char& c : body) {
            if (c == ',') {
                c = ' ';
            }
        }
        std::istringstream iss(body);
        double x, y, z;
        if (!(iss >> x >> y >> z)) {
            return false;
        }
        out = Eigen::Vector3d(x, y, z);
        return true;
    }

    std::vector<std::vector<Eigen::Vector3d>> load_fallback_sequences_from_yaml(const std::string& path) {
        std::ifstream in(path);
        if (!in.is_open()) {
            RCLCPP_WARN(this->get_logger(), "无法打开结构化回退配置文件: %s", path.c_str());
            return {};
        }

        std::vector<std::vector<Eigen::Vector3d>> sequences;
        std::vector<Eigen::Vector3d> current_group;
        std::string line;
        while (std::getline(in, line)) {
            std::string t = trim(line);
            if (t.empty() || t[0] == '#') {
                continue;
            }

            Eigen::Vector3d vec;
            if (t.rfind("- car1:", 0) == 0) {
                if (!current_group.empty()) {
                    if (current_group.size() == 3) {
                        sequences.push_back(current_group);
                    }
                    current_group.clear();
                }
                if (parse_bracket_vec3(t, vec)) {
                    current_group.push_back(vec);
                }
            } else if (t.rfind("car2:", 0) == 0 || t.rfind("car3:", 0) == 0) {
                if (parse_bracket_vec3(t, vec)) {
                    current_group.push_back(vec);
                    if (current_group.size() == 3) {
                        sequences.push_back(current_group);
                        current_group.clear();
                    }
                }
            }
        }

        if (!current_group.empty() && current_group.size() == 3) {
            sequences.push_back(current_group);
        }
        return sequences;
    }

    void print_tcp_params() {
        for (int i = 0; i < 3; ++i) {
            RCLCPP_INFO(
                this->get_logger(),
                "滑台 %d TCP外参 tcp_bias_in_car = [%.3f, %.3f, %.3f] mm",
                i + 1,
                tcp_bias_in_car_[i].x(),
                tcp_bias_in_car_[i].y(),
                tcp_bias_in_car_[i].z());
        }
    }

    Eigen::Affine3d make_pose_from_config(const Eigen::Vector3d& position, const Eigen::Vector3d& rpy_deg) {
        Eigen::Affine3d t = Eigen::Affine3d::Identity();
        t.translate(position);
        t.linear() = euler_to_matrix(
            rpy_deg.x() * M_PI / 180.0,
            rpy_deg.y() * M_PI / 180.0,
            rpy_deg.z() * M_PI / 180.0);
        return t;
    }

    void apply_configured_car_poses() {
        for (int i = 0; i < 3; ++i) {
            if (car_pose_sources_[i] == "config") {
                current_car_poses_[i] = make_pose_from_config(configured_car_positions_[i], configured_car_rpy_deg_[i]);
                received_poses_[i] = true;
            }
        }
        if (obj_pose_source_ == "config") {
            obj_pose_curr_ = make_pose_from_config(configured_obj_position_, configured_obj_rpy_deg_);
            received_poses_[3] = true;
        }
    }

    void print_car_pose_sources() {
        for (int i = 0; i < 3; ++i) {
            if (car_pose_sources_[i] == "config") {
                RCLCPP_INFO(
                    this->get_logger(),
                    "小车 %d 位姿来源: config, Pos[%.3f, %.3f, %.3f], RPY[%.3f, %.3f, %.3f]",
                    i + 1,
                    configured_car_positions_[i].x(),
                    configured_car_positions_[i].y(),
                    configured_car_positions_[i].z(),
                    configured_car_rpy_deg_[i].x(),
                    configured_car_rpy_deg_[i].y(),
                    configured_car_rpy_deg_[i].z());
            } else {
                RCLCPP_INFO(this->get_logger(), "小车 %d 位姿来源: mocap", i + 1);
            }
        }
        if (obj_pose_source_ == "config") {
            RCLCPP_INFO(
                this->get_logger(),
                "物体 位姿来源: config, Pos[%.3f, %.3f, %.3f], RPY[%.3f, %.3f, %.3f]",
                configured_obj_position_.x(),
                configured_obj_position_.y(),
                configured_obj_position_.z(),
                configured_obj_rpy_deg_.x(),
                configured_obj_rpy_deg_.y(),
                configured_obj_rpy_deg_.z());
        } else {
            RCLCPP_INFO(this->get_logger(), "物体 位姿来源: mocap");
        }
        RCLCPP_INFO(
            this->get_logger(),
            "初始化后如小车位姿来源为 mocap，将自动回写备份到: %s",
            config_yaml_output_path_.c_str());
    }

    void persist_current_config_yaml() {
        std::ofstream out(config_yaml_output_path_, std::ios::trunc);
        if (!out.is_open()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "无法写入配置文件: %s",
                config_yaml_output_path_.c_str());
            return;
        }

        out << "huatai_control_node_tcp_model_fixed_cars:\n";
        out << "  ros__parameters:\n";
        for (int i = 0; i < 3; ++i) {
            out << "    car" << (i + 1) << "_pose_source: \"" << car_pose_sources_[i] << "\"\n";
        }
        out << "\n";
        for (int i = 0; i < 3; ++i) {
            out << "    car" << (i + 1) << "_config_position: ["
                << std::fixed << std::setprecision(6) << configured_car_positions_[i].x() << ", "
                << configured_car_positions_[i].y() << ", "
                << configured_car_positions_[i].z() << "]\n";
            out << "    car" << (i + 1) << "_config_rpy_deg: ["
                << configured_car_rpy_deg_[i].x() << ", "
                << configured_car_rpy_deg_[i].y() << ", "
                << configured_car_rpy_deg_[i].z() << "]\n\n";
        }
        out << "    tcp_bias_car1: ["
            << std::fixed << std::setprecision(2) << tcp_bias_in_car_[0].x() << ", "
            << tcp_bias_in_car_[0].y() << ", "
            << tcp_bias_in_car_[0].z() << "]\n";
        out << "    tcp_bias_car2: ["
            << tcp_bias_in_car_[1].x() << ", "
            << tcp_bias_in_car_[1].y() << ", "
            << tcp_bias_in_car_[1].z() << "]\n";
        out << "    tcp_bias_car3: ["
            << tcp_bias_in_car_[2].x() << ", "
            << tcp_bias_in_car_[2].y() << ", "
            << tcp_bias_in_car_[2].z() << "]\n";
        out.close();

        RCLCPP_INFO(
            this->get_logger(),
            "已将当前小车配置位姿回写到配置文件: %s",
            config_yaml_output_path_.c_str());
    }

    Eigen::Vector3d compute_tcp_in_car(const Eigen::Vector3d& stage_reading, const Eigen::Vector3d& tcp_bias_in_car) {
        return stage_reading + tcp_bias_in_car;
    }

    Eigen::Vector3d compute_stage_target_from_target_pose(
        const Eigen::Affine3d& target_obj_pose,
        const Eigen::Affine3d& car_pose_world,
        const Eigen::Vector3d& contact_point_in_obj,
        const Eigen::Vector3d& tcp_bias_in_car)
    {
        Eigen::Vector3d tcp_target_w = target_obj_pose * contact_point_in_obj;
        Eigen::Vector3d tcp_target_c = car_pose_world.inverse() * tcp_target_w;
        return tcp_target_c - tcp_bias_in_car;
    }

    void init_preset_commands() {
        // 预存装配序列：
        // 1. 先到预备装配位姿：Y/Z/姿态与目标装配位姿一致，仅 X 留出插入余量
        // 2. 再沿 X 轴插入到最终装配位姿bui
        //
        // 当前按 +X 方向插入建模，因此预备装配位姿取 final_x - 50 mm。
        const double final_x = 12099.94;
        const double final_y = -5204.93;
        const double final_z = 1477.50;
        const double final_roll = -1.62;
        const double final_pitch = 3.22;
        const double final_yaw = -2.71;
        const double pre_insert_x_offset = 50.0;

        preset_commands_ = {
            // 绝对模式: [x, y, z, roll, pitch, yaw, min_time, mode]
            {final_x - pre_insert_x_offset, final_y, final_z, final_roll, final_pitch, final_yaw, 8.0, 1.0},
            {final_x, final_y, final_z, final_roll, final_pitch, final_yaw, 6.0, 1.0},
        };
    }

    void execute_preset_commands() {
        if (preset_commands_.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "预存装配序列不足 2 步，无法执行。");
            return;
        }
        RCLCPP_INFO(this->get_logger(), ">>> 开始执行预设装配序列 (共 %zu 条) <<<", preset_commands_.size());

        const auto& preinsert_cmd = preset_commands_[0];
        RCLCPP_INFO(this->get_logger(), "正在执行第 1/%zu 条指令: 预备装配位姿对准...", preset_commands_.size());
        bool preinsert_ok = execute_tiaozi_internal(
            preinsert_cmd[0], preinsert_cmd[1], preinsert_cmd[2],
            preinsert_cmd[3], preinsert_cmd[4], preinsert_cmd[5],
            preinsert_cmd[6], (preinsert_cmd[7] != 0),
            PREINSERT_POSITION_TOLERANCE,
            PREINSERT_ROTATION_ERROR_SUM_TOLERANCE,
            PREINSERT_CORRECTION_TIMEOUT,
            "预备装配");

        if (!preinsert_ok) {
            RCLCPP_ERROR(this->get_logger(), "预备装配位姿未达到姿态精度要求，禁止执行最终 X 插入。");
            execute_preinsert_fallback();
            return;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        const auto& insertion_cmd = preset_commands_[1];
        RCLCPP_INFO(this->get_logger(), "正在执行第 2/%zu 条指令: 沿 X 轴插入...", preset_commands_.size());
        execute_tiaozi_internal(
            insertion_cmd[0], insertion_cmd[1], insertion_cmd[2],
            insertion_cmd[3], insertion_cmd[4], insertion_cmd[5],
            insertion_cmd[6], (insertion_cmd[7] != 0),
            POSITION_ERROR_TOLERANCE,
            ROTATION_ERROR_SUM_TOLERANCE,
            INSERTION_CORRECTION_TIMEOUT,
            "最终插入");

        RCLCPP_INFO(this->get_logger(), ">>> 预设装配序列执行完毕 <<<");
    }

    void execute_preinsert_fallback(bool continuous = false) {
        if (fallback_stage_sequences_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "未配置任何回退位序列。");
            return;
        }

        RCLCPP_WARN(
            this->get_logger(),
            ">>> 启动回退策略：%s执行 %zu 组预定义滑台回退位 <<<",
            continuous ? "连续" : "分步",
            fallback_stage_sequences_.size());

        for (size_t i = 0; i < fallback_stage_sequences_.size(); ++i) {
            const auto& fallback_targets = fallback_stage_sequences_[i];
            std::string tag = "预备装配失败回退-第" + std::to_string(i + 1) + "组";
            bool ok = execute_stage_targets_direct(fallback_targets, fallback_move_time_sec_, tag);
            if (!ok) {
                RCLCPP_ERROR(this->get_logger(), "%s执行失败。", tag.c_str());
                return;
            }
            if (i + 1 < fallback_stage_sequences_.size()) {
                if (continuous) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(fallback_pause_ms_));
                } else {
                    std::cout << "\n按回车继续执行下一组回退位，或按 q 退出回退序列...";
                    std::string input;
                    std::getline(std::cin, input);
                    if (input == "q" || input == "Q") {
                        RCLCPP_WARN(this->get_logger(), "用户中断回退序列执行。");
                        return;
                    }
                }
            }
        }
    }

    bool execute_stage_targets_direct(
        const std::vector<Eigen::Vector3d>& target_m,
        double move_time,
        const std::string& motion_tag) {
        if (target_m.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "%s: 目标滑台数量不是 3。", motion_tag.c_str());
            return false;
        }

        for (int i = 0; i < 3; ++i) {
            if (target_m[i].x() < X_MIN - 0.1 || target_m[i].x() > X_MAX + 0.1 ||
                target_m[i].y() < Y_MIN - 0.1 || target_m[i].y() > Y_MAX + 0.1 ||
                target_m[i].z() < Z_MIN - 0.1 || target_m[i].z() > Z_MAX + 0.1) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "%s: 滑台 %d 回退目标越界 [%.2f, %.2f, %.2f]",
                    motion_tag.c_str(), i + 1, target_m[i].x(), target_m[i].y(), target_m[i].z());
                return false;
            }
        }

        std::vector<Eigen::Vector3d> start_pts(3, Eigen::Vector3d::Zero());
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            start_pts = current_local_pts_;
        }

        RCLCPP_INFO(this->get_logger(), "------------------------------------------------------");
        RCLCPP_INFO(this->get_logger(), ">>> %s：直接下发滑台绝对目标读数 <<<", motion_tag.c_str());
        for (int i = 0; i < 3; ++i) {
            RCLCPP_INFO(
                this->get_logger(),
                "滑台 %d [自身坐标系]: X:%.2f, Y:%.2f, Z:%.2f",
                i + 1, target_m[i].x(), target_m[i].y(), target_m[i].z());
        }
        RCLCPP_INFO(this->get_logger(), "------------------------------------------------------");

        std::vector<rclcpp::Publisher<base_interfaces_demo::msg::MotorCommand>::SharedPtr> pubs = {
            publisher1_, publisher2_, publisher3_
        };
        for (int i = 0; i < 3; ++i) {
            auto msg = base_interfaces_demo::msg::MotorCommand();
            msg.command_type = "position";
            msg.x = target_m[i].x();
            msg.y = target_m[i].y();
            msg.z = target_m[i].z();
            msg.time = move_time;
            msg.is_relative = false;
            pubs[i]->publish(msg);
        }

        bool arrived = wait_for_arrival(target_m, move_time + 1.5);
        warn_if_commanded_but_not_moved(target_m, start_pts, motion_tag);
        Eigen::Vector3d p;
        Eigen::Vector3d e;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            p = obj_pose_curr_.translation();
            e = get_euler_stable(obj_pose_curr_.linear());
        }
        RCLCPP_INFO(this->get_logger(), ">>> %s完成后物体(Rigid8)当前实际位姿 <<<", motion_tag.c_str());
        RCLCPP_INFO(this->get_logger(), "位置 (Pos): X: %.2f, Y: %.2f, Z: %.2f (mm)", p.x(), p.y(), p.z());
        RCLCPP_INFO(this->get_logger(), "姿态 (Deg): Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°", e[0], e[1], e[2]);
        return arrived;
    }

    void execute_tiaozi(double dx, double dy, double dz, double rx, double ry, double rz, double time_min, bool is_absolute) {
        execute_tiaozi_internal(
            dx, dy, dz, rx, ry, rz, time_min, is_absolute,
            POSITION_ERROR_TOLERANCE,
            ROTATION_ERROR_SUM_TOLERANCE,
            GENERAL_CORRECTION_TIMEOUT,
            "常规调姿");
    }

    bool execute_tiaozi_internal(
        double dx, double dy, double dz,
        double rx, double ry, double rz,
        double time_min, bool is_absolute,
        double position_tolerance,
        double rotation_tolerance_sum,
        double correction_timeout_sec,
        const std::string& motion_tag) {
        if (!poses_initialized_ || !motors_initialized_) {
            return false;
        }

        bool accuracy_met = false;
        try {
            Eigen::Affine3d obj_pose_snapshot = Eigen::Affine3d::Identity();
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                obj_pose_snapshot = obj_pose_curr_;
            }

            Eigen::Affine3d T_target = Eigen::Affine3d::Identity();
            if (is_absolute) {
                T_target.translation() << dx, dy, dz;
                T_target.linear() = euler_to_matrix(rx * M_PI / 180.0, ry * M_PI / 180.0, rz * M_PI / 180.0);
            } else {
                T_target.translation() = obj_pose_snapshot.translation() + Eigen::Vector3d(dx, dy, dz);
                T_target.linear() = euler_to_matrix(rx * M_PI / 180.0, ry * M_PI / 180.0, rz * M_PI / 180.0) * obj_pose_snapshot.linear();
            }

            int retries = 0;
            auto correction_start = std::chrono::steady_clock::now();

            do {
                Eigen::Affine3d obj_pose_now = Eigen::Affine3d::Identity();
                std::vector<Eigen::Affine3d> car_poses_now(3, Eigen::Affine3d::Identity());
                std::vector<Eigen::Vector3d> contact_points_now(3, Eigen::Vector3d::Zero());

                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    obj_pose_now = obj_pose_curr_;
                    car_poses_now = initial_car_poses_;
                    contact_points_now = contact_points_in_obj_;
                }

                const double correction_min_time = (retries == 0) ? time_min : 0.0;
                double move_time = std::max(calculate_move_time(obj_pose_now, T_target), correction_min_time);
                std::vector<Eigen::Vector3d> target_m(3, Eigen::Vector3d::Zero());
                for (int i = 0; i < 3; ++i) {
                    target_m[i] = compute_stage_target_from_target_pose(
                        T_target, car_poses_now[i], contact_points_now[i], tcp_bias_in_car_[i]);

                    if (target_m[i].x() < X_MIN - 0.1 || target_m[i].x() > X_MAX + 0.1 ||
                        target_m[i].y() < Y_MIN - 0.1 || target_m[i].y() > Y_MAX + 0.1 ||
                        target_m[i].z() < Z_MIN - 0.1 || target_m[i].z() > Z_MAX + 0.1) {
                        RCLCPP_WARN(
                            this->get_logger(),
                            "❌ 行程拦截：滑台 %d 无法运动到 [%.1f, %.1f, %.1f]",
                            i + 1, target_m[i].x(), target_m[i].y(), target_m[i].z());
                        return false;
                    }
                }

                std::vector<Eigen::Vector3d> start_pts(3, Eigen::Vector3d::Zero());
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    start_pts = current_local_pts_;
                }

                RCLCPP_INFO(this->get_logger(), "------------------------------------------------------");
                if (retries == 0) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        ">>> %s: TCP模型(固定小车位姿)预计算滑台目标读数 (第%d次闭环发送) <<<",
                        motion_tag.c_str(),
                        retries + 1);
                    for (int i = 0; i < 3; ++i) {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "滑台 %d [自身坐标系]: X:%.2f, Y:%.2f, Z:%.2f",
                            i + 1, target_m[i].x(), target_m[i].y(), target_m[i].z());
                    }
                } else {
                    RCLCPP_INFO(
                        this->get_logger(),
                        ">>> %s: 第%d次闭环发送：沿用同一次目标位姿继续纠偏 <<<",
                        motion_tag.c_str(),
                        retries + 1);
                }
                RCLCPP_INFO(this->get_logger(), "------------------------------------------------------");
                RCLCPP_INFO(
                    this->get_logger(),
                    "指令已发送 (%s模式，固定小车位姿TCP模型闭环纠偏，%s)，规划耗时 %.2fs...",
                    is_absolute ? "绝对" : "相对",
                    motion_tag.c_str(),
                    move_time);

                std::vector<rclcpp::Publisher<base_interfaces_demo::msg::MotorCommand>::SharedPtr> pubs = {
                    publisher1_, publisher2_, publisher3_
                };
                for (int i = 0; i < 3; ++i) {
                    auto msg = base_interfaces_demo::msg::MotorCommand();
                    msg.command_type = "position";
                    msg.x = target_m[i].x();
                    msg.y = target_m[i].y();
                    msg.z = target_m[i].z();
                    msg.time = move_time;
                    msg.is_relative = false;
                    pubs[i]->publish(msg);
                }

                wait_for_arrival(target_m, move_time + 1.5);
                warn_if_commanded_but_not_moved(target_m, start_pts, motion_tag);

                std::lock_guard<std::mutex> lock(state_mutex_);
                Eigen::Vector3d pos_err = obj_pose_curr_.translation() - T_target.translation();
                double p_e = pos_err.norm();
                Eigen::Vector3d c_e = get_euler_stable(obj_pose_curr_.linear());
                Eigen::Vector3d t_e = get_euler_stable(T_target.linear());
                Eigen::Vector3d angle_err_deg(
                    std::abs(MathUtils::normalize_angle((c_e[0] - t_e[0]) * M_PI / 180.0)) * 180.0 / M_PI,
                    std::abs(MathUtils::normalize_angle((c_e[1] - t_e[1]) * M_PI / 180.0)) * 180.0 / M_PI,
                    std::abs(MathUtils::normalize_angle((c_e[2] - t_e[2]) * M_PI / 180.0)) * 180.0 / M_PI);
                double r_e_sum = angle_err_deg.sum();

                RCLCPP_INFO(
                    this->get_logger(),
                    "当前闭环误差: |dpos|=%.3f mm, 角度和=%.3f deg, dxyz=[%.3f, %.3f, %.3f], dangle=[%.3f, %.3f, %.3f]",
                    p_e,
                    r_e_sum,
                    pos_err.x(),
                    pos_err.y(),
                    pos_err.z(),
                    angle_err_deg.x(),
                    angle_err_deg.y(),
                    angle_err_deg.z());

                if (p_e < position_tolerance && r_e_sum < rotation_tolerance_sum) {
                    accuracy_met = true;
                    RCLCPP_INFO(this->get_logger(), "闭环纠偏达标，结束发送。");
                } else if (std::chrono::duration<double>(std::chrono::steady_clock::now() - correction_start).count() >= correction_timeout_sec) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "%s超时 %.2fs 仍未达标，停止继续纠偏。",
                        motion_tag.c_str(),
                        correction_timeout_sec);
                    break;
                } else {
                    ++retries;
                }
            } while (!accuracy_met && rclcpp::ok());

            Eigen::Vector3d p;
            Eigen::Vector3d e;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                p = obj_pose_curr_.translation();
                e = get_euler_stable(obj_pose_curr_.linear());
            }
            RCLCPP_INFO(this->get_logger(), "------------------------------------------------------");
            RCLCPP_INFO(this->get_logger(), ">>> 运动完成：物体(Rigid8)当前实际位姿 <<<");
            RCLCPP_INFO(this->get_logger(), "位置 (Pos): X: %.2f, Y: %.2f, Z: %.2f (mm)", p.x(), p.y(), p.z());
            RCLCPP_INFO(this->get_logger(), "姿态 (Deg): Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°", e[0], e[1], e[2]);
            RCLCPP_INFO(this->get_logger(), "------------------------------------------------------");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "异常: %s", e.what());
            return false;
        }
        return accuracy_met;
    }

    void warn_if_commanded_but_not_moved(
        const std::vector<Eigen::Vector3d>& target_m,
        const std::vector<Eigen::Vector3d>& start_pts,
        const std::string& motion_tag) {
        std::vector<Eigen::Vector3d> end_pts(3, Eigen::Vector3d::Zero());
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            end_pts = current_local_pts_;
        }

        for (int i = 0; i < 3; ++i) {
            double commanded_delta = (target_m[i] - start_pts[i]).norm();
            double actual_delta = (end_pts[i] - start_pts[i]).norm();
            if (commanded_delta > COMMAND_SIGNIFICANT_DELTA && actual_delta < MOTOR_NO_MOTION_THRESHOLD) {
                RCLCPP_WARN(
                    this->get_logger(),
                    ">>> 报警 <<< %s 后滑台 %d 目标变化 %.3f mm，但实际仅变化 %.3f mm，小车可能死掉了或未响应。",
                    motion_tag.c_str(),
                    i + 1,
                    commanded_delta,
                    actual_delta);
            }
        }
    }

    void check_ready_internal() {
        if (received_poses_[0] && received_poses_[1] && received_poses_[2] && received_poses_[3]) {
            if (!motors_initialized_ || poses_initialized_) {
                return;
            }

            initial_car_poses_ = current_car_poses_;
            car_poses_locked_ = true;

            for (int i = 0; i < 3; ++i) {
                if (car_pose_sources_[i] == "mocap") {
                    configured_car_positions_[i] = initial_car_poses_[i].translation();
                    configured_car_rpy_deg_[i] = get_euler_stable(initial_car_poses_[i].linear());
                }
            }
            persist_current_config_yaml();

            for (int i = 0; i < 3; ++i) {
                Eigen::Vector3d tcp_init_c = compute_tcp_in_car(current_local_pts_[i], tcp_bias_in_car_[i]);
                Eigen::Vector3d tcp_init_w = initial_car_poses_[i] * tcp_init_c;
                contact_points_in_obj_[i] = obj_pose_curr_.inverse() * tcp_init_w;
            }

            poses_initialized_ = true;
            auto p = obj_pose_curr_.translation();
            auto e = get_euler_stable(obj_pose_curr_.linear());
            RCLCPP_INFO(this->get_logger(), "======================================================");
            RCLCPP_INFO(this->get_logger(), "===    固定小车位姿TCP模型初始化完成: 接触点已锁定   ===");
            RCLCPP_INFO(this->get_logger(), "初始锁定位置: X: %.2f, Y: %.2f, Z: %.2f", p.x(), p.y(), p.z());
            RCLCPP_INFO(this->get_logger(), "初始锁定姿态: R: %.2f, P: %.2f, Y: %.2f", e[0], e[1], e[2]);
            for (int i = 0; i < 3; ++i) {
                auto car_pos = initial_car_poses_[i].translation();
                auto car_euler = get_euler_stable(initial_car_poses_[i].linear());
                RCLCPP_INFO(
                    this->get_logger(),
                    "小车 %d 初始位姿已锁定: Pos[%.3f, %.3f, %.3f], RPY[%.3f, %.3f, %.3f]",
                    i + 1,
                    car_pos.x(), car_pos.y(), car_pos.z(),
                    car_euler.x(), car_euler.y(), car_euler.z());
                RCLCPP_INFO(
                    this->get_logger(),
                    "物体接触点 %d 在物体坐标系下锁定为: [%.3f, %.3f, %.3f]",
                    i + 1,
                    contact_points_in_obj_[i].x(),
                    contact_points_in_obj_[i].y(),
                    contact_points_in_obj_[i].z());
            }
            RCLCPP_INFO(this->get_logger(), "======================================================");
        }
    }

    void handle_pose_stamped(int id, const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (id < 3 && car_pose_sources_[id] == "config") {
            return;
        }
        if (id < 3 && car_poses_locked_) {
            return;
        }
        if (id == 3 && obj_pose_source_ == "config") {
            return;
        }

        if (msg->pose.position.x <= -180.0) {
            std::string n = (id == 0) ? "Car1" : (id == 1) ? "Car2" : (id == 2) ? "Car3" : "Obj";
            RCLCPP_ERROR(this->get_logger(), ">>> 测量致命错误 <<< 刚体 [%s] 信号丢失 (-181)。程序强制退出！", n.c_str());
            rclcpp::shutdown();
            exit(EXIT_FAILURE);
        }

        std::lock_guard<std::mutex> lock(state_mutex_);
        if (id < 3) {
            current_car_poses_[id] = process_pose(msg, id);
        } else {
            obj_pose_curr_ = process_pose(msg, id);
        }
        received_poses_[id] = true;
        check_ready_internal();
    }

    void handle_motor_status(int id, const base_interfaces_demo::msg::MotorStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_local_pts_[id] << msg->x, msg->y, msg->z;
        if (!motor_init_flags_[id]) {
            motor_init_flags_[id] = true;
            if (motor_init_flags_[0] && motor_init_flags_[1] && motor_init_flags_[2]) {
                motors_initialized_ = true;
                check_ready_internal();
            }
        }
    }

    bool wait_for_arrival(const std::vector<Eigen::Vector3d>& target, double timeout) {
        auto start = std::chrono::steady_clock::now();
        double last_e = 999.0;
        int s_cnt = 0;
        while (rclcpp::ok()) {
            double m_e = 0;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                for (int i = 0; i < 3; ++i) {
                    m_e = std::max(m_e, (current_local_pts_[i] - target[i]).norm());
                }
            }
            if (m_e <= MOTOR_ARRIVAL_TOLERANCE) {
                return true;
            }
            if (std::abs(last_e - m_e) < 0.005) {
                s_cnt++;
            } else {
                s_cnt = 0;
            }
            last_e = m_e;
            if (s_cnt >= 12 && m_e <= MOTOR_STABLE_ERROR_LIMIT) {
                return true;
            }
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > timeout) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "等待滑台到位超时，当前最大电机误差 %.3f mm",
                    m_e);
                return false;
            }
            rclcpp::sleep_for(100ms);
        }
        return false;
    }

    void init_motor_subscriptions() {
        for (int i = 0; i < 3; ++i) {
            std::string t = "huatai" + std::to_string(i + 1) + "_pos_spe_p";
            motor_subs_.push_back(
                create_subscription<base_interfaces_demo::msg::MotorStatus>(
                    t,
                    10,
                    [this, i](const base_interfaces_demo::msg::MotorStatus::SharedPtr msg) {
                        handle_motor_status(i, msg);
                    }));
        }
    }

    void init_pose_subscriptions() {
        std::vector<std::string> t = {"Rigid17/pose", "Rigid14/pose", "Rigid15/pose", "Rigid8/pose"};
        for (int i = 0; i < 4; ++i) {
            pose_subs_.push_back(
                create_subscription<geometry_msgs::msg::PoseStamped>(
                    t[i],
                    10,
                    [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                        handle_pose_stamped(i, msg);
                    }));
        }
    }

    bool preview_command(double dx, double dy, double dz, double rx, double ry, double rz, double time_min, bool is_absolute, const std::string& motion_tag) {
        if (!poses_initialized_ || !motors_initialized_) {
            std::cout << "[错误] 系统未初始化完成，无法预览\n";
            return false;
        }

        Eigen::Affine3d obj_pose_now = Eigen::Affine3d::Identity();
        std::vector<Eigen::Affine3d> car_poses_now(3, Eigen::Affine3d::Identity());
        std::vector<Eigen::Vector3d> contact_points_now(3, Eigen::Vector3d::Zero());
        std::vector<Eigen::Vector3d> current_pts(3, Eigen::Vector3d::Zero());
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            obj_pose_now = obj_pose_curr_;
            car_poses_now = initial_car_poses_;
            contact_points_now = contact_points_in_obj_;
            current_pts = current_local_pts_;
        }

        Eigen::Affine3d T_target = Eigen::Affine3d::Identity();
        if (is_absolute) {
            T_target.translation() << dx, dy, dz;
            T_target.linear() = euler_to_matrix(rx * M_PI / 180.0, ry * M_PI / 180.0, rz * M_PI / 180.0);
        } else {
            T_target.translation() = obj_pose_now.translation() + Eigen::Vector3d(dx, dy, dz);
            T_target.linear() = euler_to_matrix(rx * M_PI / 180.0, ry * M_PI / 180.0, rz * M_PI / 180.0) * obj_pose_now.linear();
        }

        double move_time = calculate_move_time(obj_pose_now, T_target);
        move_time = std::max(move_time, time_min);

        std::vector<Eigen::Vector3d> target_m(3, Eigen::Vector3d::Zero());
        bool all_in_range = true;
        std::cout << "\n========== 指令预览 ==========\n";
        std::cout << "目标物体位姿: Pos[%.2f, %.2f, %.2f] RPY[%.2f, %.2f, %.2f]\n";
        printf("目标物体位姿: Pos[%.2f, %.2f, %.2f] RPY[%.2f, %.2f, %.2f]\n",
               dx, dy, dz, rx, ry, rz);
        printf("指令模式: %s, 最小时间: %.2fs\n", is_absolute ? "绝对" : "相对", time_min);
        std::cout << "-----------------------------\n";

        for (int i = 0; i < 3; ++i) {
            target_m[i] = compute_stage_target_from_target_pose(
                T_target, car_poses_now[i], contact_points_now[i], tcp_bias_in_car_[i]);

            double delta_x = target_m[i].x() - current_pts[i].x();
            double delta_y = target_m[i].y() - current_pts[i].y();
            double delta_z = target_m[i].z() - current_pts[i].z();
            double delta_total = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);

            bool in_range = (target_m[i].x() >= X_MIN - 0.1 && target_m[i].x() <= X_MAX + 0.1 &&
                            target_m[i].y() >= Y_MIN - 0.1 && target_m[i].y() <= Y_MAX + 0.1 &&
                            target_m[i].z() >= Z_MIN - 0.1 && target_m[i].z() <= Z_MAX + 0.1);

            printf("滑台 %d: 目标[%.2f, %.2f, %.2f] 当前[%.2f, %.2f, %.2f] 变化[%.2f, %.2f, %.2f] 总变化%.2f %s\n",
                   i + 1,
                   target_m[i].x(), target_m[i].y(), target_m[i].z(),
                   current_pts[i].x(), current_pts[i].y(), current_pts[i].z(),
                   delta_x, delta_y, delta_z, delta_total,
                   in_range ? "" : " [越界!]");

            if (!in_range) all_in_range = false;
        }

        std::cout << "-----------------------------\n";
        printf("预计移动时间: %.2fs\n", move_time);
        std::cout << "==============================\n";
        std::cout << "按回车确认执行，Ctrl+C 取消\n";

        return all_in_range;
    }

    void read_user_input() {
        while (rclcpp::ok()) {
            if (!poses_initialized_) {
                std::this_thread::sleep_for(500ms);
                continue;
            }
            std::cout << "\n输入目标(dx dy dz rx ry rz min_t mode) 或 [p]执行预存 或 [f]分步回退 或 [r]连续回退 或 [q]退出: ";
            std::string input;
            std::getline(std::cin, input);

            if (input == "q") {
                rclcpp::shutdown();
                return;
            }
            if (input == "p") {
                pending_cmd_valid_ = false;
                execute_preset_commands();
                continue;
            }
            if (input == "f" || input == "F") {
                pending_cmd_valid_ = false;
                execute_preinsert_fallback(false);
                continue;
            }
            if (input == "r" || input == "R") {
                pending_cmd_valid_ = false;
                execute_preinsert_fallback(true);
                continue;
            }
            if (input == "c" || input == "C") {
                pending_cmd_valid_ = false;
                std::cout << "[已取消 pending 命令]\n";
                continue;
            }

            std::istringstream iss(input);
            std::vector<double> p;
            double v;
            while (iss >> v) {
                p.push_back(v);
            }
            if (p.size() == 8) {
                if (pending_cmd_valid_) {
                    std::cout << "[正在执行 pending 命令]\n";
                    execute_tiaozi(pending_cmd_[0], pending_cmd_[1], pending_cmd_[2],
                                   pending_cmd_[3], pending_cmd_[4], pending_cmd_[5],
                                   pending_cmd_[6], (pending_cmd_[7] != 0));
                    pending_cmd_valid_ = false;
                } else {
                    pending_cmd_ = p;
                    bool valid = preview_command(p[0], p[1], p[2], p[3], p[4], p[5], p[6], (p[7] != 0), "常规调姿");
                    if (valid) {
                        pending_cmd_valid_ = true;
                    } else {
                        pending_cmd_valid_ = false;
                        std::cout << "[预览失败，请检查滑台行程范围]\n";
                    }
                }
            }
        }
    }

    rclcpp::Publisher<base_interfaces_demo::msg::MotorCommand>::SharedPtr publisher1_, publisher2_, publisher3_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
    std::vector<rclcpp::Subscription<base_interfaces_demo::msg::MotorStatus>::SharedPtr> motor_subs_;
    std::vector<Eigen::Affine3d> current_car_poses_;
    std::vector<Eigen::Affine3d> initial_car_poses_;
    Eigen::Affine3d obj_pose_curr_;
    std::vector<Eigen::Vector3d> current_local_pts_;
    std::vector<Eigen::Vector3d> contact_points_in_obj_;
    std::vector<Eigen::Vector3d> tcp_bias_in_car_;
    std::vector<std::vector<Eigen::Vector3d>> fallback_stage_sequences_;
    double fallback_move_time_sec_ = 6.0;
    int fallback_pause_ms_ = 500;
    std::vector<Eigen::Vector3d> configured_car_positions_ = std::vector<Eigen::Vector3d>(3, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> configured_car_rpy_deg_ = std::vector<Eigen::Vector3d>(3, Eigen::Vector3d::Zero());
    std::vector<std::string> car_pose_sources_ = std::vector<std::string>(3, "mocap");
    std::string obj_pose_source_ = "mocap";
    Eigen::Vector3d configured_obj_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d configured_obj_rpy_deg_ = Eigen::Vector3d::Zero();
    std::string config_yaml_output_path_;
    std::string fallback_sequences_yaml_path_;
    std::vector<std::optional<double>> prev_raw_theta_;
    std::vector<double> theta_unwrapped_;
    std::vector<bool> motor_init_flags_, received_poses_;
    std::vector<std::vector<double>> preset_commands_;
    std::atomic<bool> poses_initialized_, motors_initialized_;
    bool car_poses_locked_;
    std::thread input_thread_;
    std::mutex state_mutex_;
    std::vector<double> pending_cmd_;
    bool pending_cmd_valid_;
    std::string pending_motion_tag_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HuataiControlNodeTcpModelFixedCars>());
    rclcpp::shutdown();
    return 0;
}

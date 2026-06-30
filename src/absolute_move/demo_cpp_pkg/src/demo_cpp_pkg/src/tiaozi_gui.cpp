#include "demo_cpp_pkg/tiaozi_gui.h"
#include "ui_tiaozi_gui.h"

#include <QTimer>
#include <QMetaObject>
#include <QDateTime>
#include <QtConcurrent>
#include <QScrollBar>
#include <fstream>
#include <sstream>
#include <iomanip>

HuataiControlWidget::HuataiControlWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::HuataiControlWidget)
    , poses_initialized_(false)
    , motors_initialized_(false)
    , is_moving_(false)
    , move_expected_time_(0.0)
    , is_pose_stable_(false)
{
    std::cout << "[DEBUG] ctor: setupUi" << std::endl;
    ui->setupUi(this);

    std::cout << "[DEBUG] ctor: init vectors" << std::endl;
    current_car_poses_.resize(3, Eigen::Affine3d::Identity());
    car_raw_pos_.resize(3, Eigen::Vector3d::Zero());
    motor_zeros_.resize(3);
    motor_zeros_captured_.resize(3, false);
    current_local_pts_.resize(3, Eigen::Vector3d::Zero());
    grab_points_in_obj_.resize(3, Eigen::Vector3d::Zero());
    initial_tips_local_.resize(3, Eigen::Vector3d::Zero());
    motor_init_flags_ = {false, false, false};
    received_poses_.resize(4, false);
    theta_unwrapped_.resize(4, 0.0);
    prev_raw_theta_.resize(4, std::nullopt);

    std::cout << "[DEBUG] ctor: timer" << std::endl;
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &HuataiControlWidget::update_display);
    update_timer_->start(100);
    
    std::cout << "[DEBUG] ctor: signal connect" << std::endl;
    connect(ui->spin_pos_id, QOverload<int>::of(&QSpinBox::valueChanged), this, &HuataiControlWidget::handle_pos_id_changed);
    connect(ui->btn_abs_execute, &QPushButton::clicked, this, &HuataiControlWidget::on_btn_abs_execute_clicked);
    connect(ui->btn_rel_execute, &QPushButton::clicked, this, &HuataiControlWidget::on_btn_rel_execute_clicked);
    connect(ui->btn_abs_record, &QPushButton::clicked, this, &HuataiControlWidget::on_btn_abs_record_clicked);
    connect(ui->btn_clear_records, &QPushButton::clicked, this, &HuataiControlWidget::on_btn_clear_records_clicked);
    connect(ui->btn_skip_motor, &QPushButton::clicked, this, &HuataiControlWidget::on_btn_skip_motor_clicked);
    connect(ui->btn_record_pos, &QPushButton::clicked, this, &HuataiControlWidget::on_btn_record_pos_clicked);
    connect(ui->btn_go_to_pos, &QPushButton::clicked, this, &HuataiControlWidget::on_btn_go_to_pos_clicked);
    connect(ui->btn_delete_pos, &QPushButton::clicked, this, &HuataiControlWidget::on_btn_delete_pos_clicked);
    std::cout << "[DEBUG] ctor: button signals connected" << std::endl;

    try {
        std::cout << "[DEBUG] ctor: rclcpp init" << std::endl;
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        node_ = rclcpp::Node::make_shared("huatai_control_gui");

        std::cout << "[DEBUG] ctor: create publishers" << std::endl;
        publisher1_ = node_->create_publisher<base_interfaces_demo::msg::MotorCommand>("/huatai1_pos_spe_pd", 10);
        publisher2_ = node_->create_publisher<base_interfaces_demo::msg::MotorCommand>("/huatai2_pos_spe_pd", 10);
        publisher3_ = node_->create_publisher<base_interfaces_demo::msg::MotorCommand>("/huatai3_pos_spe_pd", 10);

        std::cout << "[DEBUG] ctor: create motor subs" << std::endl;
        for(int i=0; i<3; ++i) {
            std::string t = "huatai" + std::to_string(i+1) + "_pos_spe_p_std";
            std::cout << "[DEBUG] ctor: subscribing to " << t << std::endl;
            motor_subs_.push_back(node_->create_subscription<std_msgs::msg::Float64MultiArray>(
                t, 10, [this, i](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                    try {
                        handle_motor_status(i, msg);
                    } catch (...) {}
                }));
        }

        std::cout << "[DEBUG] ctor: create pose subs" << std::endl;
        std::vector<std::string> t = {"Rigid17/pose", "Rigid14/pose", "Rigid15/pose", "Rigid8/pose"};
        for(int i=0; i<4; ++i) {
            pose_subs_.push_back(node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                t[i], 10, [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    try {
                        handle_pose_stamped(i, msg);
                    } catch (...) {}
                }));
        }

        std::cout << "[DEBUG] ctor: start spin timer" << std::endl;
        spin_timer_ = new QTimer(this);
        connect(spin_timer_, &QTimer::timeout, this, [this]() {
            rclcpp::spin_some(node_);
        });
        spin_timer_->start(20);

        std::cout << "[DEBUG] ctor: post init" << std::endl;
        ui->text_log->append("=== 协同调姿控制器 GUI 启动 ===");
        
        QTimer::singleShot(100, this, &HuataiControlWidget::load_positions_from_file_wrapper);
    } catch (const std::exception& e) {
        std::cerr << "启动异常: " << e.what() << std::endl;
        throw;
    }
    
    std::cout << "[DEBUG] ctor: done" << std::endl;
}

HuataiControlWidget::~HuataiControlWidget()
{
    rclcpp::shutdown();
    delete ui;
}

Eigen::Matrix3d HuataiControlWidget::euler_to_matrix(double r, double p, double y) {
    // ZYX 欧拉角约定：R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

Eigen::Vector3d HuataiControlWidget::get_euler_stable(const Eigen::Matrix3d& R) {
    // ZYX 欧拉角逆分解：返回 (roll, pitch, yaw)
    double r, p, y;
    p = -asin(R(2,0));
    if (cos(p) > 0.001) {
        r = atan2(R(2,1), R(2,2));
        y = atan2(R(1,0), R(0,0));
    } else {
        r = 0;
        y = atan2(-R(0,1), R(1,1));
    }
    return Eigen::Vector3d(r * 180 / M_PI, p * 180 / M_PI, y * 180 / M_PI);
}

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double HuataiControlWidget::calculate_move_time(const Eigen::Affine3d& start, const Eigen::Affine3d& end) {
    double d_p = (end.translation() - start.translation()).norm();
    Eigen::Vector3d s_e = get_euler_stable(start.linear());
    Eigen::Vector3d e_e = get_euler_stable(end.linear());
    double d_r = std::max(std::abs(normalize_angle((e_e[0]-s_e[0])*M_PI/180.0)),
                 std::max(std::abs(normalize_angle((e_e[1]-s_e[1])*M_PI/180.0)),
                          std::abs(normalize_angle((e_e[2]-s_e[2])*M_PI/180.0)))) * 180.0/M_PI;
    return std::max(std::max(d_p / MAX_TRANS_SPEED, d_r / MAX_ROT_SPEED), 2.0);
}

void HuataiControlWidget::handle_motor_status(int id, const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 3) return;
    
    double x = msg->data[0];
    double y = msg->data[1];
    double z = msg->data[2];
    
    bool need_log_zero = false;
    bool need_log_init = false;
    bool need_check_ready = false;
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!motor_zeros_captured_[id]) {
            motor_zeros_[id] << x, y, z;
            motor_zeros_captured_[id] = true;
            need_log_zero = true;
        }
        current_local_pts_[id] << x, y, z;
        if (!motor_init_flags_[id]) {
            motor_init_flags_[id] = true;
            need_log_init = true;
            if (motor_init_flags_[0] && motor_init_flags_[1] && motor_init_flags_[2] &&
                motor_zeros_captured_[0] && motor_zeros_captured_[1] && motor_zeros_captured_[2]) {
                motors_initialized_ = true;
                need_check_ready = true;
            }
        }
    }
    
    if (need_log_zero) {
        add_log(QString(">>> 滑台 %1 初始零点已自动设定为: [%2, %3, %4]").arg(id+1).arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(z, 0, 'f', 2));
    }
    if (need_log_init) {
        add_log(QString(">>> 电机 %1 状态已接收: [%2, %3, %4]").arg(id+1).arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(z, 0, 'f', 2));
    }
    if (need_check_ready) {
        check_ready_internal();
    }
    QMetaObject::invokeMethod(this, "update_init_status", Qt::QueuedConnection);
}

Eigen::Affine3d HuataiControlWidget::process_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg, size_t idx) {
    // 坐标系转换：OptiTrack Y-up → 世界 Z-up
    // 位置：OptiTrack X→世界X, OptiTrack Z→世界-Y, OptiTrack Y→世界Z
    // 轴映射: OptiTrack X→世界X, OptiTrack Z→世界-Y, OptiTrack Y→世界Z
    // 因此: roll(绕世界X)←OptiTrack X, pitch(绕世界Y)←OptiTrack Z的负, yaw(绕世界Z)←OptiTrack Y
    double rx_rad = msg->pose.orientation.x * M_PI / 180.0;
    double ry_rad = -msg->pose.orientation.z * M_PI / 180.0;
    double rz_rad = msg->pose.orientation.y * M_PI / 180.0;
    if (!prev_raw_theta_[idx].has_value()) theta_unwrapped_[idx] = rz_rad;
    else theta_unwrapped_[idx] += normalize_angle(rz_rad - prev_raw_theta_[idx].value());
    prev_raw_theta_[idx] = rz_rad;
    Eigen::Affine3d t = Eigen::Affine3d::Identity();
    t.translate(Eigen::Vector3d(msg->pose.position.x, -msg->pose.position.z, msg->pose.position.y));
    t.linear() = euler_to_matrix(rx_rad, ry_rad, theta_unwrapped_[idx]);
    return t;
}

void HuataiControlWidget::handle_pose_stamped(int id, const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (msg->pose.position.x <= -180.0) {
        std::string n = (id==0)?"Car1":(id==1)?"Car2":(id==2)?"Car3":"Obj";
        add_log(QString(">>> 测量错误 <<< 刚体 [%1] 信号丢失").arg(n.c_str()));
        return;
    }
    
    bool need_log = false;
    bool need_check_ready = false;
    std::string name;
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (id < 3) current_car_poses_[id] = process_pose(msg, id);
        else obj_pose_curr_ = process_pose(msg, id);
        if (!received_poses_[id]) {
            received_poses_[id] = true;
            name = (id==0)?"Car1":(id==1)?"Car2":(id==2)?"Car3":"Obj";
            need_log = true;
            need_check_ready = true;
        }
    }
    
    if (need_log) {
        add_log(QString(">>> 刚体 [%1] 位姿数据已接收").arg(name.c_str()));
    }
    if (need_check_ready) {
        check_ready_internal();
    }
}

void HuataiControlWidget::check_ready_internal() {
    if (received_poses_[0] && received_poses_[1] && received_poses_[2] && received_poses_[3]) {
        if (poses_initialized_) return;
        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3d p_init_w(current_car_poses_[i].translation().x(),
                                     current_car_poses_[i].translation().y(),
                                     obj_pose_curr_.translation().z());
            initial_tips_local_[i] = current_car_poses_[i].inverse() * p_init_w;
            grab_points_in_obj_[i] = obj_pose_curr_.inverse() * p_init_w;
        }
        poses_initialized_ = true;
        auto p = obj_pose_curr_.translation();
        auto e = get_euler_stable(obj_pose_curr_.linear());
        add_log("======================================================");
        add_log("===          物体协同控制初始化完成: 抓取点已锁定      ===");
        add_log(QString("初始锁定位置: X: %1, Y: %2, Z: %3").arg(p.x(), 0, 'f', 2).arg(p.y(), 0, 'f', 2).arg(p.z(), 0, 'f', 2));
        add_log(QString("初始锁定姿态: R: %1, P: %2, Y: %3").arg(e[0], 0, 'f', 2).arg(e[1], 0, 'f', 2).arg(e[2], 0, 'f', 2));
        add_log("======================================================");
        
        is_pose_stable_ = true;
        last_stable_pos_ = p;
        last_stable_euler_ = e;
        
        initial_pos_ = p;
        initial_rot_ = e;
        
        QMetaObject::invokeMethod(this, "update_abs_spins", Qt::QueuedConnection);
        QMetaObject::invokeMethod(this, "update_positions_table", Qt::QueuedConnection);
    }
}

bool HuataiControlWidget::wait_for_arrival(const std::vector<Eigen::Vector3d>& target,
        const std::vector<rclcpp::Publisher<base_interfaces_demo::msg::MotorCommand>::SharedPtr>& pubs,
        double timeout) {
    auto start = std::chrono::steady_clock::now();
    double last_e = 999.0; int s_cnt = 0;
    
    if (!motors_initialized_) {
        add_log(QString(">>> 电机状态未初始化，等待 %1 秒后完成").arg(timeout));
        std::this_thread::sleep_for(std::chrono::seconds((int)timeout));
        add_log(">>> 运动指令已发送");
        return true;
    }
    
    while (rclcpp::ok()) {
        for (size_t i = 0; i < 3 && i < pubs.size(); ++i) {
            base_interfaces_demo::msg::MotorCommand msg;
            msg.command_type = "position";
            msg.x = target[i].x();
            msg.y = target[i].y();
            msg.z = target[i].z();
            msg.time = 5.0;
            msg.is_relative = false;
            pubs[i]->publish(msg);
        }
        
        double max_e = 0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            for (size_t i = 0; i < 3; ++i) {
                double e = (current_local_pts_[i] - target[i]).norm();
                max_e = std::max(max_e, e);
            }
        }
        
        if (max_e < 0.5) {
            s_cnt++;
            if (s_cnt > 10) {
                add_log(">>> 电机到达目标位置");
                return true;
            }
        } else {
            s_cnt = 0;
        }
        
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start).count();
        if (elapsed > timeout) {
            add_log(QString(">>> 超时未到达目标位置 (误差: %1mm)").arg(max_e));
            return false;
        }
        
        if (last_e < 999 && max_e > last_e * 2) {
            add_log(">>> 检测到运动方向异常，停止等待");
            return false;
        }
        last_e = max_e;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
}

void HuataiControlWidget::execute_tiaozi(double dx, double dy, double dz, double rx, double ry, double rz, double time_min, bool is_absolute) {
    if (!poses_initialized_) {
        add_log("警告: 位姿初始化未完成");
        return;
    }

    const double MAX_ANGLE_DEG = 2.0;
    auto clamp = [&](double v) { return std::max(-MAX_ANGLE_DEG, std::min(MAX_ANGLE_DEG, v)); };

    Eigen::Affine3d T_target = Eigen::Affine3d::Identity();
    std::vector<Eigen::Vector3d> target_m(3, Eigen::Vector3d::Zero());
    double move_time;
    Eigen::Vector3d target_pos(dx, dy, dz);
    Eigen::Vector3d target_rot(rx, ry, rz);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (is_absolute) {
            Eigen::Vector3d ce = get_euler_stable(obj_pose_curr_.linear());
            double dr = rx - ce[0];
            double dp = ry - ce[1];
            double droll = rz - ce[2];
            // 限制单次旋转幅度
            if (std::abs(dr) > MAX_ANGLE_DEG || std::abs(dp) > MAX_ANGLE_DEG || std::abs(droll) > MAX_ANGLE_DEG) {
                add_log(QString("警告: 旋转增量过大 dr=%1 dp=%2 dy=%3 (限制±%4)")
                    .arg(dr,0,'f',2).arg(dp,0,'f',2).arg(droll,0,'f',2).arg(MAX_ANGLE_DEG));
                return;
            }
            std::cout << "[DEBUG] abs euler: curr=(" << ce[0] << "," << ce[1] << "," << ce[2] 
                      << ") input=(" << rx << "," << ry << "," << rz 
                      << ") delta=(" << dr << "," << dp << "," << droll << ")" << std::endl;
            // 位置delta + 旋转绝对值（spin框=绝对角度, 增量已在上面计算）
            T_target.translation() = obj_pose_curr_.translation()
                + Eigen::Vector3d(dx - obj_pose_curr_.translation().x(),
                                  dy - obj_pose_curr_.translation().y(),
                                  dz - obj_pose_curr_.translation().z());
            T_target.linear() = euler_to_matrix(rx*M_PI/180.0, ry*M_PI/180.0, rz*M_PI/180.0);
            Eigen::Vector3d te = get_euler_stable(T_target.linear());
            std::cout << "[DEBUG] target euler=(" << te[0] << "," << te[1] << "," << te[2] << ")" << std::endl;
        } else {
            T_target.translation() = obj_pose_curr_.translation() + Eigen::Vector3d(dx, dy, dz);
            // 相对模式：左乘（绕世界轴），与原版 tiaozi.cpp 一致
            T_target.linear() = euler_to_matrix(clamp(rx)*M_PI/180.0, clamp(ry)*M_PI/180.0, clamp(rz)*M_PI/180.0)
                              * obj_pose_curr_.linear();
        }

        move_time = std::max(calculate_move_time(obj_pose_curr_, T_target),
                             (time_min > 0 ? time_min : 3.0));

        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3d tip_car = (current_local_pts_[i] - motor_zeros_[i]) + initial_tips_local_[i];
            Eigen::Vector3d tip_world = current_car_poses_[i] * tip_car;
            Eigen::Vector3d tip_obj = obj_pose_curr_.inverse() * tip_world;
            Eigen::Vector3d tip_target_world = T_target * tip_obj;
            Eigen::Vector3d tip_target_car = current_car_poses_[i].inverse() * tip_target_world;
            target_m[i] = (tip_target_car - initial_tips_local_[i]) + motor_zeros_[i];

            if (target_m[i].x() < X_MIN-0.1 || target_m[i].x() > X_MAX+0.1 ||
                target_m[i].y() < Y_MIN-0.1 || target_m[i].y() > Y_MAX+0.1 ||
                target_m[i].z() < Z_MIN-0.1 || target_m[i].z() > Z_MAX+0.1) {
                add_log(QString("行程拦截: 滑台%1 [%2,%3,%4]")
                    .arg(i+1).arg(target_m[i].x(),0,'f',1).arg(target_m[i].y(),0,'f',1).arg(target_m[i].z(),0,'f',1));
                return;
            }
        }
        is_moving_ = true;
    }

    add_log(QString("目标: d(%1,%2,%3) dR(%4,%5,%6) 耗时%7s")
        .arg(T_target.translation().x() - obj_pose_curr_.translation().x(), 0, 'f', 1)
        .arg(T_target.translation().y() - obj_pose_curr_.translation().y(), 0, 'f', 1)
        .arg(T_target.translation().z() - obj_pose_curr_.translation().z(), 0, 'f', 1)
        .arg(rx, 0, 'f', 2).arg(ry, 0, 'f', 2).arg(rz, 0, 'f', 2)
        .arg(move_time, 0, 'f', 1));
    for (int i = 0; i < 3; ++i)
        add_log(QString("滑台%1: X%2 Y%3 Z%4").arg(i+1).arg(target_m[i].x(),0,'f',2).arg(target_m[i].y(),0,'f',2).arg(target_m[i].z(),0,'f',2));

    QtConcurrent::run([this, target_m, move_time, target_pos, target_rot, is_absolute]() {
        std::vector<std::string> can_ids = {"1", "2", "3"};
        std::vector<rclcpp::Publisher<base_interfaces_demo::msg::MotorCommand>::SharedPtr> pubs = {publisher1_, publisher2_, publisher3_};
        
        for (int i = 0; i < 3; ++i) {
            auto msg = base_interfaces_demo::msg::MotorCommand();
            msg.command_type = "position";
            msg.x = target_m[i].x(); msg.y = target_m[i].y(); msg.z = target_m[i].z();
            msg.time = move_time;
            msg.is_relative = false;
            msg.can_id = can_ids[i];
            if (pubs[i]) {
                pubs[i]->publish(msg);
                std::cout << "[DEBUG] Published to motor " << i+1 
                          << ": x=" << msg.x << ", y=" << msg.y << ", z=" << msg.z 
                          << ", time=" << msg.time << ", can_id=" << msg.can_id << std::endl;
            }
        }
        
        wait_for_arrival(target_m, pubs, move_time + 5.0);
        
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            is_moving_ = false;
        }
        
        QMetaObject::invokeMethod(this, [this, target_pos, target_rot, is_absolute]() {
            add_log(">>> 运动完成");
            
            Eigen::Vector3d actual_pos;
            Eigen::Vector3d actual_rot;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                actual_pos = obj_pose_curr_.translation();
                actual_rot = get_euler_stable(obj_pose_curr_.linear());
            }
            
            double dx = target_pos.x() - actual_pos.x();
            double dy = target_pos.y() - actual_pos.y();
            double dz = target_pos.z() - actual_pos.z();
            double drx = target_rot[0] - actual_rot[0];
            double dry = target_rot[1] - actual_rot[1];
            double drz = target_rot[2] - actual_rot[2];
            
            double pos_error = sqrt(dx*dx + dy*dy + dz*dz);
            double rot_error = sqrt(drx*drx + dry*dry + drz*drz);
            
            add_deviation_record(is_absolute, target_pos, target_rot, actual_pos, actual_rot, pos_error, rot_error);
            
            add_log(QString(">>> 误差记录: 位置误差=%1mm, 旋转误差=%2度")
                .arg(pos_error, 0, 'f', 2).arg(rot_error, 0, 'f', 2));
        }, Qt::QueuedConnection);
    });
}

void HuataiControlWidget::on_btn_rel_execute_clicked() {
    std::cout << "[DEBUG] btn_rel_execute clicked" << std::endl;
    double dx = ui->spin_rel_x->value();
    double dy = ui->spin_rel_y->value();
    double dz = ui->spin_rel_z->value();
    double rx = ui->spin_rel_rx->value();
    double ry = ui->spin_rel_ry->value();
    double rz = ui->spin_rel_rz->value();
    execute_tiaozi(dx, dy, dz, rx, ry, rz, ui->spin_rel_time->value(), false);
}

void HuataiControlWidget::on_btn_abs_execute_clicked() {
    double dx = ui->spin_abs_x->value();
    double dy = ui->spin_abs_y->value();
    double dz = ui->spin_abs_z->value();
    double rx = ui->spin_abs_rx->value();
    double ry = ui->spin_abs_ry->value();
    double rz = ui->spin_abs_rz->value();
    double time_min = ui->spin_abs_time->value();
    add_log(QString(">>> 执行绝对运动: X=%1 Y=%2 Z=%3 R=%4 P=%5 Y=%6 t=%7s")
        .arg(dx,0,'f',2).arg(dy,0,'f',2).arg(dz,0,'f',2)
        .arg(rx,0,'f',2).arg(ry,0,'f',2).arg(rz,0,'f',2).arg(time_min,0,'f',1));
    execute_tiaozi(dx, dy, dz, rx, ry, rz, time_min, true);
}

void HuataiControlWidget::on_btn_abs_record_clicked() {
    std::cout << "[DEBUG] btn_abs_record clicked, poses_initialized_=" << poses_initialized_ << std::endl;
    
    if (!poses_initialized_) {
        add_log(">>> 警告: 初始化未完成，无法记录误差");
        return;
    }
    
    double target_x = ui->spin_abs_x->value();
    double target_y = ui->spin_abs_y->value();
    double target_z = ui->spin_abs_z->value();
    double target_rx = ui->spin_abs_rx->value();
    double target_ry = ui->spin_abs_ry->value();
    double target_rz = ui->spin_abs_rz->value();
    
    Eigen::Vector3d target_pos(target_x, target_y, target_z);
    Eigen::Vector3d target_rot(target_rx, target_ry, target_rz);
    
    Eigen::Vector3d actual_pos;
    Eigen::Vector3d actual_rot;
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        actual_pos = obj_pose_curr_.translation();
        actual_rot = get_euler_stable(obj_pose_curr_.linear());
    }
    
    std::cout << "[DEBUG] record: target=(" << target_x << "," << target_y << "," << target_z 
              << ") actual=(" << actual_pos.x() << "," << actual_pos.y() << "," << actual_pos.z() << ")" << std::endl;
    
    double dx = target_pos.x() - actual_pos.x();
    double dy = target_pos.y() - actual_pos.y();
    double dz = target_pos.z() - actual_pos.z();
    double drx = target_rot[0] - actual_rot[0];
    double dry = target_rot[1] - actual_rot[1];
    double drz = target_rot[2] - actual_rot[2];
    
    double pos_error = sqrt(dx*dx + dy*dy + dz*dz);
    double rot_error = sqrt(drx*drx + dry*dry + drz*drz);
    
    // 直接在主线程添加记录并更新表格
    {
        std::lock_guard<std::mutex> lock(records_mutex_);
        DeviationRecord record;
        record.timestamp = std::chrono::system_clock::now();
        record.target_pos = target_pos;
        record.target_rot = target_rot;
        record.actual_pos = actual_pos;
        record.actual_rot = actual_rot;
        record.pos_error = pos_error;
        record.rot_error = rot_error;
        
        abs_deviation_records_.push_back(record);
        if (abs_deviation_records_.size() > 100) abs_deviation_records_.pop_front();
        
        std::cout << "[DEBUG] record added, total=" << abs_deviation_records_.size() << std::endl;
    }
    
    // 直接更新表格（已在主线程，无需invokeMethod）
    update_deviation_table();
    
    add_log("------------------------------------------------------");
    add_log(">>> 误差记录:");
    add_log(QString("   目标: X=%1 Y=%2 Z=%3").arg(target_x, 0, 'f', 2).arg(target_y, 0, 'f', 2).arg(target_z, 0, 'f', 2));
    add_log(QString("   实际: X=%1 Y=%2 Z=%3").arg(actual_pos.x(), 0, 'f', 2).arg(actual_pos.y(), 0, 'f', 2).arg(actual_pos.z(), 0, 'f', 2));
    add_log(QString("   误差: dx=%1 dy=%2 dz=%3").arg(dx, 0, 'f', 2).arg(dy, 0, 'f', 2).arg(dz, 0, 'f', 2));
    add_log("------------------------------------------------------");
}

void HuataiControlWidget::on_btn_skip_motor_clicked() {
}

void HuataiControlWidget::on_btn_clear_records_clicked() {
    {
        std::lock_guard<std::mutex> lock(records_mutex_);
        abs_deviation_records_.clear();
        rel_deviation_records_.clear();
    }
    update_deviation_table();
    add_log(">>> 误差记录已清空");
}

void HuataiControlWidget::update_display() {
    static int count = 0;
    if (count++ % 50 == 0) {
        std::cout << "[DEBUG] update_display called, count=" << count << std::endl;
    }
    
    std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
        if (count % 50 == 0) std::cout << "[DEBUG] update_display: lock not acquired" << std::endl;
        return;
    }

    if (poses_initialized_) {
        auto p = obj_pose_curr_.translation();
        auto e = get_euler_stable(obj_pose_curr_.linear());

        ui->value_curr_x->setText(QString("%1").arg(p.x(), 0, 'f', 2));
        ui->value_curr_y->setText(QString("%1").arg(p.y(), 0, 'f', 2));
        ui->value_curr_z->setText(QString("%1").arg(p.z(), 0, 'f', 2));
        ui->value_curr_rx->setText(QString("%1").arg(e[0], 0, 'f', 2));
        ui->value_curr_ry->setText(QString("%1").arg(e[1], 0, 'f', 2));
        ui->value_curr_rz->setText(QString("%1").arg(e[2], 0, 'f', 2));
    }
}

void HuataiControlWidget::update_init_status() {
    std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) return;
    
    std::vector<QLabel*> pose_labels = {ui->status_car1, ui->status_car2, ui->status_car3, ui->status_obj};
    
    for (int i = 0; i < 4; ++i) {
        if (received_poses_[i]) {
            pose_labels[i]->setText(QString("%1: 已接收").arg((i==0)?"Car1":(i==1)?"Car2":(i==2)?"Car3":"Obj"));
            pose_labels[i]->setStyleSheet("color: green;");
        } else {
            pose_labels[i]->setText(QString("%1: ---").arg((i==0)?"Car1":(i==1)?"Car2":(i==2)?"Car3":"Obj"));
            pose_labels[i]->setStyleSheet("color: red;");
        }
    }
    
    std::vector<QLabel*> motor_labels = {ui->status_motor1, ui->status_motor2, ui->status_motor3};
    for (int i = 0; i < 3; ++i) {
        if (motor_init_flags_[i]) {
            motor_labels[i]->setText(QString("H%1: 已初始化").arg(i+1));
            motor_labels[i]->setStyleSheet("color: green;");
        } else {
            motor_labels[i]->setText(QString("H%1: ---").arg(i+1));
            motor_labels[i]->setStyleSheet("color: red;");
        }
    }
    
    int count = 0;
    for (bool b : received_poses_) if (b) count++;
    for (bool b : motor_init_flags_) if (b) count++;
    
    ui->label_init_progress->setText(QString("进度: %1/7").arg(count));
    
    if (motors_initialized_ && poses_initialized_) {
        ui->label_init_status->setText("就绪");
        ui->label_init_status->setStyleSheet("color: green; font-weight: bold;");
        ui->label_status->setText("状态: 就绪");
        ui->label_status->setStyleSheet("color: green; font-weight: bold;");
    } else {
        ui->label_init_status->setText("等待数据...");
        ui->label_init_status->setStyleSheet("color: orange; font-weight: bold;");
        ui->label_status->setText("状态: 等待初始化...");
        ui->label_status->setStyleSheet("color: orange; font-weight: bold;");
    }
}

void HuataiControlWidget::update_abs_spins() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (poses_initialized_) {
        auto p = obj_pose_curr_.translation();
        auto e = get_euler_stable(obj_pose_curr_.linear());
        
        ui->spin_abs_x->setValue(p.x());
        ui->spin_abs_y->setValue(p.y());
        ui->spin_abs_z->setValue(p.z());
        ui->spin_abs_rx->setValue(e[0]);
        ui->spin_abs_ry->setValue(e[1]);
        ui->spin_abs_rz->setValue(e[2]);
    }
}

void HuataiControlWidget::add_log(const QString& msg) {
    QMetaObject::invokeMethod(this, [this, msg]() {
        ui->text_log->append(msg);
        ui->text_log->verticalScrollBar()->setValue(ui->text_log->verticalScrollBar()->maximum());
    }, Qt::QueuedConnection);
}

void HuataiControlWidget::add_deviation_record(bool is_absolute, const Eigen::Vector3d& target_pos, const Eigen::Vector3d& target_rot, 
                              const Eigen::Vector3d& actual_pos, const Eigen::Vector3d& actual_rot,
                              double pos_error, double rot_error) {
    {
        std::lock_guard<std::mutex> lock(records_mutex_);
        
        DeviationRecord record;
        record.timestamp = std::chrono::system_clock::now();
        record.target_pos = target_pos;
        record.target_rot = target_rot;
        record.actual_pos = actual_pos;
        record.actual_rot = actual_rot;
        record.pos_error = pos_error;
        record.rot_error = rot_error;
        
        if (is_absolute) {
            abs_deviation_records_.push_back(record);
            if (abs_deviation_records_.size() > 100) abs_deviation_records_.pop_front();
            std::cout << "[DEBUG] add_deviation_record(abs), total=" << abs_deviation_records_.size() << std::endl;
        } else {
            rel_deviation_records_.push_back(record);
            if (rel_deviation_records_.size() > 100) rel_deviation_records_.pop_front();
        }
    }
    
    // 使用lambda方式调用，更可靠
    QMetaObject::invokeMethod(this, [this]() {
        update_deviation_table();
    }, Qt::QueuedConnection);
}

void HuataiControlWidget::update_deviation_table() {
    std::cout << "[DEBUG] update_deviation_table called" << std::endl;
    
    ui->table_deviation->setRowCount(0);
    
    std::lock_guard<std::mutex> lock(records_mutex_);
    
    std::cout << "[DEBUG] update_deviation_table: records=" << abs_deviation_records_.size() << std::endl;
    
    int idx = 1;
    for (const auto& record : abs_deviation_records_) {
        int row = ui->table_deviation->rowCount();
        ui->table_deviation->insertRow(row);
        
        auto time_t = std::chrono::system_clock::to_time_t(record.timestamp);
        auto tm = *std::localtime(&time_t);
        QString time_str = QString("%1:%2:%3")
            .arg(tm.tm_hour, 2, 10, QChar('0'))
            .arg(tm.tm_min, 2, 10, QChar('0'))
            .arg(tm.tm_sec, 2, 10, QChar('0'));
        
        double dx = record.target_pos.x() - record.actual_pos.x();
        double dy = record.target_pos.y() - record.actual_pos.y();
        double dz = record.target_pos.z() - record.actual_pos.z();
        double drx = record.target_rot[0] - record.actual_rot[0];
        double dry = record.target_rot[1] - record.actual_rot[1];
        double drz = record.target_rot[2] - record.actual_rot[2];
        
        ui->table_deviation->setItem(row, 0, new QTableWidgetItem(QString::number(idx++)));
        ui->table_deviation->setItem(row, 1, new QTableWidgetItem(time_str));
        ui->table_deviation->setItem(row, 2, new QTableWidgetItem(QString::number(dx, 'f', 2)));
        ui->table_deviation->setItem(row, 3, new QTableWidgetItem(QString::number(dy, 'f', 2)));
        ui->table_deviation->setItem(row, 4, new QTableWidgetItem(QString::number(dz, 'f', 2)));
        ui->table_deviation->setItem(row, 5, new QTableWidgetItem(QString::number(drx, 'f', 2)));
        ui->table_deviation->setItem(row, 6, new QTableWidgetItem(QString::number(dry, 'f', 2)));
        ui->table_deviation->setItem(row, 7, new QTableWidgetItem(QString::number(drz, 'f', 2)));
        ui->table_deviation->setItem(row, 8, new QTableWidgetItem(QString::number(record.rot_error, 'f', 2)));
    }
    
    std::cout << "[DEBUG] update_deviation_table done, rows=" << ui->table_deviation->rowCount() << std::endl;
}

void HuataiControlWidget::load_positions_from_file() {
    // 调用者必须已持有 position_mutex_
    position_records_.clear();
    
    std::ifstream file(positions_file_);
    if (!file.is_open()) {
        add_log(">>> 位置记录文件不存在");
        return;
    }
    
    int count = 0;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        int id;
        double x, y, z, rx, ry, rz;
        long long timestamp;
        
        if (iss >> id >> x >> y >> z >> rx >> ry >> rz >> timestamp) {
            PositionRecord record;
            record.id = id;
            record.pos = Eigen::Vector3d(x, y, z);
            record.rot = Eigen::Vector3d(rx, ry, rz);
            record.timestamp = std::chrono::system_clock::from_time_t(timestamp);
            position_records_[id] = record;
            count++;
        }
    }
    file.close();
    
    add_log(QString(">>> 已加载 %1 个位置记录").arg(count));
}

void HuataiControlWidget::load_positions_from_file_wrapper() {
    std::lock_guard<std::mutex> lock(position_mutex_);
    load_positions_from_file();
    update_positions_table();
}

void HuataiControlWidget::save_positions_to_file() {
    // 调用者必须已持有 position_mutex_
    
    std::ofstream file(positions_file_);
    if (!file.is_open()) {
        add_log(">>> 警告: 无法保存位置记录文件");
        return;
    }
    
    for (const auto& pair : position_records_) {
        const auto& record = pair.second;
        auto timestamp = std::chrono::system_clock::to_time_t(record.timestamp);
        file << record.id << " "
             << record.pos.x() << " " << record.pos.y() << " " << record.pos.z() << " "
             << record.rot[0] << " " << record.rot[1] << " " << record.rot[2] << " "
             << timestamp << std::endl;
    }
    file.close();
    
    add_log(QString(">>> 已保存 %1 个位置记录").arg(position_records_.size()));
}

void HuataiControlWidget::update_positions_table() {
    // 调用者必须已持有 position_mutex_
    
    ui->table_positions->setRowCount(0);
    
    int current_id = ui->spin_pos_id->value();
    
    int row = ui->table_positions->rowCount();
    ui->table_positions->insertRow(row);
    
    int col = 0;
    ui->table_positions->setItem(row, col++, new QTableWidgetItem(QString::number(current_id)));
    
    auto it = position_records_.find(current_id);
    if (it != position_records_.end()) {
        const auto& record = it->second;
        ui->table_positions->setItem(row, col++, new QTableWidgetItem(QString::number(record.pos.x(), 'f', 2)));
        ui->table_positions->setItem(row, col++, new QTableWidgetItem(QString::number(record.pos.y(), 'f', 2)));
        ui->table_positions->setItem(row, col++, new QTableWidgetItem(QString::number(record.pos.z(), 'f', 2)));
        ui->table_positions->setItem(row, col++, new QTableWidgetItem(QString::number(record.rot[0], 'f', 2)));
        ui->table_positions->setItem(row, col++, new QTableWidgetItem(QString::number(record.rot[1], 'f', 2)));
        ui->table_positions->setItem(row, col++, new QTableWidgetItem(QString::number(record.rot[2], 'f', 2)));
        
        auto time_t = std::chrono::system_clock::to_time_t(record.timestamp);
        auto tm = *std::localtime(&time_t);
        QString time_str = QString("%1:%2:%3")
            .arg(tm.tm_hour, 2, 10, QChar('0'))
            .arg(tm.tm_min, 2, 10, QChar('0'))
            .arg(tm.tm_sec, 2, 10, QChar('0'));
        ui->table_positions->setItem(row, col++, new QTableWidgetItem(time_str));
    } else {
        if (poses_initialized_) {
            ui->table_positions->setItem(row, col++, new QTableWidgetItem(QString::number(initial_pos_.x(), 'f', 2)));
            ui->table_positions->setItem(row, col++, new QTableWidgetItem(QString::number(initial_pos_.y(), 'f', 2)));
            ui->table_positions->setItem(row, col++, new QTableWidgetItem(QString::number(initial_pos_.z(), 'f', 2)));
            ui->table_positions->setItem(row, col++, new QTableWidgetItem(QString::number(initial_rot_[0], 'f', 2)));
            ui->table_positions->setItem(row, col++, new QTableWidgetItem(QString::number(initial_rot_[1], 'f', 2)));
            ui->table_positions->setItem(row, col++, new QTableWidgetItem(QString::number(initial_rot_[2], 'f', 2)));
            ui->table_positions->setItem(row, col++, new QTableWidgetItem("初始值"));
        } else {
            for (int i = 0; i < 6; ++i) {
                ui->table_positions->setItem(row, col++, new QTableWidgetItem("---"));
            }
            ui->table_positions->setItem(row, col++, new QTableWidgetItem("未初始化"));
        }
    }
}

void HuataiControlWidget::on_btn_record_pos_clicked() {
    if (!poses_initialized_) {
        add_log(">>> 警告: 初始化未完成，无法记录位置");
        return;
    }
    
    int pos_id = ui->spin_pos_id->value();
    
    Eigen::Vector3d actual_pos;
    Eigen::Vector3d actual_euler;
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        actual_pos = obj_pose_curr_.translation();
        actual_euler = get_euler_stable(obj_pose_curr_.linear());
    }
    
    {
        std::lock_guard<std::mutex> lock(position_mutex_);
        
        PositionRecord record;
        record.id = pos_id;
        record.pos = actual_pos;
        record.rot = actual_euler;
        record.timestamp = std::chrono::system_clock::now();
        
        position_records_[pos_id] = record;
        
        save_positions_to_file();
        update_positions_table();
    }
    
    add_log(QString(">>> 已记录位置: 编号=%1").arg(pos_id));
}

void HuataiControlWidget::on_btn_go_to_pos_clicked() {
    if (!poses_initialized_) {
        add_log(">>> 警告: 初始化未完成，无法执行定位");
        return;
    }
    
    int pos_id = ui->spin_pos_id->value();
    
    std::lock_guard<std::mutex> lock(position_mutex_);
    auto it = position_records_.find(pos_id);
    
    if (it == position_records_.end()) {
        add_log(QString(">>> 警告: 位置编号 %1 不存在").arg(pos_id));
        return;
    }
    
    const auto& record = it->second;
    
    ui->spin_abs_x->setValue(record.pos.x());
    ui->spin_abs_y->setValue(record.pos.y());
    ui->spin_abs_z->setValue(record.pos.z());
    ui->spin_abs_rx->setValue(record.rot[0]);
    ui->spin_abs_ry->setValue(record.rot[1]);
    ui->spin_abs_rz->setValue(record.rot[2]);
    
    add_log("------------------------------------------------------");
    add_log(QString(">>> 快速定位: 编号=%1").arg(pos_id));
    add_log(QString("   X: %1, Y: %2, Z: %3 (mm)").arg(record.pos.x(), 0, 'f', 2).arg(record.pos.y(), 0, 'f', 2).arg(record.pos.z(), 0, 'f', 2));
    add_log(">>> 目标位置已填入绝对运动输入框，请点击执行");
    add_log("------------------------------------------------------");
}

void HuataiControlWidget::on_btn_delete_pos_clicked() {
    int pos_id = ui->spin_pos_id->value();
    
    {
        std::lock_guard<std::mutex> lock(position_mutex_);
        auto it = position_records_.find(pos_id);
        
        if (it == position_records_.end()) {
            add_log(QString(">>> 警告: 位置编号 %1 不存在").arg(pos_id));
            return;
        }
        
        position_records_.erase(it);
        save_positions_to_file();
        update_positions_table();
    }
    
    add_log(QString(">>> 已删除位置记录: 编号=%1").arg(pos_id));
}

void HuataiControlWidget::handle_pos_id_changed(int) {
    std::lock_guard<std::mutex> lock(position_mutex_);
    update_positions_table();
}
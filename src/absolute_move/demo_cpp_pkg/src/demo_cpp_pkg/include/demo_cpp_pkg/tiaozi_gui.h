#ifndef TIAOZI_GUI_H
#define TIAOZI_GUI_H

#include <QWidget>
#include <QTimer>
#include <QTableWidgetItem>
#include <QString>
#include <mutex>
#include <memory>
#include <vector>
#include <deque>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/motor_command.hpp"
#include "base_interfaces_demo/msg/motor_status.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace Ui {
class HuataiControlWidget;
}

class HuataiControlWidget : public QWidget
{
    Q_OBJECT

public:
    explicit HuataiControlWidget(QWidget *parent = nullptr);
    ~HuataiControlWidget();

private slots:
    void on_btn_rel_execute_clicked();
    void on_btn_abs_execute_clicked();
    void on_btn_abs_record_clicked();
    void on_btn_skip_motor_clicked();
    void on_btn_clear_records_clicked();
    void update_display();
    void update_init_status();
    void update_abs_spins();
    void update_positions_table();
    void on_btn_record_pos_clicked();
    void on_btn_go_to_pos_clicked();
    void on_btn_delete_pos_clicked();
    void handle_pos_id_changed(int value);

private:
    Ui::HuataiControlWidget *ui;

    const double POSITION_TOLERANCE = 1.0;
    const double ROTATION_TOLERANCE = 0.1;
    const double MAX_TRANS_SPEED = 10.0;
    const double MAX_ROT_SPEED = 0.5;
    const double X_MIN = 1.0, X_MAX = 275.0, Y_MIN = 1.0, Y_MAX = 275.0, Z_MIN = 1.0, Z_MAX = 195.0;

    Eigen::Matrix3d euler_to_matrix(double r, double p, double y);
    Eigen::Vector3d get_euler_stable(const Eigen::Matrix3d& R);
    double calculate_move_time(const Eigen::Affine3d& start, const Eigen::Affine3d& end);

    void handle_motor_status(int id, const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    Eigen::Affine3d process_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg, size_t idx);
    void handle_pose_stamped(int id, const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    bool wait_for_arrival(const std::vector<Eigen::Vector3d>& target,
        const std::vector<rclcpp::Publisher<base_interfaces_demo::msg::MotorCommand>::SharedPtr>& pubs,
        double timeout);
    void check_ready_internal();

    void execute_tiaozi(double dx, double dy, double dz, double rx, double ry, double rz, double time_min, bool is_absolute);
    void add_log(const QString& msg);
    void add_deviation_record(bool is_absolute, const Eigen::Vector3d& target_pos, const Eigen::Vector3d& target_rot, 
                              const Eigen::Vector3d& actual_pos, const Eigen::Vector3d& actual_rot,
                              double pos_error, double rot_error);
    void update_deviation_table();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<base_interfaces_demo::msg::MotorCommand>::SharedPtr publisher1_, publisher2_, publisher3_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr> motor_subs_;
    std::vector<Eigen::Affine3d> current_car_poses_;
    Eigen::Affine3d obj_pose_curr_;
    std::vector<Eigen::Vector3d> car_raw_pos_;  // 原始世界坐标 (px, py, pz)
    Eigen::Vector3d obj_raw_pos_;
    std::vector<Eigen::Vector3d> motor_zeros_, grab_points_in_obj_, initial_tips_local_, current_local_pts_;
    std::vector<std::optional<double>> prev_raw_theta_;
    std::vector<double> theta_unwrapped_;
    std::vector<bool> motor_init_flags_, received_poses_;
    std::atomic<bool> poses_initialized_, motors_initialized_;
    std::vector<bool> motor_zeros_captured_;
    std::mutex state_mutex_;

    QTimer* update_timer_;
    QTimer* spin_timer_;

    std::atomic<bool> is_moving_;
    std::chrono::system_clock::time_point move_start_time_;
    double move_expected_time_;
    Eigen::Vector3d move_target_pos_;

    std::atomic<bool> is_pose_stable_;
    Eigen::Vector3d last_stable_pos_;
    Eigen::Vector3d last_stable_euler_;

    struct DeviationRecord {
        std::chrono::system_clock::time_point timestamp;
        Eigen::Vector3d target_pos;
        Eigen::Vector3d target_rot;
        Eigen::Vector3d actual_pos;
        Eigen::Vector3d actual_rot;
        double pos_error;
        double rot_error;
    };
    std::deque<DeviationRecord> rel_deviation_records_;
    std::deque<DeviationRecord> abs_deviation_records_;
    std::mutex records_mutex_;

    struct PositionRecord {
        int id;
        Eigen::Vector3d pos;
        Eigen::Vector3d rot;
        std::chrono::system_clock::time_point timestamp;
    };
    std::map<int, PositionRecord> position_records_;
    std::mutex position_mutex_;
    const std::string positions_file_ = "/home/nkk/huati_absolute_move/positions.dat";
    Eigen::Vector3d initial_pos_;
    Eigen::Vector3d initial_rot_;

    void load_positions_from_file();
    void load_positions_from_file_wrapper();
    void save_positions_to_file();

};
#endif
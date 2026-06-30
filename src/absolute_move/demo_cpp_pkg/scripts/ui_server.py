#!/usr/bin/env python3
"""
华台协同调姿控制 — Web UI 服务器
基于 Flask + rclpy，提供实时位姿显示和滑台控制
"""
import json
import threading
import time
import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from base_interfaces_demo.msg import MotorStatus
from std_msgs.msg import String

from flask import Flask, render_template, request, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

# 全局状态（主线程更新，Flask 线程读取）
state = {
    "obj_x": 0.0, "obj_y": 0.0, "obj_z": 0.0,
    "obj_roll": 0.0, "obj_pitch": 0.0, "obj_yaw": 0.0,
    "motor_ready": [False, False, False],
    "motor_x": [0.0, 0.0, 0.0],
    "motor_y": [0.0, 0.0, 0.0],
    "motor_z": [0.0, 0.0, 0.0],
    "state": "init",
    "connected": False,
    "timestamp": "",
}
state_lock = threading.Lock()

# ROS 2 publisher for commands
cmd_pub = None


class UIStateSubscriber(Node):
    """订阅动捕位姿和滑台状态，提供状态查询"""

    def __init__(self):
        super().__init__("tiaozi_web_ui")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 订阅状态话题
        self.status_sub = self.create_subscription(
            String, "/tiaozi_status", self.status_callback, 10
        )

        # 命令发布器
        global cmd_pub
        cmd_pub = self.create_publisher(String, "/tiaozi_cmd", 10)

        # 备用: 直接订阅位姿和滑台话题
        self.poses_ready = [False, False, False, False]
        self.motors_ready = [False, False, False]

        for i, topic in enumerate(
            ["Rigid17/pose", "Rigid14/pose", "Rigid15/pose", "Rigid8/pose"]
        ):
            self.create_subscription(
                PoseStamped, topic, lambda m, i=i: self.pose_callback(i, m), qos
            )

        for i in range(3):
            topic = f"huatai{i+1}_pos_spe_p"
            self.create_subscription(
                MotorStatus, topic, lambda m, i=i: self.motor_callback(i, m), qos
            )

        self.get_logger().info("Web UI 状态订阅已启动")

    def status_callback(self, msg: String):
        """解析来自 C++ 节点的 JSON 状态"""
        try:
            data = json.loads(msg.data)
            with state_lock:
                for key in [
                    "obj_x", "obj_y", "obj_z",
                    "obj_roll", "obj_pitch", "obj_yaw",
                    "motor_ready", "motor_x", "motor_y", "motor_z",
                    "state",
                ]:
                    if key in data:
                        state[key] = data[key]
                state["connected"] = True
                state["timestamp"] = time.strftime("%H:%M:%S")
        except json.JSONDecodeError:
            pass

    def pose_callback(self, idx, msg: PoseStamped):
        """直接位姿回调 (备用)"""
        with state_lock:
            if idx == 3:  # 物体 Rigid8
                state["obj_x"] = round(msg.pose.position.x, 2)
                state["obj_y"] = round(-msg.pose.position.z, 2)
                state["obj_z"] = round(msg.pose.position.y, 2)
                state["obj_roll"] = round(msg.pose.orientation.x, 2)
                state["obj_pitch"] = round(-msg.pose.orientation.z, 2)
                state["obj_yaw"] = round(msg.pose.orientation.y, 2)
            self.poses_ready[idx] = True
            if all(self.poses_ready):
                state["connected"] = True
                state["timestamp"] = time.strftime("%H:%M:%S")

    def motor_callback(self, idx, msg: MotorStatus):
        """直接滑台状态回调 (备用)"""
        with state_lock:
            state["motor_x"][idx] = round(msg.x, 2)
            state["motor_y"][idx] = round(msg.y, 2)
            state["motor_z"][idx] = round(msg.z, 2)
            if not state["motor_ready"][idx]:
                state["motor_ready"][idx] = True


def ros_spin():
    """ROS 2 主循环 (在单独线程中运行)"""
    rclpy.init(args=sys.argv)
    node = UIStateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ── Flask 路由 ──

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/status")
def api_status():
    with state_lock:
        return jsonify(state)


@app.route("/api/command", methods=["POST"])
def api_command():
    """发送移动指令到 C++ 节点"""
    data = request.get_json()
    if not data:
        return jsonify({"ok": False, "error": "需要 JSON body"}), 400

    # 预设序列
    if data.get("preset"):
        if cmd_pub:
            msg = String()
            msg.data = '{"preset":true}'
            cmd_pub.publish(msg)
        return jsonify({"ok": True, "action": "preset"})

    # 移动指令: {dx, dy, dz, rx, ry, rz, time}
    try:
        dx = float(data.get("dx", 0))
        dy = float(data.get("dy", 0))
        dz = float(data.get("dz", 0))
        rx = float(data.get("rx", 0))
        ry = float(data.get("ry", 0))
        rz = float(data.get("rz", 0))
        t = float(data.get("time", 3.0))
    except (ValueError, TypeError):
        return jsonify({"ok": False, "error": "参数格式错误"}), 400

    cmd = json.dumps(
        {"dx": dx, "dy": dy, "dz": dz, "rx": rx, "ry": ry, "rz": rz, "time": t}
    )
    if cmd_pub:
        msg = String()
        msg.data = cmd
        cmd_pub.publish(msg)

    return jsonify({"ok": True, "cmd": cmd})


# ── 启动 ──

def main():
    # 启动 ROS 2 线程
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # 等待 ROS 2 初始化
    time.sleep(1.0)

    # 启动 Flask
    host = os.environ.get("FLASK_HOST", "0.0.0.0")
    port = int(os.environ.get("FLASK_PORT", "5000"))

    print(f"\n{'='*50}")
    print(f"  华台协同调姿 Web UI")
    print(f"  浏览器打开: http://{host}:{port}")
    print(f"{'='*50}\n")

    app.run(host=host, port=port, debug=False, threaded=True)


if __name__ == "__main__":
    main()

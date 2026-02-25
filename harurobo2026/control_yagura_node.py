import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import numpy as np

import atexit
from sensor_msgs.msg import Joy

#自作ライブラリ
import os
import sys

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from ah_python_can import *
from dyna_lib import *
from auto_robot_interfaces.msg import DynaFeedback, DynaTarget

bus = can.interface.Bus(bustype="socketcan",
                        channel="can0",
                        asynchronous=True,
                        bitrate=1000000)

# ---- Config ----
ring_push_air_id = 0x010  # リング押出機構id
yagura_up_air_id = 0x030  # やぐら昇降機構id
yagura_front_id = 0x031  #  櫓手前id
yagura_mid_id = 0x032  #    櫓中id
yagura_back_id = 0x033  #   櫓奥 id


def update_state(now_button_state, last_button_state, state_counter,
                 state_length):
    "櫓機構のステータス更新"

    if now_button_state == 1 and last_button_state == 0:
        state_counter += 1

    state_counter = state_counter % state_length
    last_button_state = now_button_state

    return state_counter, last_button_state


class YaguraController(Node):

    def __init__(self):
        super().__init__("yagura_controller")

        self.subscription_joy = self.create_subscription(
            Joy,  # メッセージの型
            "/joy",  # 購読するトピック名
            self.joy_callback,  # 呼び出すコールバック関数
            10,
        )  # キューサイズ(溜まっていく)
        self.subscription_joy

        # publisherの設定
        self.dyna_pos_publisher = self.create_publisher(DynaTarget,
                                                        "/dyna_target_pos", 10)
        # air
        send_packet_1byte(0x010, 0, 5, bus)
        send_packet_1byte(0x011, 0, 5, bus)
        send_packet_1byte(0x012, 0, 5, bus)
        send_packet_1byte(0x013, 0, 5, bus)
        send_packet_1byte(0x020, 0, 5, bus)

        self.now_button_state = [0, 0, 0, 0, 0]
        self.last_button_state = [0, 0, 0, 0, 0]
        self.state_counter = [0, 0, 0, 0, 0]

    def publish_dyna_pos(self, id, target):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_pos_publisher.publish(msg)

    def joy_callback(self, msg):
        """joyを受取、各機構を動作

        Args:
            msg (Joy): joy_stick_message
        """

        #受取部分
        self.now_button_state[0] = msg.buttons[0]
        self.now_button_state[1] = msg.buttons[1]
        self.now_button_state[2] = msg.buttons[2]
        self.now_button_state[3] = msg.buttons[3]
        self.now_button_state[4] = msg.buttons[4]

        #ステータス更新
        self.state_counter[0], self.last_button_state[0] = update_state(
            self.now_button_state[0], self.last_button_state[0],
            self.state_counter[0], 2)

        self.state_counter[1], self.last_button_state[1] = update_state(
            self.now_button_state[1], self.last_button_state[1],
            self.state_counter[1], 2)

        self.state_counter[2], self.last_button_state[2] = update_state(
            self.now_button_state[2], self.last_button_state[2],
            self.state_counter[2], 2)

        self.state_counter[3], self.last_button_state[3] = update_state(
            self.now_button_state[3], self.last_button_state[3],
            self.state_counter[3], 2)

        self.state_counter[4], self.last_button_state[4] = update_state(
            self.now_button_state[4], self.last_button_state[4],
            self.state_counter[4], 2)

        #動作部分
        send_packet_1byte(0x010, 12, self.state_counter[0], bus)  # air 閉じる
        send_packet_1byte(0x012, 12, self.state_counter[1], bus)  # air 閉じる
        send_packet_1byte(0x011, 12, self.state_counter[2], bus)  # air 閉じる
        send_packet_1byte(0x013, 12, self.state_counter[3], bus)  # air 閉じる
        send_packet_1byte(0x020, 12, self.state_counter[4], bus)  # air 閉じる


def main():
    rclpy.init()  # rclpyライブラリの初期化

    yagura_controller_node = YaguraController()

    rclpy.spin(yagura_controller_node)  # ノードをスピンさせる
    yagura_controller_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


def stop():
    send_packet_1byte(0x010, 0, 0, bus)
    send_packet_1byte(0x011, 0, 0, bus)
    send_packet_1byte(0x012, 0, 0, bus)
    send_packet_1byte(0x013, 0, 0, bus)
    send_packet_1byte(0x020, 0, 0, bus)


atexit.register(stop)

if __name__ == "__main__":
    main()

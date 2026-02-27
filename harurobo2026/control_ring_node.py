"""リング取得機構
十字キーで昇降
ボタンで回転
ボタンでハンド

昇降は、
一番下
段差超え高さ
櫓高さ
Vゴール高さ
の4種類

ハンドは開く、閉じるの2種類
回転は0度、180度の2種類
"""
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
from dyna_interfaces.msg import DynaFeedback, DynaTarget

#const int ENC_PINNUM_A[4] = {19, 18, 21, 15};  足回りのピン
#const int ENC_PINNUM_B[4] = {23, 17, 16, 2};

# ---- Config ----
ring_dc_motor_id = 0x011  # リング昇降モーターid

bus = can.interface.Bus(bustype="socketcan",
                        channel="can0",
                        asynchronous=True,
                        bitrate=1000000)


def update_state(now_button_state, last_button_state, state_counter,
                 state_length):
    "ステータス更新関数"
    if now_button_state == 1 and last_button_state == 0:
        state_counter += 1

    elif now_button_state == -1 and last_button_state == 0:
        state_counter += -1

    state_counter = (state_counter + state_length) % state_length
    last_button_state = now_button_state

    return state_counter, last_button_state


class RingController(Node):

    def __init__(self):
        super().__init__("ring_controller")

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

        self.now_button_state = [0, 0, 0]  #[昇降、ハンド、回転]
        self.last_button_state = [0, 0, 0]

        self.now_state_counter = [0, 0, 0]
        self.last_state_counter = [0, 0, 0]

        self.now_physwitch_state = 0
        self.last_physwitch_state = 0
        self.init_flag = 1  #初期化flag

        self.timer = self.create_timer(0.01, self.timer_callback)

        #pwm mode
        send_packet_1byte(ring_dc_motor_id, 0, 4, bus)  #停止、内部初期化
        send_packet_4byte(ring_dc_motor_id, 3, -300, bus)  #初期化のため、下方向に降りる

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
        self.now_button_state[0] = msg.axes[7]  #昇降: 十字上下
        self.now_button_state[1] = msg.buttons[5]  #ハンド: R1
        self.now_button_state[2] = msg.buttons[4]  #回転: L1

    def timer_callback(self):

        #昇降機構の初期化処理
        #初期化が終わるまで、他の操作は一切受け付けない
        if (self.now_physwitch_state == 1 and self.last_physwitch_state == 0 and
                self.init_flag == 0):
            self.init_flag = 1

        #初期化が終わっていなければ、returnでガード
        if (self.init_flag == 0):
            return

        # 角度制御モードへの切り替え
        send_packet_1byte(ring_dc_motor_id, 0, 0, bus)  #停止、内部初期化
        send_packet_1byte(ring_dc_motor_id, 0, 1, bus)  #init_encoder_pid
        send_packet_4byte(ring_dc_motor_id, 6, 5, bus)  #set_p_gain

        #ステータス更新
        self.now_state_counter[0], self.last_button_state[0] = update_state(
            self.now_button_state[0], self.last_button_state[0],
            self.now_state_counter[0], 4)

        self.now_state_counter[1], self.last_button_state[1] = update_state(
            self.now_button_state[1], self.last_button_state[1],
            self.now_state_counter[1], 2)

        self.now_state_counter[2], self.last_button_state[2] = update_state(
            self.now_button_state[2], self.last_button_state[2],
            self.now_state_counter[2], 2)

        #動作部分

        #ハンド
        if (self.now_state_counter[1] == 0 and self.last_state_counter[1] == 0):
            self.publish_dyna_pos(4, 2000)  # hidari: 閉じる
            self.publish_dyna_pos(5, 3200)  # 閉じる
            self.last_state_counter[1] = 1

        elif (self.now_state_counter[1] == 1 and
              self.last_state_counter[1] == 1):
            self.publish_dyna_pos(4, 2700)  # 開く
            self.publish_dyna_pos(5, 2500)  # 開く
            self.last_state_counter[1] = 0

        #回転
        if (self.now_state_counter[2] == 0 and self.last_state_counter[2] == 0):
            self.publish_dyna_pos(3, 1050)  # 0度
            self.last_state_counter[2] = 1

        elif (self.now_state_counter[2] == 1 and
              self.last_state_counter[2] == 1):
            self.publish_dyna_pos(3, 3050)  # 180度
            self.last_state_counter[2] = 0

        #昇降
        if (self.now_state_counter[0] == 0):
            send_packet_4byte(ring_dc_motor_id, 1, 0, bus)  # 昇降　１番上 Vゴール
        elif (self.now_state_counter[0] == 3):
            send_packet_4byte(ring_dc_motor_id, 1, -2300, bus)  # 昇降　櫓
        elif (self.now_state_counter[0] == 2):
            send_packet_4byte(ring_dc_motor_id, 1, -3300, bus)  # 昇降　段差超え
        elif (self.now_state_counter[0] == 1):
            send_packet_4byte(ring_dc_motor_id, 1, -4000, bus)  # 昇降　取得

        print("昇降", self.now_state_counter[0])
        print("ハンド", self.now_state_counter[1])
        print("回転", self.now_state_counter[2])


def main():
    rclpy.init()  # rclpyライブラリの初期化

    ring_controller_node = RingController()

    rclpy.spin(ring_controller_node)  # ノードをスピンさせる
    ring_controller_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


def stop():
    send_packet_1byte(ring_dc_motor_id, 0, 0, bus)


atexit.register(stop)

if __name__ == "__main__":
    main()

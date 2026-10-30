#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import csv
import os
import signal
import sys
import matplotlib.pyplot as plt

from degeneracy_detector.msg import Degeneracy


class DegeneracyRecorder:

    def __init__(self):

        # 创建保存目录
        self.save_dir = os.path.expanduser(
            "~/cartographer_ws/degeneracy_data"
        )

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # 使用 ROS 时间生成文件名
        time_str = rospy.Time.now().to_sec()
        self.csv_path = os.path.join(
            self.save_dir,
            "degeneracy_{:.0f}.csv".format(time_str)
        )

        self.csv_file = open(
            self.csv_path,
            "w"
        )

        self.writer = csv.writer(self.csv_file)

        # CSV表头
        self.writer.writerow([
            "time",
            "score",
            "lambda_max",
            "lambda_min",
            "degenerate",
            "direction_x",
            "direction_y",
            "direction_z"
        ])

        self.csv_file.flush()

        # 用于绘图的数据
        self.times = []
        self.scores = []
        self.lambda_max = []
        self.lambda_min = []
        self.degenerate = []

        self.start_time = None

        # 订阅退化检测结果
        self.sub = rospy.Subscriber(
            "/degeneracy",
            Degeneracy,
            self.callback,
            queue_size=100
        )

        rospy.loginfo("==========================================")
        rospy.loginfo("Degeneracy Recorder Started")
        rospy.loginfo("Recording topic: /degeneracy")
        rospy.loginfo("CSV file:")
        rospy.loginfo(self.csv_path)
        rospy.loginfo("Press Ctrl+C to stop and plot.")
        rospy.loginfo("==========================================")

    def callback(self, msg):

        # 使用消息时间
        t = msg.header.stamp.to_sec()

        if self.start_time is None:
            self.start_time = t

        relative_time = t - self.start_time

        # 保存CSV
        self.writer.writerow([
            t,
            msg.score,
            msg.lambda_max,
            msg.lambda_min,
            int(msg.degenerate),
            msg.direction.x,
            msg.direction.y,
            msg.direction.z
        ])

        self.csv_file.flush()

        # 保存绘图数据
        self.times.append(relative_time)
        self.scores.append(msg.score)
        self.lambda_max.append(msg.lambda_max)
        self.lambda_min.append(msg.lambda_min)
        self.degenerate.append(
            1 if msg.degenerate else 0
        )

    def close(self):

        self.csv_file.close()

        rospy.loginfo("Recording finished.")
        rospy.loginfo(
            "Total samples: {}".format(len(self.times))
        )
        rospy.loginfo(
            "CSV saved to: {}".format(self.csv_path)
        )

    def plot(self):

        if len(self.times) == 0:
            rospy.logwarn("No degeneracy data received.")
            return

        # =====================================================
        # Figure 1: Score
        # =====================================================

        plt.figure()

        plt.plot(
            self.times,
            self.scores,
            linewidth=1.2
        )

        plt.xlabel("Time (s)")
        plt.ylabel("Degeneracy Score")

        plt.title("Degeneracy Score")

        plt.grid(True)

        plt.tight_layout()

        score_path = os.path.join(
            self.save_dir,
            "score_curve.png"
        )

        plt.savefig(
            score_path,
            dpi=200
        )

        # =====================================================
        # Figure 2: Eigenvalues
        # =====================================================

        plt.figure()

        plt.plot(
            self.times,
            self.lambda_max,
            label="Lambda Max",
            linewidth=1.2
        )

        plt.plot(
            self.times,
            self.lambda_min,
            label="Lambda Min",
            linewidth=1.2
        )

        plt.xlabel("Time (s)")
        plt.ylabel("Eigenvalue")

        plt.title("Covariance Eigenvalues")

        plt.legend()

        plt.grid(True)

        plt.tight_layout()

        eigen_path = os.path.join(
            self.save_dir,
            "eigenvalue_curve.png"
        )

        plt.savefig(
            eigen_path,
            dpi=200
        )

        # =====================================================
        # Figure 3: Degenerate State
        # =====================================================

        plt.figure()

        plt.step(
            self.times,
            self.degenerate,
            where="post",
            linewidth=1.5
        )

        plt.xlabel("Time (s)")
        plt.ylabel("Degenerate")

        plt.title("Degeneracy State")

        plt.yticks(
            [0, 1],
            ["Normal", "Degenerate"]
        )

        plt.ylim(
            -0.1,
            1.1
        )

        plt.grid(True)

        plt.tight_layout()

        state_path = os.path.join(
            self.save_dir,
            "degeneracy_state.png"
        )

        plt.savefig(
            state_path,
            dpi=200
        )

        rospy.loginfo("==========================================")
        rospy.loginfo("Plots generated:")
        rospy.loginfo(score_path)
        rospy.loginfo(eigen_path)
        rospy.loginfo(state_path)
        rospy.loginfo("==========================================")

        plt.show()


def main():

    rospy.init_node(
        "degeneracy_recorder",
        anonymous=False
    )

    recorder = DegeneracyRecorder()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        pass

    finally:
        recorder.close()
        recorder.plot()


if __name__ == "__main__":
    main()

#!/usr/bin/python3

import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription

from mag_pl_detector.msg._mag_measurements import MagMeasurements


class MagMeasurementSaver(Node):
    def __init__(self, folder):
        super().__init__("mag_measurements_saver")

        self.folder = folder

        self.cnt = 0

        self.sub = self.create_subscription(
            MagMeasurements,
            "/mag_measurements",
            self.callback,
            10
        )

    def callback(self, msg: MagMeasurements):
        filename = os.path.join(
            self.folder,
            str(self.cnt) + ".txt"
        )
        self.cnt += 1

        print(filename)

        with open(filename,"w+") as f:
            for i in range(msg.count):
                for j in range(12):
                    f.write(str(msg.samples[i].time_offset[j]) + "\t")

                for j in range(12):
                    f.write(str(msg.samples[i].data[j]))

                    if (j == 11):
                        f.write("\n")
                    else:
                        f.write("\t")


if __name__ == "__main__":
    rclpy.init()

    filepath = sys.argv[1]

    node = MagMeasurementSaver(filepath)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

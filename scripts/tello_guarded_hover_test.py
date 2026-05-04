#!/usr/bin/env python3
import socket
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tello_msgs.msg import FlightData


RELAY_ADDR = ("192.168.65.254", 18889)
MAX_HEIGHT_CM = 55
HOVER_SECONDS = 5.0


class GuardedHover(Node):
    def __init__(self):
        super().__init__("tello_guarded_hover_test")
        self.flight = None
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(FlightData, "/flight_data", self.flight_cb, 10)

    def flight_cb(self, msg):
        self.flight = msg

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def wait_for_flight_data(self, timeout=8.0):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.flight is not None:
                return self.flight
        return None

    def height_cm(self):
        if self.flight is None:
            return None
        # h is the SDK barometer/height field in cm; tof is range finder in cm.
        return max(int(self.flight.h), int(self.flight.tof))


def send_sdk(cmd, timeout=8.0):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 0))
    sock.settimeout(timeout)
    try:
        sock.sendto(cmd.encode("ascii"), RELAY_ADDR)
        data, addr = sock.recvfrom(1024)
        reply = data.decode("ascii", errors="replace").strip()
        print(f"SDK {cmd!r} -> {reply!r} from {addr}", flush=True)
        return reply
    except socket.timeout:
        print(f"SDK {cmd!r} -> timeout", flush=True)
        return "timeout"
    finally:
        sock.close()


def main():
    rclpy.init()
    node = GuardedHover()
    try:
        flight = node.wait_for_flight_data()
        if flight is None:
            print("ABORT: no /flight_data", flush=True)
            return 2

        print(
            f"PRECHECK bat={flight.bat} temp={flight.templ}-{flight.temph} "
            f"h={flight.h} tof={flight.tof}",
            flush=True,
        )
        if flight.bat < 25:
            print("ABORT: battery below 25%", flush=True)
            return 3
        if flight.temph >= 90:
            print("ABORT: high temperature, cooling required", flush=True)
            return 4

        reply = send_sdk("takeoff", timeout=12.0)
        if "ok" not in reply.lower():
            print("ABORT: takeoff was not accepted", flush=True)
            send_sdk("land", timeout=5.0)
            return 5

        start = time.time()
        landed = False
        while rclpy.ok() and time.time() - start < HOVER_SECONDS:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.publish_zero()
            h = node.height_cm()
            if node.flight is not None:
                print(
                    f"HOVER t={time.time() - start:.1f}s h={node.flight.h} "
                    f"tof={node.flight.tof} bat={node.flight.bat}",
                    flush=True,
                )
            if h is not None and h > MAX_HEIGHT_CM:
                print(f"HEIGHT_GUARD: {h}cm > {MAX_HEIGHT_CM}cm, landing now", flush=True)
                send_sdk("land", timeout=8.0)
                landed = True
                break
            time.sleep(0.4)

        if not landed:
            send_sdk("land", timeout=10.0)

        end = time.time() + 8.0
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.flight is not None:
                print(
                    f"POST h={node.flight.h} tof={node.flight.tof} "
                    f"bat={node.flight.bat}",
                    flush=True,
                )
                if node.flight.h == 0:
                    break
        return 0
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

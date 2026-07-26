#!/usr/bin/env python3
import time
import math
import rclpy
from nav_msgs.msg import Odometry

def main():
    rclpy.init()
    node = rclpy.create_node("drift_monitor")

    samples = []
    def cb(msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        siny_cosp = 2 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1 - 2 * (o.y * o.y + o.z * o.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        samples.append((time.time(), p.x, p.y, yaw, v, w))

    sub = node.create_subscription(Odometry, "/odom", cb, 10)

    print("Sampling stationary /odom for 600 seconds (10 minutes)...", flush=True)
    start = time.time()
    last_print = start
    while time.time() - start < 600:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - last_print >= 60:
            elapsed = time.time() - start
            print(f"Elapsed: {elapsed:.0f}s / 600s | Samples collected: {len(samples)}", flush=True)
            last_print = time.time()

    node.destroy_node()
    rclpy.shutdown()

    if samples:
        t0, x0, y0, yaw0, v0, w0 = samples[0]
        t1, x1, y1, yaw1, v1, w1 = samples[-1]
        xs = [s[1] for s in samples]
        ys = [s[2] for s in samples]
        yaws = [s[3] for s in samples]
        max_dx = max(abs(x - x0) for x in xs)
        max_dy = max(abs(y - y0) for y in ys)
        max_dyaw = max(abs(y - yaw0) for y in yaws)
        duration = t1 - t0
        print(f"=== Stationary Drift Test Complete ({duration:.1f}s, {len(samples)} samples) ===", flush=True)
        print(f"Initial Pose:  X={x0:.6f} m, Y={y0:.6f} m, Yaw={yaw0:.6f} rad", flush=True)
        print(f"Final Pose:    X={x1:.6f} m, Y={y1:.6f} m, Yaw={yaw1:.6f} rad", flush=True)
        print(f"Max Delta X:   {max_dx:.6f} m", flush=True)
        print(f"Max Delta Y:   {max_dy:.6f} m", flush=True)
        print(f"Max Delta Yaw: {max_dyaw:.6f} rad", flush=True)
        print(f"Drift Rate X:  {(max_dx/duration)*1000:.6f} mm/s", flush=True)
        print(f"Drift Rate Y:  {(max_dy/duration)*1000:.6f} mm/s", flush=True)

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import time
import rclpy
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = rclpy.create_node("tf_verifier")
    buf = Buffer()
    lis = TransformListener(buf, node)

    start = time.time()
    laser_ok = False
    imu_ok = False

    while rclpy.ok() and (time.time() - start < 4.0):
        rclpy.spin_once(node, timeout_sec=0.1)

        if not laser_ok and buf.can_transform("base_link", "laser_frame", rclpy.time.Time()):
            t_laser = buf.lookup_transform("base_link", "laser_frame", rclpy.time.Time())
            tr = t_laser.transform.translation
            rot = t_laser.transform.rotation
            print(f"[VERIFIED] base_link -> laser_frame: translation=[{tr.x:.5f}, {tr.y:.5f}, {tr.z:.5f}], rotation=[{rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f}]")
            laser_ok = True

        if not imu_ok and buf.can_transform("base_link", "imu_link", rclpy.time.Time()):
            t_imu = buf.lookup_transform("base_link", "imu_link", rclpy.time.Time())
            tr = t_imu.transform.translation
            rot = t_imu.transform.rotation
            print(f"[VERIFIED] base_link -> imu_link: translation=[{tr.x:.5f}, {tr.y:.5f}, {tr.z:.5f}], rotation=[{rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f}]")
            imu_ok = True

        if laser_ok and imu_ok:
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import unittest
import math


def quaternion_to_rotation_matrix(w, x, y, z):
    """Convert normalized quaternion (w, x, y, z) to 3x3 rotation matrix R(Q)."""
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm > 0:
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
    else:
        w, x, y, z = 1.0, 0.0, 0.0, 0.0

    r11 = 1 - 2 * (y * y + z * z)
    r12 = 2 * (x * y - w * z)
    r13 = 2 * (x * z + w * y)

    r21 = 2 * (x * y + w * z)
    r22 = 1 - 2 * (x * x + z * z)
    r23 = 2 * (y * z - w * x)

    r31 = 2 * (x * z - w * y)
    r32 = 2 * (y * z + w * x)
    r33 = 1 - 2 * (x * x + y * y)

    return [
        [r11, r12, r13],
        [r21, r22, r23],
        [r31, r32, r33]
    ]


def compute_euler_angles(w, x, y, z):
    """Compute Euler Pitch, Roll, and Yaw in degrees from quaternion (w, x, y, z)."""
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm > 0:
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
    else:
        w, x, y, z = 1.0, 0.0, 0.0, 0.0

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll_rad = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch_rad = math.copysign(math.pi / 2, sinp)
    else:
        pitch_rad = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)

    return (
        math.degrees(roll_rad),
        math.degrees(pitch_rad),
        math.degrees(yaw_rad)
    )


def compute_box_matrix(w, x, y, z):
    """
    Compute 3x3 Box Matrix R_box = M_basis * R(Q) * M_basis^T.
    M_basis maps Rover Body Frame (+X forward, +Y left, +Z up) to Viewport Box Frame (+Z FRONT, -X LEFT, -Y TOP).
    """
    R = quaternion_to_rotation_matrix(w, x, y, z)
    r11, r12, r13 = R[0]
    r21, r22, r23 = R[1]
    r31, r32, r33 = R[2]

    # R_box = [
    #   [  r22,  r23, -r21 ],
    #   [  r32,  r33, -r31 ],
    #   [ -r12, -r13,  r11 ]
    # ]
    return [
        [r22, r23, -r21],
        [r32, r33, -r31],
        [-r12, -r13, r11]
    ]


def matrix_determinant_3x3(M):
    """Compute determinant of a 3x3 matrix."""
    a, b, c = M[0]
    d, e, f = M[1]
    g, h, i = M[2]
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


class TestIMU3DTransform(unittest.TestCase):
    """Test suite for live BNO08x quaternion to 3D visualization coordinate transform."""

    def setUp(self):
        # Recorded physical flat rover quaternion sample
        self.q_flat = (0.86322021484375, 0.032470703125, -0.00799560546875, -0.503662109375)

    def test_euler_angle_calculations_for_flat_sample(self):
        """Verify that Euler angles for physical flat sample match true physical values (~3.7° roll, ~1.1° pitch, ~-60.5° yaw)."""
        roll, pitch, yaw = compute_euler_angles(*self.q_flat)
        self.assertAlmostEqual(roll, 3.68, delta=0.2)
        self.assertAlmostEqual(pitch, 1.08, delta=0.2)
        self.assertAlmostEqual(yaw, -60.49, delta=0.2)

    def test_box_matrix_flat_state_is_level(self):
        """Verify that for physical flat sample, the 3D box TOP face normal points >99% straight UP."""
        M_box = compute_box_matrix(*self.q_flat)
        det = matrix_determinant_3x3(M_box)
        self.assertAlmostEqual(det, 1.0, places=4, msg="R_box must be a valid 3D rotation with det=+1.0")

        # Column 2 of M_box represents image of -Y_box (TOP face / UP axis)
        # M_box[1][1] is the Y-component of Column 2 (R33 = 1 - 2(x^2+y^2))
        top_face_up_component = M_box[1][1]
        self.assertGreater(top_face_up_component, 0.99, msg="3D box TOP face must point >99% straight UP when rover is flat")

    def test_nose_up_motion_raises_front_face(self):
        """Verify that when pitch increases (nose up), the FRONT face Y-component tilts upwards on screen."""
        # Quaternion for +15 deg pitch up (w=cos(7.5°), y=sin(7.5°))
        angle_rad = math.radians(15.0)
        qw = math.cos(angle_rad / 2)
        qy = math.sin(angle_rad / 2)
        M_box = compute_box_matrix(qw, 0.0, qy, 0.0)

        # Column 3 of M_box is FRONT face (+Z_box) direction
        # M_box[1][2] is -r31 = -2(xz - wy) = +2*wy for pure pitch
        front_face_y = M_box[1][2]
        self.assertGreater(front_face_y, 0.2, msg="Nose up motion must visibly raise FRONT face on screen")

    def test_left_side_up_motion_raises_left_face(self):
        """Verify that when roll increases (left side up), the LEFT face Y-component tilts upwards on screen."""
        # Quaternion for +15 deg roll left up (w=cos(7.5°), x=sin(7.5°))
        angle_rad = math.radians(15.0)
        qw = math.cos(angle_rad / 2)
        qx = math.sin(angle_rad / 2)
        M_box = compute_box_matrix(qw, qx, 0.0, 0.0)

        # Column 1 of M_box is X_box direction (image of -Y_rover = LEFT)
        # M_box[1][0] is r32 = +2*wx for pure roll
        left_face_y = M_box[1][0]
        self.assertGreater(left_face_y, 0.2, msg="Left side up motion must visibly raise LEFT face on screen")

    def test_yaw_rotation_preserves_level_up_axis(self):
        """Verify that rotating yaw around Z_rover axis spins in plane without altering TOP face UP alignment."""
        # Test various yaw angles
        for deg in [-90, -45, 0, 45, 90, 180]:
            angle_rad = math.radians(deg)
            qw = math.cos(angle_rad / 2)
            qz = math.sin(angle_rad / 2)
            M_box = compute_box_matrix(qw, 0.0, 0.0, qz)

            # TOP face UP component M_box[1][1] must remain exactly 1.0 for pure yaw
            self.assertAlmostEqual(M_box[1][1], 1.0, places=4, msg=f"Pure yaw at {deg}° must keep TOP face 100% UP")


if __name__ == '__main__':
    unittest.main()

import math

#!/usr/bin/env python


def q2euler(q_w, q_x, q_y, q_z):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    # Roll (x-axis rotation)
    sr_cp = 2.0 * (q_w * q_x + q_y * q_z)
    cr_cp = 1.0 - 2.0 * (q_x * q_x + q_y * q_y)
    roll = math.atan2(sr_cp, cr_cp)

    # Pitch (y-axis rotation)
    sin_p = 2.0 * (q_w * q_y - q_z * q_x)
    if abs(sin_p) >= 1:
        pitch = math.copysign(math.pi / 2, sin_p)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sin_p)

    # Yaw (z-axis rotation)
    sy_cp = 2.0 * (q_w * q_z + q_x * q_y)
    cy_cp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z)
    yaw = math.atan2(sy_cp, cy_cp)

    return roll, pitch, yaw

if __name__ == '__main__':
    try:
        q_w = float(input("Enter quaternion w: "))
        q_x = float(input("Enter quaternion x: "))
        q_y = float(input("Enter quaternion y: "))
        q_z = float(input("Enter quaternion z: "))

        roll_rad, pitch_rad, yaw_rad = q2euler(q_w, q_x, q_y, q_z)

        # Convert radians to degrees for readability
        roll_deg = math.degrees(roll_rad)
        pitch_deg = math.degrees(pitch_rad)
        yaw_deg = math.degrees(yaw_rad)

        print("\n--- Euler Angles ---")
        print(f"Roll:  {roll_rad:.4f} radians, {roll_deg:.4f} degrees")
        print(f"Pitch: {pitch_rad:.4f} radians, {pitch_deg:.4f} degrees")
        print(f"Yaw:   {yaw_rad:.4f} radians, {yaw_deg:.4f} degrees")

    except ValueError:
        print("Invalid input. Please enter numeric values.")
    except Exception as e:
        print(f"An error occurred: {e}")
import math

def yaw_from_quaternion( w, x, y, z):
    # Quaternion to yaw (rotation around Z axis). Result is in [-pi, pi].
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
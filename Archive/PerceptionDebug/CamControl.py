import subprocess
import sys
import tty
import termios
import math

x, y, z = 1.51, 2.0, 1.82
yaw = 0.34

STEP = 0.05
YAW_STEP = 0.05

def set_pose(x, y, z, yaw):
    # Convert yaw to quaternion (rotation around Z axis)
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    req = (
        f'name: "animated_rgbd_camera" '
        f'position: {{x: {x:.3f} y: {y:.3f} z: {z:.3f}}} '
        f'orientation: {{z: {qz:.6f} w: {qw:.6f}}}'
    )
    subprocess.run([
        'gz', 'service',
        '--service', '/world/lidar_sensor/set_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', req
    ], capture_output=True)

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

print("Camera keyboard controller")
print("W/S: forward/back | A/D: strafe left/right | Q/E: up/down | Z/X: rotate | Ctrl+C: quit")

set_pose(x, y, z, yaw)

while True:
    k = get_key()
    if k == '\x03':
        break

    dx, dy = 0.0, 0.0

    if k == 'w':
        dx =  STEP
    elif k == 's':
        dx = -STEP
    elif k == 'a':
        dy =  STEP
    elif k == 'd':
        dy = -STEP
    elif k == 'q':
        z += STEP
    elif k == 'e':
        z -= STEP
    elif k == 'z':
        yaw += YAW_STEP
    elif k == 'x':
        yaw -= YAW_STEP
    else:
        continue

    # Rotate movement vector by current yaw
    x += dx * math.cos(yaw) - dy * math.sin(yaw)
    y += dx * math.sin(yaw) + dy * math.cos(yaw)

    print(f"x={x:.2f} y={y:.2f} z={z:.2f} yaw={yaw:.2f}")
    set_pose(x, y, z, yaw)
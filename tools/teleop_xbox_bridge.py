"""Xbox 360 teleop via Windows host -> rosbridge -> /cmd_vel (RUN ON WINDOWS).

Why: WSL2 stock kernel has no joystick drivers, so /dev/input/js0 never
appears even when usbipd-win forwards USB. Read the pad natively on Windows
(pygame/XInput) and publish geometry_msgs Twist into ROS2 via rosbridge 9090
(already exposed by gcs.launch.py).

Windows PowerShell setup:
  1. Keep pad on Windows (do NOT attach to WSL2):
     usbipd detach --busid 2-3 ; usbipd unbind --busid 2-3
  2. pip install pygame roslibpy
  3. Start rosbridge in WSL2 (below), then: python teleop_xbox_bridge.py

WSL2 (one terminal):
  source /opt/ros/jazzy/setup.bash
  source ~/projects/autonomous_rover/ros2_ws/install/setup.bash
  ros2 launch rover_slam gcs.launch.py pi_host:=192.168.0.12
  # minimal: ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 address:=0.0.0.0

Controls (pygame indices): X(btn2) enable, A(btn0) enable-alt, RB(btn5)
turbo, B(btn1)/Back(btn6) E-STOP latch, Start(btn7) clear, ax1 linear,
ax3 angular. Publishes 20 Hz; rover_bridge zeroes cmd after 0.5 s idle.
"""
from __future__ import annotations

import argparse
import sys
import time

ROSBRIDGE_HOST = "127.0.0.1"
ROSBRIDGE_PORT = 9090
TOPIC_NAME = "/cmd_vel"

AXIS_LINEAR_X = 1
AXIS_ANGULAR_Z = 3
BUTTON_ENABLE = 2
BUTTON_ENABLE_ALT = 0
BUTTON_TURBO = 5
BUTTON_ESTOP = 1
BUTTON_ESTOP_ALT = 6
BUTTON_CLEAR = 7

SCALE_LINEAR_NORMAL = 0.3
SCALE_LINEAR_TURBO = 0.6
SCALE_ANGULAR = 1.0
PUBLISH_RATE_HZ = 20
DEADZONE = 0.1


def apply_deadzone(val: float, deadzone: float = DEADZONE) -> float:
    return 0.0 if abs(val) < deadzone else val


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Xbox teleop via rosbridge")
    p.add_argument("--host", default=ROSBRIDGE_HOST)
    p.add_argument("--port", type=int, default=ROSBRIDGE_PORT)
    p.add_argument("--topic", default=TOPIC_NAME)
    p.add_argument("--calibrate", action="store_true")
    p.add_argument("--linear-axis", type=int, default=AXIS_LINEAR_X)
    p.add_argument("--angular-axis", type=int, default=AXIS_ANGULAR_Z)
    return p.parse_args(argv)


def print_calibrate(joy, pygame) -> None:
    print(f"[+] Calibrating: {joy.get_name()} "
          f"({joy.get_numaxes()} axes, {joy.get_numbuttons()} buttons)")
    print("    Move sticks / press buttons. Ctrl+C to exit.")
    try:
        while True:
            pygame.event.pump()
            axes = [f"{i}:{joy.get_axis(i):+.2f}" for i in range(joy.get_numaxes())]
            pressed = [str(i) for i in range(joy.get_numbuttons()) if joy.get_button(i)]
            print(f"axes [{' '.join(axes)}] btns: "
                  f"{','.join(pressed) if pressed else '-'}", end="\r")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nDone.")
    finally:
        pygame.quit()


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        import pygame
    except ImportError:
        print("[!] pip install pygame roslibpy")
        return 1
    try:
        import roslibpy
    except ImportError:
        print("[!] pip install pygame roslibpy")
        return 1
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("[!] No joystick on Windows. Connect pad, keep it on Windows.")
        pygame.quit()
        return 1
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"[+] Joystick: {joy.get_name()}")
    if args.calibrate:
        print_calibrate(joy, pygame)
        return 0
    client = roslibpy.Ros(host=args.host, port=args.port)
    print(f"[...] Connecting ws://{args.host}:{args.port} ...")
    try:
        client.run()
    except Exception as exc:
        print(f"[!] rosbridge failed: {exc}")
        print("    WSL2: ros2 launch rover_slam gcs.launch.py pi_host:=192.168.0.12")
        pygame.quit()
        return 1
    deadline = time.time() + 5.0
    while not client.is_connected and time.time() < deadline:
        time.sleep(0.1)
    if not client.is_connected:
        print("[!] handshake timeout; is 127.0.0.1:9090 listening?")
        try:
            client.terminate()
        finally:
            pygame.quit()
        return 1
    topic = roslibpy.Topic(client, args.topic, "geometry_msgs/msg/Twist")
    print(f"[+] Publishing {args.topic} 20Hz. Hold X to drive, RB turbo.")
    interval = 1.0 / PUBLISH_RATE_HZ
    estop_latched = False
    prev_estop = False
    prev_clear = False

    def zero():
        topic.publish(roslibpy.Message(
            {"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
             "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}))

    try:
        while client.is_connected:
            start = time.time()
            pygame.event.pump()
            estop_now = bool(joy.get_button(BUTTON_ESTOP)
                             or (joy.get_numbuttons() > BUTTON_ESTOP_ALT
                                 and joy.get_button(BUTTON_ESTOP_ALT)))
            clear_now = bool(joy.get_numbuttons() > BUTTON_CLEAR
                             and joy.get_button(BUTTON_CLEAR))
            if estop_now and not prev_estop and not estop_latched:
                print("[!] E-STOP latched (B). Start clears.")
                estop_latched = True
            if clear_now and not prev_clear and estop_latched:
                estop_latched = False
                print("[+] E-STOP cleared.")
            prev_estop, prev_clear = estop_now, clear_now
            enabled = bool(joy.get_button(BUTTON_ENABLE)
                           or joy.get_button(BUTTON_ENABLE_ALT))
            turbo = bool(joy.get_numbuttons() > BUTTON_TURBO
                         and joy.get_button(BUTTON_TURBO))
            linear_x = 0.0
            angular_z = 0.0
            if enabled and not estop_latched:
                raw_lin = -joy.get_axis(args.linear_axis)
                raw_ang = -joy.get_axis(args.angular_axis)
                scale = SCALE_LINEAR_TURBO if turbo else SCALE_LINEAR_NORMAL
                linear_x = apply_deadzone(raw_lin) * scale
                angular_z = apply_deadzone(raw_ang) * SCALE_ANGULAR
            topic.publish(roslibpy.Message(
                {"linear": {"x": float(linear_x), "y": 0.0, "z": 0.0},
                 "angular": {"x": 0.0, "y": 0.0, "z": float(angular_z)}}))
            time.sleep(max(0.0, interval - (time.time() - start)))
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            if client.is_connected:
                zero()
        finally:
            try:
                topic.unadvertise()
            except Exception:
                pass
            client.terminate()
            pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())

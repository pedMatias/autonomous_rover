"""Windows Host Xbox Teleop -> ROS 2 via rosbridge (Path A).

WSL2 stock kernel lacks CONFIG_INPUT_JOYSTICK/CONFIG_JOYSTICK_XPAD so
/dev/input/js0 never appears. Run this on native Windows (pygame/XInput)
and publish geometry_msgs/msg/Twist to /cmd_vel via rosbridge 9090.

Mapping (teleop_twist_joy xbox.config.yaml):
  Button 2 (X) = Enable | Button 5 (RB) = Turbo
  Axis 1 = Linear X | Axis 3 = Angular Z
  Button 1 (B) = E-STOP latch | Button 7 (Start) = clear E-STOP

Usage (Windows PowerShell):
  pip install -r requirements-windows.txt
  python teleop_xbox_bridge.py
  usbipd detach --busid 2-3  # release from WSL2 first (then unbind)
"""
from __future__ import annotations
import argparse
import sys
import time
import pygame
import roslibpy

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 9090
DEFAULT_TOPIC = "/cmd_vel"

def apply_deadzone(val: float, dz: float = 0.1) -> float:
    return 0.0 if abs(val) < dz else val

def zero_twist() -> dict:
    return {"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}

def build_twist(lx: float, az: float) -> dict:
    return {"linear": {"x": float(lx), "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": float(az)}}

def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Windows Xbox teleop -> rosbridge /cmd_vel")
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--topic", default=DEFAULT_TOPIC)
    p.add_argument("--joystick", type=int, default=0)
    p.add_argument("--axis-linear", type=int, default=1)
    p.add_argument("--axis-angular", type=int, default=3)
    p.add_argument("--button-enable", type=int, default=2)
    p.add_argument("--button-turbo", type=int, default=5)
    p.add_argument("--linear-normal", type=float, default=0.3)
    p.add_argument("--linear-turbo", type=float, default=0.6)
    p.add_argument("--angular-scale", type=float, default=1.0)
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--deadzone", type=float, default=0.1)
    p.add_argument("--list", action="store_true")
    return p.parse_args(argv)

def list_joysticks() -> int:
    pygame.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    print(f"Joysticks detected: {count}")
    for i in range(count):
        try:
            j = pygame.joystick.Joystick(i)
            j.init()
            print(f"  [{i}] {j.get_name()} axes={j.get_numaxes()} buttons={j.get_numbuttons()}")
            j.quit()
        except pygame.error as exc:
            print(f"  [{i}] <error: {exc}>")
    pygame.quit()
    return 0 if count else 1

def wait_for_rosbridge(host: str, port: int, timeout: float = 30.0):
    deadline = time.time() + timeout
    attempt = 0
    while True:
        attempt += 1
        client = roslibpy.Ros(host=host, port=port)
        try:
            client.run()
            time.sleep(0.5)
            if client.is_connected:
                print(f"[+] Connected to rosbridge at {host}:{port} (attempt {attempt})")
                return client
            raise RuntimeError("not connected")
        except Exception as exc:
            print(f"[...] rosbridge attempt {attempt} failed: {exc}")
            try:
                client.terminate()
            except Exception:
                pass
            if time.time() >= deadline:
                raise RuntimeError(f"no rosbridge at {host}:{port} after {attempt} tries") from exc
            time.sleep(1.0)

def open_joystick(index: int):
    pygame.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    if count == 0:
        print("[!] No joystick on Windows. Connect controller, release from WSL2:")
        print("    usbipd detach --busid <busid> / usbipd unbind --busid <busid>")
        print("    Check joy.cpl, then re-run with --list.")
        sys.exit(1)
    if not 0 <= index < count:
        print(f"[!] Joystick {index} out of range 0..{count-1}. Use --list.")
        sys.exit(1)
    joy = pygame.joystick.Joystick(index)
    joy.init()
    print(f"[+] Joystick [{index}]: {joy.get_name()} axes={joy.get_numaxes()} buttons={joy.get_numbuttons()}")
    return joy

def safe_axis(joy, axis: int) -> float:
    if 0 <= axis < joy.get_numaxes():
        try:
            return float(joy.get_axis(axis))
        except pygame.error:
            return 0.0
    return 0.0

def safe_button(joy, btn: int) -> bool:
    if 0 <= btn < joy.get_numbuttons():
        try:
            return bool(joy.get_button(btn))
        except pygame.error:
            return False
    return False

def main(argv=None) -> int:
    args = parse_args(argv)
    if args.list:
        return list_joysticks()
    joy = open_joystick(args.joystick)
    try:
        client = wait_for_rosbridge(args.host, args.port)
    except RuntimeError as exc:
        print(f"[!] {exc}")
        pygame.quit()
        return 1
    topic = roslibpy.Topic(client, args.topic, "geometry_msgs/msg/Twist")
    print(f"[+] Publishing {args.topic} at {args.rate}Hz. Hold btn {args.button_enable} to drive, RB turbo.")
    print("    B(1)=E-STOP latch | Start(7)=clear | Ctrl+C=stop+zero.")
    interval = 1.0 / max(args.rate, 1.0)
    estop = False
    prev_e = prev_r = False
    try:
        while client.is_connected:
            start = time.time()
            pygame.event.pump()
            enabled = safe_button(joy, args.button_enable)
            turbo = safe_button(joy, args.button_turbo)
            e_btn = safe_button(joy, 1)
            r_btn = safe_button(joy, 7)
            if e_btn and not prev_e:
                estop = True
                print("[!] E-STOP latched. Press Start to clear.")
            if r_btn and not prev_r and estop:
                estop = False
                print("[+] E-STOP cleared.")
            prev_e, prev_r = e_btn, r_btn
            lx = az = 0.0
            if enabled and not estop:
                raw_l = -safe_axis(joy, args.axis_linear)
                raw_a = -safe_axis(joy, args.axis_angular)
                sc = args.linear_turbo if turbo else args.linear_normal
                lx = apply_deadzone(raw_l, args.deadzone) * sc
                az = apply_deadzone(raw_a, args.deadzone) * args.angular_scale
            try:
                topic.publish(roslibpy.Message(build_twist(lx, az)))
            except Exception as exc:
                print(f"[!] Publish failed: {exc}. Reconnecting...")
                try:
                    client.terminate()
                except Exception:
                    pass
                try:
                    client = wait_for_rosbridge(args.host, args.port)
                    topic = roslibpy.Topic(client, args.topic, "geometry_msgs/msg/Twist")
                except RuntimeError as exc2:
                    print(f"[!] {exc2}")
                    return 1
            time.sleep(max(0.0, interval - (time.time() - start)))
        print("[!] Lost rosbridge connection.")
        return 1
    except KeyboardInterrupt:
        print("\nStopping...")
        return 0
    finally:
        try:
            if client.is_connected:
                topic.publish(roslibpy.Message(zero_twist()))
                time.sleep(0.1)
        except Exception:
            pass
        try:
            topic.unadvertise()
        except Exception:
            pass
        try:
            client.terminate()
        except Exception:
            pass
        pygame.quit()
        print("[+] Zero sent. Closed.")

if __name__ == "__main__":
    raise SystemExit(main())


#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from typing import Tuple, Dict, Any


@dataclass(frozen=True)
class Quaternion:
    w: float
    x: float
    y: float
    z: float

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return (self.w, self.x, self.y, self.z)


def quat_conjugate(q: Quaternion) -> Quaternion:
    return Quaternion(q.w, -q.x, -q.y, -q.z)


def quat_norm2(q: Quaternion) -> float:
    return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z


def quat_inverse(q: Quaternion) -> Quaternion:
    n2 = quat_norm2(q)
    if n2 == 0.0:
        raise ValueError("Zero-norm quaternion cannot be inverted")
    qc = quat_conjugate(q)
    return Quaternion(qc.w / n2, qc.x / n2, qc.y / n2, qc.z / n2)


def quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    aw, ax, ay, az = a.w, a.x, a.y, a.z
    bw, bx, by, bz = b.w, b.x, b.y, b.z
    rw = aw * bw - ax * bx - ay * by - az * bz
    rx = aw * bx + ax * bw + ay * bz - az * by
    ry = aw * by - ax * bz + ay * bw + az * bx
    rz = aw * bz + ax * by - ay * bx + az * bw
    return Quaternion(rw, rx, ry, rz)


def yaw_from_quat(q: Quaternion, degrees: bool = False) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    if degrees:
        return math.degrees(yaw)
    return yaw


def euler_from_quat(q: Quaternion, degrees: bool = True) -> Tuple[float, float, float]:
    # Roll (x)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if sinp > 1.0:
        sinp = 1.0
    elif sinp < -1.0:
        sinp = -1.0
    pitch = math.asin(sinp)

    # Yaw (z)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    if degrees:
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
    return (roll, pitch, yaw)


def quat_from_yaw(yaw: float, degrees: bool = False) -> Quaternion:
    if degrees:
        yaw = math.radians(yaw)
    cy = math.cos(0.5 * yaw)
    sy = math.sin(0.5 * yaw)
    return Quaternion(cy, 0.0, 0.0, sy)


def small_angle_error_ex_ey(qd: Quaternion, qe: Quaternion) -> Tuple[float, float]:
    # q_err = q_des * inverse(q_est)
    inv_qe = quat_inverse(qe)
    er = quat_multiply(qd, inv_qe)
    if er.w < 0.0:
        er = Quaternion(-er.w, -er.x, -er.y, -er.z)
    ex = 2.0 * er.x
    ey = 2.0 * er.y
    return (ex, ey)


def _print(obj: Any, use_json: bool) -> None:
    if use_json:
        def conv(o: Any) -> Any:
            if isinstance(o, Quaternion):
                return {"w": o.w, "x": o.x, "y": o.y, "z": o.z}
            if isinstance(o, tuple):
                return list(o)
            return o
        print(json.dumps(conv(obj), ensure_ascii=False))
    else:
        if isinstance(obj, Quaternion):
            print(f"({obj.w:.9f}, {obj.x:.9f}, {obj.y:.9f}, {obj.z:.9f})")
        elif isinstance(obj, tuple):
            print(" ".join(f"{v:.9f}" for v in obj))
        else:
            print(obj)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Quaternion calculator (multiply, inverse, yaw, euler, from-yaw, small-angle-error)")
    p.add_argument("--json", action="store_true", help="Output JSON format")
    sub = p.add_subparsers(dest="cmd")

    def add_quat_args(sp: argparse.ArgumentParser, prefix: str) -> None:
        sp.add_argument(f"--{prefix}", nargs=4, type=float, metavar=("w", "x", "y", "z"), required=True,
                        help=f"Quaternion {prefix} components w x y z")

    sp_mul = sub.add_parser("multiply", help="Multiply two quaternions: r = a * b")
    add_quat_args(sp_mul, "a")
    add_quat_args(sp_mul, "b")

    sp_inv = sub.add_parser("inverse", help="Inverse of a quaternion")
    add_quat_args(sp_inv, "q")

    sp_yaw = sub.add_parser("yaw", help="Extract yaw from quaternion")
    add_quat_args(sp_yaw, "q")
    sp_yaw.add_argument("--deg", action="store_true", help="Return in degrees (default radians)")

    sp_euler = sub.add_parser("euler", help="Convert quaternion to Euler angles (roll, pitch, yaw)")
    add_quat_args(sp_euler, "q")
    sp_euler.add_argument("--rad", action="store_true", help="Return in radians (default degrees)")

    sp_fy = sub.add_parser("from-yaw", help="Create quaternion from yaw")
    sp_fy.add_argument("yaw", type=float, help="Yaw angle value")
    sp_fy.add_argument("--deg", action="store_true", help="Provided yaw in degrees (default radians)")

    sp_sae = sub.add_parser("small-angle-error", help="Compute small angle error ex, ey from desired/estimated quaternions")
    add_quat_args(sp_sae, "qd")
    add_quat_args(sp_sae, "qe")

    return p


def parse_quat(arg: Tuple[float, float, float, float]) -> Quaternion:
    w, x, y, z = arg
    return Quaternion(float(w), float(x), float(y), float(z))


def main(argv=None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.cmd is None:
        parser.print_help()
        print("\nExamples:")
        print("  python x_memory/py/quaternion_calc.py multiply --a 1 0 0 0 --b 0.70710678 0 0 0.70710678")
        print("  python x_memory/py/quaternion_calc.py inverse --q 0.9238795 0 0.3826834 0")
        print("  python x_memory/py/quaternion_calc.py yaw --q 0.9238795 0 0.3826834 0 --deg")
        print("  python x_memory/py/quaternion_calc.py euler --q 0.70710678 0.70710678 0 0")
        print("  python x_memory/py/quaternion_calc.py from-yaw 45 --deg")
        print("  python x_memory/py/quaternion_calc.py small-angle-error --qd 1 0 0 0 --qe 0.9807853 0 0.1950903 0")
        return

    if args.cmd == "multiply":
        a = parse_quat(tuple(args.a))
        b = parse_quat(tuple(args.b))
        r = quat_multiply(a, b)
        _print(r, args.json)
        return

    if args.cmd == "inverse":
        q = parse_quat(tuple(args.q))
        r = quat_inverse(q)
        _print(r, args.json)
        return

    if args.cmd == "yaw":
        q = parse_quat(tuple(args.q))
        y = yaw_from_quat(q, degrees=bool(args.deg))
        _print(y, args.json)
        return

    if args.cmd == "euler":
        q = parse_quat(tuple(args.q))
        roll, pitch, yaw = euler_from_quat(q, degrees=not bool(args.rad))
        _print((roll, pitch, yaw), args.json)
        return

    if args.cmd == "from-yaw":
        q = quat_from_yaw(args.yaw, degrees=bool(args.deg))
        _print(q, args.json)
        return

    if args.cmd == "small-angle-error":
        qd = parse_quat(tuple(args.qd))
        qe = parse_quat(tuple(args.qe))
        ex, ey = small_angle_error_ex_ey(qd, qe)
        _print((ex, ey), args.json)
        return

    parser.error("Unknown command")


if __name__ == "__main__":
    main()



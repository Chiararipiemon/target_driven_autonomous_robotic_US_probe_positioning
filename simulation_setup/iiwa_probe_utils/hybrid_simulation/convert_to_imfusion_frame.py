#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

This script converts a list of poses exported from the robot/MoveIt world frame
into the ImFusion frame, and writes a new CSV in the format expected by ImFusion.

Input CSV format (each row must contain at least 7 numeric values):
  ... x, y, z, qx, qy, qz, qw
or exactly:
  x, y, z, qx, qy, qz, qw

Output CSV format:
  x_mm, y_mm, z_mm, qx, qy, qz, qw
where the pose is expressed in the ImFusion coordinate frame.

Notes about units:
- If INPUT_UNITS = "m", translations are interpreted as meters and are converted
  to millimeters in the output.
- If INPUT_UNITS = "mm", translations are assumed already in millimeters.

The homogeneous transform T_IMFUSION_FROM_ROBOT is applied as:
  T_imfusion = T_IMFUSION_FROM_ROBOT @ T_robot
"""

import csv
import os
import numpy as np

# ========= CONFIG =========

# Input CSV (poses in robot/world source frame).
# Each row must contain ... x, y, z, qx, qy, qz, qw
CSV_IN  = "/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/raster/18Jan/serpentina_only_no_spikes_us_poses_1768732237.csv"

# Output CSV compatible with ImFusion (poses in ImFusion frame).
CSV_OUT = "/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/raster/18Jan/us_poses_1768732237_serpentine_imfusion_frame.csv"

# Units of the INPUT CSV translations:
# - "m"  if positions are in meters
# - "mm" if positions are already in millimeters
INPUT_UNITS = "mm"  # here: input already saved in mm, so keep "mm"

# Homogeneous transform from robot frame to ImFusion frame (translation in mm).
# This matrix maps a pose expressed in the robot frame into the ImFusion frame.
T_IMFUSION_FROM_ROBOT = np.array([
    [1.0,  0.0,  0.0, -642.0],
    [0.0,  0.0,  1.0, -364.0],  # e.g., extra -84 mm offset included here
    [0.0, -1.0,  0.0, -200.0],
    [0.0,  0.0,  0.0,    1.0]
], dtype=float)

# ==========================


def quat_to_rot(qx, qy, qz, qw):
    """
    Convert a quaternion (x, y, z, w) into a 3x3 rotation matrix.

    The quaternion is normalized before conversion. If the quaternion norm is zero,
    the identity matrix is returned.
    """
    q = np.array([qx, qy, qz, qw], dtype=float)
    n = np.linalg.norm(q)
    if n == 0:
        return np.eye(3)
    q /= n
    x, y, z, w = q

    # Standard quaternion-to-rotation formula
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ], dtype=float)


def rot_to_quat(R):
    """
    Convert a 3x3 rotation matrix into a quaternion (x, y, z, w).

    Uses a common stable algorithm based on the matrix trace.
    The resulting quaternion is normalized. If normalization fails, returns identity quaternion.
    """
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    tr = m00 + m11 + m22

    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S

    q = np.array([qx, qy, qz, qw], dtype=float)
    n = np.linalg.norm(q)
    if n == 0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def load_poses(csv_path):
    """
    Load poses from the input CSV.

    Supported row formats:
      - [..., x, y, z, qx, qy, qz, qw]
      - [x, y, z, qx, qy, qz, qw]

    Returns:
      A list of 4x4 homogeneous transforms (numpy arrays), with translations
      in the input units (meters or millimeters depending on INPUT_UNITS).
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"Input file not found: {csv_path}")

    poses = []
    with open(csv_path, "r") as f:
        r = csv.reader(f)
        header = next(r, None)

        def row_to_pose(row):
            # Parse numeric values, then extract the last 7 as pose data
            vals = [float(v) for v in row]
            if len(vals) < 7:
                return None
            x, y, z, qx, qy, qz, qw = vals[-7:]

            # Build homogeneous transform
            R = quat_to_rot(qx, qy, qz, qw)
            T = np.eye(4, dtype=float)
            T[:3, :3] = R
            T[:3, 3] = [x, y, z]
            return T

        # If the first row is numeric, treat it as data (no textual header)
        if header:
            try:
                _ = [float(v) for v in header]
                pose = row_to_pose(header)
                if pose is not None:
                    poses.append(pose)
            except ValueError:
                # Text header: ignore it
                pass

        # Parse the rest of the rows
        for row in r:
            if not row:
                continue
            try:
                pose = row_to_pose(row)
            except ValueError:
                # Skip non-numeric rows
                continue
            if pose is not None:
                poses.append(pose)

    if not poses:
        raise RuntimeError("No valid poses found in the input CSV")

    return poses


def convert_to_imfusion_frame(poses):
    """
    Convert poses from the robot frame to the ImFusion frame.

    Operation:
      T_imfusion = T_IMFUSION_FROM_ROBOT @ T_robot

    Unit handling:
      If INPUT_UNITS == "m", translations are converted to mm after applying the transform.
      If INPUT_UNITS == "mm", translations are assumed already in mm.

    Returns:
      A list of 4x4 homogeneous transforms in the ImFusion frame.
    """
    T_conv = T_IMFUSION_FROM_ROBOT.copy()
    out = []

    for T in poses:
        # Apply the frame transform
        T_if = T_conv @ T

        # Convert meters -> millimeters if needed
        if INPUT_UNITS.lower() == "m":
            T_if = T_if.copy()
            T_if[:3, 3] *= 1000.0

        out.append(T_if)

    return out


def save_imfusion_csv(poses_if, csv_path):
    """
    Save the converted poses to a CSV compatible with ImFusion.

    Output columns:
      x_mm, y_mm, z_mm, qx, qy, qz, qw

    Where:
      - translations are in millimeters (ImFusion frame)
      - quaternion is (x, y, z, w)
    """
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x_mm", "y_mm", "z_mm", "qx", "qy", "qz", "qw"])

        for T in poses_if:
            x, y, z = T[:3, 3]
            R = T[:3, :3]
            qx, qy, qz, qw = rot_to_quat(R)
            w.writerow([x, y, z, qx, qy, qz, qw])


def main():
    """
    Main entry point:
      1) Load robot-frame poses from CSV_IN
      2) Convert them to ImFusion frame
      3) Save them to CSV_OUT
    """
    poses_robot = load_poses(CSV_IN)
    poses_if = convert_to_imfusion_frame(poses_robot)
    save_imfusion_csv(poses_if, CSV_OUT)
    print(f"Converted {len(poses_if)} poses to ImFusion frame (x,y,z,qx,qy,qz,qw) -> {CSV_OUT}")


if __name__ == "__main__":
    main()

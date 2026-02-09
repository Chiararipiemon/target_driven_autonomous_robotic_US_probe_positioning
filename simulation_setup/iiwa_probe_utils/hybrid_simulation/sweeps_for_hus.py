import imfusion
import numpy as np
import csv
import os

# This script:
# - loads a tracking path from a CSV file (already expressed in the ImFusion world/frame),
# - smooths the path to reduce zig-zag artifacts,
# - resamples it to exactly N_FRAMES points to build the "Transducer Spline",
# - creates the "Direction Spline" by translating the Transducer Spline by a fixed offset
#   along a chosen world axis.
#
# The generated splines are ImFusion annotations, intended to be reused later (e.g., exported/converted
# to a GlSpline inside an XML workflow).

# ================= CONFIG =================

# Path to the labelmap dataset used as a reference/parent dataset for the annotations.
LABELMAP_PATH = r"/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/segm_relabel.nii.gz"

# Input CSV containing poses/points already registered into the ImFusion frame.
# This script only uses the first three columns: x, y, z.
CSV_PATH      = r"/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/raster/18Jan/us_poses_1768732237_serpentine_imfusion_frame.csv"

# Number of points used for each spline after resampling.
N_FRAMES            = 100

# Unit of the CSV coordinates:
# - "mm" if the CSV is already in millimeters
# - "m"  if the CSV is in meters (will be converted to mm)
CSV_UNITS           = "mm"

# Optional global offset along world Y (mm). Keep at 0 if the tracking is already correct.
Y_OFFSET_MM         = 0.0

# Offset magnitude (mm) used to create the Direction Spline as a translated copy of Transducer Spline.
DIRECTION_OFFSET_MM = 70.0

# Half-window size for a simple moving average smoother (in samples).
# Larger values produce smoother but more lagged trajectories.
SMOOTH_WINDOW       = 15

# World-axis direction used for translating the Direction Spline.
# Choose one of: "+Z", "-Z", "+Y", "-Y", "+X", "-X"
DIRECTION_WORLD_AXIS = "-Y"

# ==========================================


def ensure_app():
    """Ensure that imfusion.app exists (useful when running outside ImFusion Suite)."""
    if not getattr(imfusion, "app", None):
        imfusion.app = imfusion.ConsoleController()


def dm():
    """Shortcut to the ImFusion data model."""
    return imfusion.app.data_model


def am():
    """Return the current ImFusion annotation model (must exist in an open workspace)."""
    model = getattr(imfusion.app, "annotation_model", None)
    if model is None:
        raise RuntimeError(
            "annotation_model is None. "
            "Run this script inside the ImFusion Python console with a workspace open."
        )
    return model


def get_all_annotations(model):
    """Return all annotations from the annotation model, handling API variations."""
    if not hasattr(model, "annotations"):
        return []
    anns = model.annotations
    # Some ImFusion versions expose annotations as a callable.
    if callable(anns):
        try:
            return list(anns())
        except TypeError:
            return []
    # Others expose it as an iterable/list-like object.
    try:
        return list(anns)
    except TypeError:
        return []


def delete_existing_splines():
    """Remove any previously created splines with the standard names used by this script."""
    model = am()
    to_remove = [
        a for a in get_all_annotations(model)
        if getattr(a, "name", "") in ("Transducer Spline", "Direction Spline")
    ]
    for a in to_remove:
        # Depending on the ImFusion API version, the removal function name can differ.
        if hasattr(model, "remove_annotation"):
            try:
                model.remove_annotation(a)
                continue
            except Exception:
                pass
        if hasattr(model, "remove"):
            try:
                model.remove(a)
            except Exception:
                pass
    if to_remove:
        print(f"Removed {len(to_remove)} old spline(s).")


def get_or_load_segm():
    """
    Retrieve the labelmap dataset called 'segm_relabel' from the data model if present;
    otherwise load it from disk and add it to the data model.
    """
    for d in dm():
        if "segm_relabel" in str(getattr(d, "name", "")):
            return d

    if not os.path.exists(LABELMAP_PATH):
        raise RuntimeError(f"Labelmap not found: {LABELMAP_PATH}")

    data_list = imfusion.load(LABELMAP_PATH)
    if not data_list:
        raise RuntimeError(f"Unable to load labelmap from {LABELMAP_PATH}")

    d = dm().add(data_list[0])
    d.name = "segm_relabel"
    return d


def load_centers(csv_path, units="mm"):
    """
    Load only (x,y,z) from a CSV file (with header).
    Optionally convert to millimeters and apply an optional global Y offset.

    The CSV is expected to have at least 3 numeric columns per row.
    """
    centers = []
    with open(csv_path, "r") as f:
        r = csv.reader(f)
        next(r, None)  # skip header
        for row in r:
            if not row:
                continue
            try:
                vals = [float(v) for v in row]
            except ValueError:
                # Skip non-numeric rows
                continue
            if len(vals) < 3:
                continue
            x, y, z = vals[0], vals[1], vals[2]
            centers.append([x, y, z])

    if not centers:
        raise RuntimeError("No valid positions found in the CSV")

    centers = np.asarray(centers, float)

    # Convert meters to millimeters if needed
    if units.lower() == "m":
        centers *= 1000.0

    # Optional global Y offset (mm)
    centers[:, 1] += Y_OFFSET_MM

    return centers


def smooth_moving_average(points, window):
    """
    Apply a simple 3D moving average along the path.

    'window' is a half-window size:
      output[i] = mean(points[i-window : i+window+1])
    """
    pts = np.asarray(points, float)
    N = len(pts)
    if N < 3 or window <= 0:
        return pts.copy()

    out = np.zeros_like(pts)
    for i in range(N):
        i0 = max(0, i - window)
        i1 = min(N, i + window + 1)
        out[i] = pts[i0:i1].mean(axis=0)
    return out


def resample_along_curve(points, k):
    """
    Uniformly resample a polyline by arc length.

    Given a sequence of points defining a polyline, this returns k points that are
    equally spaced along the curve length.
    """
    pts = np.asarray(points, float)
    if k <= 1 or len(pts) < 2:
        return np.repeat(pts[:1], k, axis=0)

    # Segment lengths and cumulative arc-length
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    dist = np.concatenate(([0.0], np.cumsum(seg)))
    total = dist[-1]

    if total == 0.0:
        return np.repeat(pts[:1], k, axis=0)

    # Target distances along the curve
    targets = np.linspace(0.0, total, k)
    out = []
    j = 0
    for td in targets:
        # Move to the segment containing td
        while j + 1 < len(dist) and dist[j + 1] < td:
            j += 1

        # If td is beyond the last segment, clamp to the last point
        if j + 1 == len(dist):
            out.append(pts[-1])
        else:
            # Linear interpolation within the segment [j, j+1]
            t0, t1 = dist[j], dist[j + 1]
            alpha = (td - t0) / (t1 - t0) if t1 > t0 else 0.0
            p = (1.0 - alpha) * pts[j] + alpha * pts[j + 1]
            out.append(p)

    return np.vstack(out)


def get_direction_vector():
    """Return a unit vector corresponding to DIRECTION_WORLD_AXIS."""
    ax = DIRECTION_WORLD_AXIS.upper()
    if ax == "+Z":
        v = np.array([0.0, 0.0, 1.0])
    elif ax == "-Z":
        v = np.array([0.0, 0.0, -1.0])
    elif ax == "+Y":
        v = np.array([0.0, 1.0, 0.0])
    elif ax == "-Y":
        v = np.array([0.0, -1.0, 0.0])
    elif ax == "+X":
        v = np.array([1.0, 0.0, 0.0])
    elif ax == "-X":
        v = np.array([-1.0, 0.0, 0.0])
    else:
        raise ValueError("DIRECTION_WORLD_AXIS must be one of '+Z', '-Z', '+Y', '-Y', '+X', '-X'")
    return v / np.linalg.norm(v)


def compute_splines(centers_raw):
    """
    Build the two splines from the raw tracking points:

    Transducer Spline:
      - smooth the raw tracking with a moving average,
      - resample uniformly along arc length to N_FRAMES points.

    Direction Spline:
      - translate the Transducer Spline by DIRECTION_OFFSET_MM along DIRECTION_WORLD_AXIS.
    """
    # Smooth the original tracking
    smooth = smooth_moving_average(centers_raw, SMOOTH_WINDOW)

    # Resample the smoothed curve to N_FRAMES points
    centers = resample_along_curve(smooth, N_FRAMES)

    # Create the direction spline as an offset copy
    dir_vec = get_direction_vector()
    dirs = centers + dir_vec * DIRECTION_OFFSET_MM

    return centers, dirs


def get_spline_type():
    """
    Pick a spline-like annotation type supported by the installed ImFusion version.
    Different builds can expose different names for spline annotations.
    """
    at = imfusion.Annotation.AnnotationType
    for name in ("SPLINE_3D", "SPLINE", "SMART_SPLINE", "POLY_LINE"):
        if hasattr(at, name):
            return getattr(at, name)
    raise RuntimeError("No spline annotation type available in this ImFusion build")


def create_spline(name, points, dataset):
    """
    Create (or replace) a spline annotation with a given name and list of 3D points.
    The annotation is also associated to the provided dataset when possible.
    """
    model = am()
    spline_type = get_spline_type()

    # Remove an existing annotation with the same name (if any)
    for a in get_all_annotations(model):
        if getattr(a, "name", "") == name:
            if hasattr(model, "remove_annotation"):
                try:
                    model.remove_annotation(a)
                    continue
                except Exception:
                    pass
            if hasattr(model, "remove"):
                try:
                    model.remove(a)
                except Exception:
                    pass

    # Create the new spline annotation
    ann = model.create_annotation(spline_type)
    ann.name = name
    ann.points = [tuple(map(float, p)) for p in points]

    # Try to attach the annotation to the dataset (API differences across versions)
    for attr in ("data", "dataset", "parent", "parent_dataset", "parentData"):
        if hasattr(ann, attr):
            try:
                setattr(ann, attr, dataset)
                break
            except Exception:
                pass

    print(f"{name}: {len(ann.points)} points (type={spline_type})")
    return ann


def main():
    """Entry point: load tracking, compute splines, and create ImFusion annotations."""
    ensure_app()
    delete_existing_splines()

    segm = get_or_load_segm()
    print("Loaded segm_relabel")

    centers_raw = load_centers(CSV_PATH, units=CSV_UNITS)
    centers, dirs = compute_splines(centers_raw)

    create_spline("Transducer Spline", centers, segm)
    create_spline("Direction Spline",  dirs,    segm)

    print("Done: smoothed Transducer Spline + translated parallel Direction Spline created.")


if __name__ == "__main__":
    main()

import h5py  # HDF5 I/O (convenient container for multi-frame ultrasound sequences)
import numpy as np

from monai.transforms import UltrasoundConfidenceMapTransform  # MONAI transform for confidence map estimation
# Implementation reference (MONAI): random-walk based confidence map for ultrasound
# https://github.com/Project-MONAI/MONAI/blob/57fdd594ac905ab2ea778aa4bb79ccd9a0a03b22/monai/transforms/intensity/array.py


# FILE PATHS (HDF5)

in_h5 = "input.h5"          # Input HDF5 file containing ultrasound frames (and optional metadata)
out_h5 = "output_single.h5" # Output HDF5 file that will store the computed confidence maps


# DATASET KEYS (HDF5 groups)

DATASET_IN = "Ultrasound 5"   # HDF5 key containing image frames (e.g., shape (N, 1, H, W))
DATASET_OUT = "confidence"    # Output dataset key for confidence maps

# If you want to process only the first N frames for a quick test, set N_TEST to an int.
# Set to None to process the entire sequence.
N_TEST = None


# MONAI TRANSFORM SETUP

# UltrasoundConfidenceMapTransform estimates a confidence value per pixel (typically higher where the
# ultrasound signal is more reliable). The algorithm is based on a random-walk formulation with seed/sink
# selection and intensity-dependent costs.
t = UltrasoundConfidenceMapTransform(
    alpha=2.0,
    beta=90.0,
    gamma=0.05,
    mode="B",          # ultrasound mode (e.g., B-mode)
    sink_mode="all",   # let MONAI automatically select sink points
    use_cg=False,      # solve the linear system with the default solver (not conjugate gradient)
)

def get_frame(ds, i):
    """
    Extract a single frame from an HDF5 dataset and return it as a 2D float32 array.

    Parameters
    ----------
    ds : h5py.Dataset
        The HDF5 dataset storing the image sequence.
    i : int
        Index of the frame to extract.

    Returns
    -------
    frame : np.ndarray, shape (H, W), dtype float32
        The extracted 2D image frame in float32.

    Notes
    -----
    This helper supports a few common dataset layouts:
      - (N, H, W)
      - (N, 1, H, W)          (single-channel with explicit channel axis)
      - (N, H, W, 3)          (RGB, converted to grayscale by channel averaging)
      - (H, W, N)             (less common; frames stored in last axis)
    """
    shp = ds.shape

    # Case: (N, H, W)
    if len(shp) == 3:
        frame = ds[i, :, :]
        return frame.astype(np.float32, copy=False)

    # Case: (N, 1, H, W)
    if len(shp) == 4 and shp[1] == 1:
        frame = ds[i, 0, :, :]
        # MONAI expects an input shaped like (C, H, W) or (1, H, W) for single-channel images.
        # We return (H, W) here and add the leading channel dimension later.
        return frame.astype(np.float32, copy=False)

    # Case: (N, H, W, 3) RGB -> grayscale
    if len(shp) == 4 and shp[-1] == 3:
        rgb = ds[i, :, :, :]
        frame = np.mean(rgb, axis=-1)
        return frame.astype(np.float32, copy=False)

    # Case: (H, W, N) (ambiguous but sometimes used)
    if len(shp) == 3 and shp[-1] > 1 and shp[0] != shp[-1]:
        frame = ds[:, :, i]
        return frame.astype(np.float32, copy=False)

    raise ValueError(f"Unsupported dataset shape: {shp}")


# MAIN CONVERSION PIPELINE

# This script reads an ultrasound sequence from an HDF5 dataset, computes a confidence map for each frame,
# and writes the resulting confidence maps to a new HDF5 file. It optionally copies spacing/shiftScale
# metadata in a way that is compatible with ImFusion conventions.
with h5py.File(in_h5, "r") as f_in, h5py.File(out_h5, "w") as f_out:
    ds_in = f_in[DATASET_IN]  # Input dataset handle (lazy access; frames read on demand)

    # Infer (N, H, W) depending on how the dataset is stored
    if len(ds_in.shape) == 3:
        # (N, H, W)
        N, H, W = ds_in.shape
    elif len(ds_in.shape) == 4 and ds_in.shape[1] == 1:
        # (N, 1, H, W)
        N, _, H, W = ds_in.shape
    elif len(ds_in.shape) == 4 and ds_in.shape[-1] == 3:
        # (N, H, W, 3)
        N, H, W, _ = ds_in.shape
    else:
        # (H, W, N)
        H, W, N = ds_in.shape

    # Decide how many frames to process
    N_out = N if N_TEST is None else min(N, int(N_TEST))


    # OPTIONAL METADATA COPYING

    # ImFusion often expects spacing and shift/scale metadata for derived datasets.
    # Here we copy the input metadata (if present) to new keys associated with the confidence dataset.
    if "Ultrasound 5_spacing" in f_in:
        f_out.create_dataset("confidence_spacing", data=f_in["Ultrasound 5_spacing"][:])
    if "Ultrasound 5_shiftScale" in f_in:
        f_out.create_dataset("confidence_shiftScale", data=f_in["Ultrasound 5_shiftScale"][:])

    
    # PADDING STRATEGY (IMPORTANT)

    # MONAI's random-walk confidence map can become numerically/heuristically degenerate on very narrow images
    # (e.g., 512 x 128). In that case the seed/sink selection or the internal graph construction may lead to a
    # near-constant output confidence map.
    #
    # A practical workaround is to pad the width to a more "square" geometry (e.g., 512 x 512), compute the
    # confidence map on the padded image, and then crop back to the original width.
    target_W = 512
    if W > target_W:
        raise ValueError(f"W={W} > target_W={target_W}. Reduce target_W or change padding strategy.")
    pad_left = (target_W - W) // 2
    pad_right = target_W - W - pad_left

    print(f"Input frames: N={N}, H={H}, W={W}")
    print(f"Processing frames: {N_out}/{N}")
    print(f"Padding width: left={pad_left}, right={pad_right} -> padded W={target_W}")
    print(f"Output file: {out_h5}")

    # Create output dataset for confidence maps:
    # - stored as (N_out, H, W)
    # - float32 to match typical confidence map output
    # - chunked by frame for efficient streaming write
    # - gzip compression to reduce file size
    ds_out = f_out.create_dataset(
        DATASET_OUT,
        shape=(N_out, H, W),
        dtype=np.float32,
        chunks=(1, H, W),
        compression="gzip",
        compression_opts=4,
    )

    # Frame-by-frame processing loop
    for i in range(N_out):
        # 1) Load a single ultrasound frame as (H, W)
        frame = get_frame(ds_in, i)

        # 2) Normalize input intensities to [0, 1] (assumes original frame is uint8-like in [0, 255])
        frame = frame / 255.0
        frame = np.clip(frame, 0.0, 1.0)

        # 3) Pad width to target_W using edge padding (keeps boundary values constant)
        #    Result shape: (H, target_W)
        frame_p = np.pad(frame, ((0, 0), (pad_left, pad_right)), mode="edge")

        # 4) Compute confidence map on padded frame
        #    MONAI expects shape (C, H, W) for a single image; here C=1.
        #    Depending on MONAI version, the output can be (1, H, W) or (H, W).
        conf_p = np.asarray(t(frame_p[None, ...]))
        conf_p = conf_p[0] if conf_p.ndim == 3 else conf_p  # ensure (H, target_W)

        # 5) Crop confidence map back to original width (H, W)
        conf2d = conf_p[:, pad_left : pad_left + W]

        # 6) Save confidence map frame to the output dataset
        ds_out[i, :, :] = conf2d.astype(np.float32, copy=False)

        # Periodic progress log
        if i % 50 == 0:
            print(f"Processed {i}/{N_out} frames")

print("Done.")

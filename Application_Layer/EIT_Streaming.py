#!/usr/bin/env python3
"""
EITVis_v7: Real-time EIT acquisition & reconstruction
(Press 0 → capture baseline V0, Press 1 → stream & reconstruct V1)
Uses standard protocol: 8 electrodes × 5 measurements = 40 channels.
"""

import sys, time
import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from pyeit.mesh import create
from pyeit.eit.protocol import create as protocol_create
from pyeit.eit.jac import JAC
from pyeit.eit.interp2d import sim2pts

# ---------------- Serial configuration ---------------- #
PORT = "/dev/cu.usbmodem1101"   # modify if your port changes
BAUD = 115200
TIMEOUT = 1

# ---------------- Data/plot configuration ---------------- #
NUM_CHANNELS = 40  # 8 excitations × 5 measurements
CAPTURE_FRAMES = 40
UPDATE_MS = 50
EIT_ALPHA = 0.9
CLIM_MIN, CLIM_MAX = 0.1, 4.0  # Tighter color limits for more contrast
CLIM_PERCENTILE = 90  # Use 90th percentile instead of 98th for more sensitive scaling

# ---------------- Debug / logging ---------------- #
# Print raw serial lines as they are received (can be very verbose).
PRINT_RX_RAW = False
# Also print a short summary of the parsed numeric frame.
PRINT_RX_PARSED_SUMMARY = True
# Print every N valid frames (set to 1 to print every frame).
PRINT_RX_EVERY_N = 1

# ---------------- Filtering configuration ---------------- #
OUTLIER_THRESHOLD = 3.0  # MAD threshold for outlier removal (higher = less aggressive)

# ---------------- EIT configuration ---------------- #
N_ELEC = 8
H0 = 0.05
DIST_EXC = 1
STEP_MEAS = 1
PARSER_MEAS = 'std'  # Standard protocol: 8 × 5 = 40 measurements
JAC_P, JAC_LAMB = 0.5, 0.3  # Increased lamb for stronger regularization (reduces artifacts)


# ------------------------------------------------------------
def open_serial():
    """Open Teensy serial port."""
    print(f"Opening serial port {PORT} @ {BAUD} baud ...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        time.sleep(2)
        ser.reset_input_buffer()
        print("Serial ready.\nPress 0 to capture V0, press 1 to stream V1.")
        return ser
    except serial.SerialException as e:
        print("Serial open error:", e)
        sys.exit(1)


def read_frame(ser):
    """Read one line of float array (40 floats)."""
    raw = ser.readline().decode("ascii", errors="ignore").strip()
    if not raw:
        return None, None
    parts = raw.replace(",", " ").split()
    if len(parts) < NUM_CHANNELS:
        return raw, None
    try:
        vals = np.array(list(map(float, parts[:NUM_CHANNELS])))
        return raw, vals
    except ValueError:
        return raw, None


def filter_outliers(data, v0_ref, threshold=3.0):
    """Remove outliers by replacing values too far from baseline."""
    diff = np.abs(data - v0_ref)
    median_diff = np.median(diff)
    mad = np.median(np.abs(diff - median_diff))
    if mad < 1e-10:
        return data
    z_score = (diff - median_diff) / (1.4826 * mad)  # MAD to std conversion
    filtered = data.copy()
    outliers = z_score > threshold
    if np.any(outliers):
        filtered[outliers] = v0_ref[outliers]
    return filtered


def face_values(mesh_obj, values):
    """Convert node-based values to face-based values."""
    if len(values) == len(mesh_obj.node):
        return np.mean(values[mesh_obj.element], axis=1)
    return values


# ------------------------------------------------------------
def main():
    # --- setup EIT ---
    print("Creating EIT mesh and protocol...")
    mesh_obj = create(N_ELEC, h0=H0)
    protocol = protocol_create(
        N_ELEC,
        dist_exc=DIST_EXC,
        step_meas=STEP_MEAS,
        parser_meas=PARSER_MEAS,
    )
    print(f"Protocol created: {protocol.n_exc} excitations × {protocol.n_meas} measurements")

    eit = JAC(mesh_obj, protocol)
    eit.setup(p=JAC_P, lamb=JAC_LAMB, method="kotre",
              perm=1.0, jac_normalized=True)

    print(f"Jacobian shape: {eit.J.shape}")
    print("Using standard protocol with 40 measurements (8 excitations × 5 measurements).")

    ser = open_serial()

    # --- figure setup ---
    pts, tri = mesh_obj.node, mesh_obj.element
    fig, ax = plt.subplots(figsize=(7, 7))
    pcm = ax.tripcolor(pts[:, 0], pts[:, 1], tri,
                       facecolors=np.zeros(len(tri)),
                       shading="flat", cmap="coolwarm")
    cb = fig.colorbar(pcm, ax=ax, shrink=0.85, pad=0.02)
    cb.set_label("Δσ (a.u.)")  # Positive Δσ (Red) = More conductive | Negative Δσ (Blue) = Less conductive

    # --- Draw electrode markers and labels ---
    n_electrodes = N_ELEC
    angles = np.linspace(0, 2 * np.pi, n_electrodes, endpoint=False)
    electrode_radius = 1.0  # Match the mesh boundary radius

    for i, angle in enumerate(angles):
        x = electrode_radius * np.cos(angle)
        y = electrode_radius * np.sin(angle)

        # Draw electrode marker (circle)
        ax.plot(x, y, 'o', color='black', markersize=12, markeredgecolor='white',
                markeredgewidth=2, zorder=10)

        # Add electrode number label
        label_offset = 1.15  # Position label slightly outside the circle
        x_label = label_offset * electrode_radius * np.cos(angle)
        y_label = label_offset * electrode_radius * np.sin(angle)
        ax.text(x_label, y_label, str(i), fontsize=11, fontweight='bold',
                ha='center', va='center', color='black',
                bbox=dict(boxstyle='circle,pad=0.3', facecolor='yellow',
                         edgecolor='black', linewidth=1.5, alpha=0.9), zorder=11)

    ax.set_aspect("equal")
    ax.set_xlim(-1.35, 1.35)
    ax.set_ylim(-1.35, 1.35)
    ax.axis("off")
    status_text = fig.text(0.02, 0.02, "Idle: press 0 to capture baseline V0", ha="left", va="bottom", fontsize=10)

    state = {
        "mode": "idle",
        "capture": [],
        "v0": None,
        "last_ds": None,
        "frame_idx": 0,
        "rx_valid_count": 0,
    }

    def set_status(msg):
        status_text.set_text(msg)

    # --------------------------------------------------------
    def on_key(event):
        key = event.key
        if key == "0":
            state["mode"] = "capture_v0"
            state["capture"].clear()
            set_status("Capturing V0 baseline ...")
            print("---- Capturing V0 baseline ----")

        elif key == "1":
            if state["v0"] is None:
                print("Press 0 first to capture V0 baseline.")
                return
            state["mode"] = "measure_v1"
            state["frame_idx"] = 0  # Reset frame counter
            state["last_ds"] = None  # Reset temporal filter
            set_status("Streaming and reconstructing V1 ...")
            print("---- Start streaming V1 ----")

    fig.canvas.mpl_connect("key_press_event", on_key)

    # --------------------------------------------------------
    def update(_):
        raw, frame = read_frame(ser)
        if frame is None:
            set_status(f"{state['mode']} | waiting for valid data ...")
            return (pcm,)

        trimmed = frame[:NUM_CHANNELS]  # 40 channels (8 × 5 measurements)
        state["rx_valid_count"] += 1

        # ---- Debug print received data ----
        if PRINT_RX_EVERY_N > 0 and (state["rx_valid_count"] % PRINT_RX_EVERY_N == 0):
            if PRINT_RX_RAW and raw is not None:
                print(f"[RX {state['rx_valid_count']:06d}] {raw}")
            if PRINT_RX_PARSED_SUMMARY:
                # Keep this lightweight to avoid slowing the animation loop too much.
                print(
                    f"[RX {state['rx_valid_count']:06d}] "
                    f"len={len(trimmed)} mean={float(np.mean(trimmed)):.6f} "
                    f"std={float(np.std(trimmed)):.6f} "
                    f"min={float(np.min(trimmed)):.6f} max={float(np.max(trimmed)):.6f}"
                )

        if state["mode"] == "capture_v0":
            state["capture"].append(trimmed)
            progress = len(state["capture"])
            set_status(f"Capturing V0 {progress}/{CAPTURE_FRAMES}")
            if progress >= CAPTURE_FRAMES:
                # Use median instead of mean for more robust baseline
                state["v0"] = np.median(state["capture"], axis=0)
                np.save("V0.npy", state["v0"])
                print(f"Baseline V0 captured, len={len(state['v0'])}")
                print(state["v0"])
                state["capture"].clear()
                state["mode"] = "idle"
                state["last_ds"] = None  # Reset filtered reconstruction
                set_status("V0 ready. Press 1 to stream V1.")
            return (pcm,)

        elif state["mode"] == "measure_v1":
            state["frame_idx"] += 1
            if state["frame_idx"] == 1:
                print(f"First V1 frame len={len(trimmed)}")

            try:
                # Filter outliers in raw measurements
                v1_filtered = filter_outliers(trimmed, state["v0"], threshold=OUTLIER_THRESHOLD)

                # Solve with normalization enabled to reduce artifacts
                ds = eit.solve(v1_filtered, state["v0"], normalize=True)
                filt = ds if state["last_ds"] is None else (
                    (1 - EIT_ALPHA) * state["last_ds"] + EIT_ALPHA * ds)
                state["last_ds"] = filt

                mapped = sim2pts(mesh_obj.node, mesh_obj.element, np.real(filt))
                face_vals = face_values(mesh_obj, mapped)
                pcm.set_array(face_vals)

                # Dynamic color scaling based on actual data range
                clim_raw = np.percentile(np.abs(face_vals), CLIM_PERCENTILE)
                clim = float(np.clip(clim_raw, CLIM_MIN, CLIM_MAX))
                pcm.set_clim(-clim, clim)
                cb.update_normal(pcm)
                set_status(f"Frame {state['frame_idx']}: Δσ range ±{clim:.2f}")
            except Exception as e:
                print("EIT reconstruction error:", e)
                set_status(f"Error: {e}")
        return (pcm,)

    ani = FuncAnimation(fig, update, interval=UPDATE_MS,
                        blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        ser.close()
        print("Serial closed.")


if __name__ == "__main__":
    main()

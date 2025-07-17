import os
import csv
from datetime import datetime
import math

# Full and short camera names
CAMERAS = {
    "front_center": "fc",
    "front_left": "fl",
    "front_right": "fr",
    "rear_center": "rc",
    "rear_left": "rl",
    "rear_right": "rr"
}

def parse_timestamp(ts_str):
    dt_part, us_part = ts_str.split('_')
    dt = datetime.strptime(dt_part, "%Y-%m-%d-%H-%M-%S")
    return dt.timestamp() + int(us_part) / 1e6

def analyze_camera(folder, full_cam):
    timing_file = os.path.join(folder, f"{full_cam}_frame-timing.csv")
    with open(timing_file, newline='') as csvfile:
        reader = csv.reader(csvfile)
        times = [parse_timestamp(row[1]) for row in reader]

    num_frames = len(times)
    duration = times[-1] - times[0] if num_frames > 1 else 0.0
    fps = num_frames / duration if duration > 0 else 0.0

    ideal_count = math.floor(duration * 3)

    return {
        "Camera": full_cam,
        "Duration (s)": round(duration, 2),
        "Avg FPS": round(fps, 2),
        "Missing/Ideal": f"{ideal_count - num_frames}/{ideal_count}"
    }

def main(folder):
    rows = []
    for full_cam in CAMERAS.keys():
        info = analyze_camera(folder, full_cam)
        rows.append(info)

    # Pretty print
    headers = ["Camera", "Duration (s)", "Avg FPS", "Missing/Ideal"]
    print(f'Run: {folder.split("/")[-1]}')
    print(f"{headers[0]:<15} {headers[1]:<15} {headers[2]:<10} {headers[3]:<15}")
    print("-" * 60)
    for row in rows:
        print(f"{row['Camera']:<15} {row['Duration (s)']:<15} {row['Avg FPS']:<10} {row['Missing/Ideal']:<15}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", help="Path to folder containing timing and video files")
    args = parser.parse_args()

    main(args.folder)

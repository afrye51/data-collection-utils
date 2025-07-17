import os
import csv
import subprocess
from datetime import datetime
import argparse
import math
import numpy as np

CAMERAS = {
    "front_center": "fc",
    "front_left": "fl",
    "front_right": "fr",
    "rear_center": "rc",
    "rear_left": "rl",
    "rear_right": "rr"
}

def find_closest_frame(timing_path, target_time):
    with open(timing_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        times = [(int(row[0]), row[1]) for row in reader]

    base_time = parse_timestamp(times[0][1])
    target_abs_time = base_time + target_time

    closest_frame = min(times, key=lambda row: abs(parse_timestamp(row[1]) - target_abs_time))

    return closest_frame

def parse_timestamp(ts_str):
    try:
        dt_part, us_part = ts_str.split('_')
        dt = datetime.strptime(dt_part, "%Y-%m-%d-%H-%M-%S")
    except ValueError:
        return np.inf
    return dt.timestamp() + int(us_part) / 1e6

def extract_frame(video_path, frame_num, output_path):
    cmd = [
        'ffmpeg', '-hide_banner', '-loglevel', 'error',
        '-i', video_path,
        '-vf', f'select=eq(n\\,{frame_num})',
        '-vframes', '1',
        output_path
    ]
    subprocess.run(cmd, check=True)

def process_all(folder_path, seconds_since_start):
    seconds_rounded = int(math.floor(seconds_since_start))

    for full_cam, short_cam in CAMERAS.items():
        timing_file = os.path.join(folder_path, f"{full_cam}_frame-timing.csv")
        video_file = os.path.join(folder_path, f"{full_cam}_video.avi")

        frame_num, _ = find_closest_frame(timing_file, seconds_since_start)
        output_name = f"{short_cam}_{seconds_rounded}.png"
        output_path = os.path.join(folder_path, output_name)

        print(f"Extracting {full_cam}: frame {frame_num} -> {output_name}")
        extract_frame(video_file, frame_num, output_path)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("seconds", type=float, help="Seconds since start of video")
    parser.add_argument("folder", help="Path to folder containing timing and video files")
    args = parser.parse_args()

    process_all(args.folder, args.seconds)

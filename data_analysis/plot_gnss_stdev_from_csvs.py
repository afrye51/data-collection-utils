import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse

def load_gps_data(gps_csv):
    gps = pd.read_csv(gps_csv)
    gps['time'] = gps['header.stamp.sec'] + gps['header.stamp.nanosec'] * 1e-9
    gps['2d_stdev'] = gps['err_horz'] / 2.0
    return gps[['time', '2d_stdev']]

def load_inspvax_data(inspvax_csv):
    inspvax = pd.read_csv(inspvax_csv)
    inspvax['time'] = inspvax['header.stamp.sec'] + inspvax['header.stamp.nanosec'] * 1e-9
    inspvax['2d_stdev'] = np.sqrt(inspvax['latitude_stdev']**2 + inspvax['longitude_stdev']**2)
    return inspvax[['time', '2d_stdev']]

def plot_data(gps, inspvax, run_name):
    plt.figure(figsize=(10, 6))

    # Align time to start at 0
    t0 = min(gps['time'].min(), inspvax['time'].min())
    plt.plot(inspvax['time'] - t0, inspvax['2d_stdev'], label='fused', color='navy')
    plt.plot(gps['time'] - t0, gps['2d_stdev'], label='gnss', color='red')

    plt.xlabel('time (sec)')
    plt.ylabel('2d-stdev (m)')
    plt.ylim([0, 3]) # Set Y axis limit to 3m to standardize plots
    plt.title(f'2D standard deviation vs. time for {run_name}')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f'{run_name}_gnss_stdev.png')
    plt.show()

def analyze_data(gps, inspvax, run_name):
    inspvax_stdev_mean = np.mean(inspvax['2d_stdev'])
    gps_stdev_mean = np.mean(gps['2d_stdev'])
    print(f'BestPos solution median stdev: {int(100*gps_stdev_mean)} cm. Fused solution median stdev: {int(100*inspvax_stdev_mean)} cm')
    
    inspvax_stdev_median = np.median(inspvax['2d_stdev'])
    gps_stdev_median = np.median(gps['2d_stdev'])
    print(f'BestPos solution median stdev: {int(100*gps_stdev_median)} cm. Fused solution median stdev: {int(100*inspvax_stdev_median)} cm')

def main(prefix):
    gps_csv = f"{prefix}_gps.csv"
    inspvax_csv = f"{prefix}_inspvax.csv"

    gps = load_gps_data(gps_csv)
    inspvax = load_inspvax_data(inspvax_csv)

    plot_data(gps, inspvax, prefix)
    analyze_data(gps, inspvax, prefix)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("prefix", help="Prefix for input files, e.g., 'run1' for 'run1_gps.csv' and 'run1_inspvax.csv'")
    args = parser.parse_args()

    main(args.prefix)

"""Plot tracking results saved by tracking_error_logger.py.

Usage:
    python3 scripts/plot_tracking_results.py --csv /tmp/ros2_demo_tracking_log.csv
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Plot ROS 2 trajectory tracking results.')
    parser.add_argument('--csv', default='/tmp/ros2_demo_tracking_log.csv', help='Path to tracking CSV file.')
    parser.add_argument('--output-dir', default='results', help='Directory for output figures.')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    csv_path = Path(args.csv).expanduser()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if not csv_path.exists():
        raise FileNotFoundError(f'CSV file not found: {csv_path}')

    data = pd.read_csv(csv_path)
    if data.empty:
        raise ValueError(f'CSV file is empty: {csv_path}')

    plt.figure(figsize=(10, 5))
    plt.plot(data['time'], data['desired_q1'], label='Desired q1')
    plt.plot(data['time'], data['actual_q1'], label='Actual q1', linestyle='--')
    plt.plot(data['time'], data['desired_q2'], label='Desired q2')
    plt.plot(data['time'], data['actual_q2'], label='Actual q2', linestyle='--')
    plt.xlabel('Time [s]')
    plt.ylabel('Joint position [rad]')
    plt.title('2-DOF Planar Robot Trajectory Tracking')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    trajectory_plot = output_dir / 'desired_vs_actual.png'
    plt.savefig(trajectory_plot, dpi=200)

    plt.figure(figsize=(10, 4))
    plt.plot(data['time'], data['error_q1'], label='q1 error')
    plt.plot(data['time'], data['error_q2'], label='q2 error')
    plt.plot(data['time'], data['error_norm'], label='error norm', linewidth=2)
    plt.xlabel('Time [s]')
    plt.ylabel('Tracking error [rad]')
    plt.title('Joint Tracking Error Under Disturbance')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    error_plot = output_dir / 'tracking_error.png'
    plt.savefig(error_plot, dpi=200)

    rmse_q1 = (data['error_q1'] ** 2).mean() ** 0.5
    rmse_q2 = (data['error_q2'] ** 2).mean() ** 0.5
    mean_norm = data['error_norm'].mean()

    print(f'Saved: {trajectory_plot}')
    print(f'Saved: {error_plot}')
    print(f'RMSE q1: {rmse_q1:.5f} rad')
    print(f'RMSE q2: {rmse_q2:.5f} rad')
    print(f'Mean error norm: {mean_norm:.5f} rad')


if __name__ == '__main__':
    main()

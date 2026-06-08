"""Plot the demonstration trajectory included in config/demo_trajectory.csv."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
CSV_PATH = ROOT / 'config' / 'demo_trajectory.csv'
OUTPUT_DIR = ROOT / 'results'
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def main() -> None:
    data = pd.read_csv(CSV_PATH)

    plt.figure(figsize=(10, 4))
    plt.plot(data['time'], data['q1'], label='q1 demonstration')
    plt.plot(data['time'], data['q2'], label='q2 demonstration')
    plt.xlabel('Time [s]')
    plt.ylabel('Joint position [rad]')
    plt.title('Demonstration trajectory for 2-DOF planar robot')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    output_path = OUTPUT_DIR / 'demo_trajectory.png'
    plt.savefig(output_path, dpi=200)
    print(f'Saved: {output_path}')


if __name__ == '__main__':
    main()

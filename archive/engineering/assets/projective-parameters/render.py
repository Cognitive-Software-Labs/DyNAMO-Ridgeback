"""Plot preserved September 2026 table values; this does not rerun a benchmark."""
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    here = Path(__file__).resolve().parent
    record = here.parents[1] / 'projective_parameter_sensitivity.md'
    section = record.read_text().split('## Recipe comparison', 1)[1].split('![', 1)[0]
    rows = []
    for line in section.splitlines():
        if line.startswith('| `'):
            cells = [cell.strip().replace('*', '') for cell in line.strip('|').split('|')]
            rows.append((float(cells[1]), float(cells[2])))
    if len(rows) != 3:
        raise ValueError('Expected the three retained recipe comparisons')
    fig, ax = plt.subplots(figsize=(8, 3.8), constrained_layout=True)
    labels = ['Nearest mode\nband 0.35 m', 'Nearest mode\nband 0.75 m', 'Otsu']
    for i, metric in enumerate(('MAE', 'p95')):
        bars = ax.bar([n + (i - .5) * .32 for n in range(3)],
                      [row[i] for row in rows], .32, label=metric,
                      color=('#237a91', '#b64c6c')[i])
        ax.bar_label(bars, fmt='%.4f', fontsize=9, padding=3)
    ax.set_xticks(range(3), labels)
    ax.set_ylabel('Error (m)')
    ax.set_ylim(0, .73)
    ax.spines[['top', 'right']].set_visible(False)
    ax.set_axisbelow(True)
    ax.grid(axis='y', alpha=.2)
    ax.legend(frameon=False)
    ax.set_title('Recorded simulation errors · September 7–8, 2026', loc='left', pad=16)
    fig.savefig(here / 'recipe-errors.png', dpi=160,
                metadata={'Software': 'DyNAMO historical evidence plot'})
    plt.close(fig)


if __name__ == '__main__':
    main()

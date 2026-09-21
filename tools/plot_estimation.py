#!/usr/bin/env python3
"""
Dibuja la evaluacion de la estimacion visual a partir de los CSV de estimation_evaluator.

Uso:
    python3 tools/plot_estimation.py hover.csv weave.csv [-o comparacion.png]

Un CSV por escenario; la leyenda usa el nombre del fichero. Se dibuja el ultimo tramo
de cada CSV (el que empieza en el ultimo reinicio del estimador). Tres paneles: error de
posicion, tamano estimado con su banda de +-2 sigma frente al real, y error de
velocidad. Sin -o, abre una ventana.
"""

import argparse
import csv
import os

import matplotlib.pyplot as plt


def load(path):
    """Lee el ultimo tramo de un CSV de estimation_evaluator como dict de columnas."""
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    if rows and 'segment' in rows[0]:
        last = rows[-1]['segment']
        rows = [r for r in rows if r['segment'] == last]
    return {k: [float(r[k]) for r in rows] for k in rows[0]} if rows else {}


def main():
    """Lee los CSV y dibuja los tres paneles."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('csv', nargs='+', help='CSV de estimation_evaluator')
    parser.add_argument('-o', '--output', help='Guardar la figura en este fichero')
    args = parser.parse_args()

    fig, (ax_pos, ax_size, ax_vel) = plt.subplots(3, 1, sharex=True, figsize=(9, 9))
    for path in args.csv:
        d = load(path)
        if not d:
            print(f'{path}: vacio, se omite')
            continue
        name = os.path.splitext(os.path.basename(path))[0]
        t = d['t']
        line, = ax_pos.plot(t, d['pos_err'], label=name)
        color = line.get_color()
        lo = [s - 2 * sg for s, sg in zip(d['est_size'], d['size_sigma'])]
        hi = [s + 2 * sg for s, sg in zip(d['est_size'], d['size_sigma'])]
        ax_size.plot(t, d['est_size'], color=color, label=name)
        ax_size.fill_between(t, lo, hi, color=color, alpha=0.15)
        ax_vel.plot(t, d['vel_err'], color=color, label=name)

        if d.get('true_size'):
            ax_size.axhline(d['true_size'][0], color='0.3', linestyle='--', linewidth=1)

    ax_pos.set_ylabel('error de posicion [m]')
    ax_size.set_ylabel('tamano [m] (+-2 sigma)')
    ax_vel.set_ylabel('error de velocidad [m/s]')
    ax_vel.set_xlabel('tiempo desde la primera estimacion [s]')
    ax_size.set_ylim(bottom=0)
    for ax in (ax_pos, ax_size, ax_vel):
        ax.grid(True, alpha=0.3)
    ax_pos.legend()
    ax_size.text(0.01, 0.92, 'discontinua = tamano real', transform=ax_size.transAxes,
                 fontsize=8, color='0.3')
    fig.tight_layout()

    if args.output:
        fig.savefig(args.output, dpi=120)
        print(f'Figura guardada en {args.output}')
    else:
        plt.show()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Plot przejazdu zapisanego przez traj_logger.py.

  python3 scripts/plot_run.py robot_logs/run_20261106_120000          # okno
  python3 scripts/plot_run.py robot_logs/run_20261106_120000 --save   # PNG do katalogu runa
  python3 scripts/plot_run.py <run> --map 12                          # konkretny snapshot mapy jako tlo

Okno 1: widok z gory - mapa (snapshot), SLAM, EKF (= GUI), tylna os z MPC, referencja,
        trasy z plannera (ostatnia pogrubiona), cele.
Okno 2: przebiegi w czasie - predkosci, kat skretu, odleglosc do celu, skoki pozycji, solver.
"""
import argparse
import glob
import os
import sys

import numpy as np
import matplotlib.pyplot as plt


def load_csv(run, name):
    path = os.path.join(run, name)
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return None
    try:
        data = np.genfromtxt(path, delimiter=',', names=True, dtype=None, encoding='utf-8')
    except ValueError:
        return None
    if data.size == 0:
        return None
    return np.atleast_1d(data)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run', help='katalog run_* z traj_logger')
    ap.add_argument('--save', action='store_true', help='zapisz PNG zamiast pokazywac')
    ap.add_argument('--map', type=int, default=-1, help='indeks snapshotu mapy (domyslnie ostatni)')
    args = ap.parse_args()
    run = args.run

    vo = load_csv(run, 'vo_pose.csv')
    odom = load_csv(run, 'odom.csv')
    mpc = load_csv(run, 'mpc.csv')
    cmd = load_csv(run, 'cmd_vel.csv')
    enc = load_csv(run, 'encoders.csv')
    goals = load_csv(run, 'goals.csv')
    plans = load_csv(run, 'plans.csv')
    maps = sorted(glob.glob(os.path.join(run, 'maps', 'map_*.npz')))

    starts = [d['t'][0] for d in (vo, odom, mpc, cmd) if d is not None]
    if not starts:
        sys.exit(f'Brak danych w {run}')
    t0 = min(starts)

    # ---------- widok z gory ----------
    fig1, ax = plt.subplots(figsize=(9, 9))
    if maps:
        m = np.load(maps[args.map])
        grid = m['grid']
        res = float(m['resolution'])
        ox, oy = float(m['origin_x']), float(m['origin_y'])
        h, w = grid.shape
        img = np.where(grid > 0, 1.0, 0.0)
        ax.imshow(img, origin='lower', cmap='Greys', vmin=0, vmax=1.5,
                  extent=[ox, ox + w * res, oy, oy + h * res], interpolation='nearest')
        ax.set_title(f'{os.path.basename(run)}  (mapa: {os.path.basename(maps[args.map])}, '
                     f't={float(m["t"]) - t0:.1f}s)')
    else:
        ax.set_title(os.path.basename(run))

    if plans is not None:
        ids = np.unique(plans['plan_id'])
        for pid in ids[:-1]:
            sel = plans['plan_id'] == pid
            ax.plot(plans['x'][sel], plans['y'][sel], color='tab:cyan', alpha=0.15, lw=1)
        sel = plans['plan_id'] == ids[-1]
        ax.plot(plans['x'][sel], plans['y'][sel], color='tab:cyan', lw=2, label=f'plan (ostatni z {len(ids)})')

    if vo is not None:
        ax.plot(vo['x'], vo['y'], color='tab:orange', lw=1, label='SLAM (kamera)')
    if odom is not None:
        ax.plot(odom['x'], odom['y'], color='tab:green', lw=1.5, label='EKF = GUI (kamera)')
        ax.plot(odom['x'][0], odom['y'][0], 'go', ms=8)
        step = max(1, len(odom) // 40)
        ax.quiver(odom['x'][::step], odom['y'][::step],
                  np.cos(odom['yaw'][::step]), np.sin(odom['yaw'][::step]),
                  color='tab:green', scale=30, width=0.003, alpha=0.6)
    if mpc is not None:
        ax.plot(mpc['x_P'], mpc['y_P'], color='tab:purple', lw=1.5, label='MPC tylna os (P)')
        ax.plot(mpc['ref_x'], mpc['ref_y'], '.', color='tab:red', ms=2, label='MPC referencja[0]')
    if goals is not None:
        g = goals[goals['event'] == 'goal']
        if g.size:
            ax.plot(g['x'].astype(float), g['y'].astype(float), 'r*', ms=15, label='cel')

    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=8)

    # ---------- przebiegi w czasie ----------
    fig2, axs = plt.subplots(5, 1, figsize=(11, 12), sharex=True)

    a = axs[0]
    if cmd is not None:
        a.plot(cmd['t'] - t0, cmd['v'], label='cmd v')
    if mpc is not None:
        a.plot(mpc['t'] - t0, mpc['v_target'], '--', label='v_target')
    if odom is not None and len(odom) > 1:
        dt = np.diff(odom['t'])
        ds = np.hypot(np.diff(odom['x']), np.diff(odom['y']))
        ok = dt > 1e-3
        a.plot(odom['t'][1:][ok] - t0, ds[ok] / dt[ok], alpha=0.5, label='|v| z EKF')
    a.set_ylabel('v [m/s]')
    a.legend(fontsize=8)
    a.grid(alpha=0.3)

    a = axs[1]
    if mpc is not None:
        a.plot(mpc['t'] - t0, np.degrees(mpc['psi_cmd']), label='psi cmd')
        a.plot(mpc['t'] - t0, np.degrees(np.unwrap(mpc['yaw'])), label='yaw')
        a.plot(mpc['t'] - t0, np.degrees(np.unwrap(mpc['ref_yaw'])), '--', label='ref yaw')
    a.set_ylabel('[deg]')
    a.legend(fontsize=8)
    a.grid(alpha=0.3)

    a = axs[2]
    if mpc is not None:
        a.plot(mpc['t'] - t0, mpc['goal_dist'], label='do celu')
        a.plot(mpc['t'] - t0, mpc['rem_dist'], label='do konca sciezki')
        a.plot(mpc['t'] - t0, np.hypot(mpc['x_P'] - mpc['ref_x'], mpc['y_P'] - mpc['ref_y']),
               label='blad sledzenia')
    a.set_ylabel('[m]')
    a.legend(fontsize=8)
    a.grid(alpha=0.3)

    a = axs[3]
    # skoki pozycji SLAMa - kazdy pik to podejrzany moment
    if vo is not None and len(vo) > 1:
        dt = np.diff(vo['t'])
        ds = np.hypot(np.diff(vo['x']), np.diff(vo['y']))
        ok = dt > 1e-3
        a.plot(vo['t'][1:][ok] - t0, ds[ok] / dt[ok], label='|v| SLAM')
    if enc is not None and len(enc) > 1:
        dt = np.diff(enc['t'])
        ok = dt > 1e-3
        a2 = a.twinx()
        a2.plot(enc['t'][1:][ok] - t0, np.diff(enc['left'])[ok] / dt[ok], 'k', alpha=0.3, lw=0.8)
        a2.plot(enc['t'][1:][ok] - t0, np.diff(enc['right'])[ok] / dt[ok], 'k--', alpha=0.3, lw=0.8)
        a2.set_ylabel('enkodery [tick/s]')
    a.set_ylabel('[m/s]')
    a.legend(fontsize=8, loc='upper left')
    a.grid(alpha=0.3)

    a = axs[4]
    if mpc is not None:
        a.plot(mpc['t'] - t0, mpc['solve_ms'], label='solver MPC')
    if vo is not None:
        # vo_pose ma stempel czasu zdjecia, wiec to jest opoznienie kamera -> poza SLAM
        a.plot(vo['t'] - t0, (vo['t'] - vo['t_msg']) * 1e3, '.-', label='opoznienie SLAM (kamera -> poza)')
    a.set_ylabel('[ms]')
    a.set_xlabel('t [s]')
    a.legend(fontsize=8)
    a.grid(alpha=0.3)

    for a in axs:
        if goals is not None:
            for row in goals:
                c = 'r' if row['event'] == 'goal' else 'k'
                a.axvline(row['t'] - t0, color=c, alpha=0.3, lw=0.8)

    fig2.tight_layout()

    if args.save:
        fig1.savefig(os.path.join(run, 'xy.png'), dpi=150)
        fig2.savefig(os.path.join(run, 'time.png'), dpi=150)
        print(f'Zapisano {run}/xy.png i {run}/time.png')
    else:
        plt.show()


if __name__ == '__main__':
    main()

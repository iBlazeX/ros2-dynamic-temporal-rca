#!/usr/bin/env python3
"""Summarises captured rca_tui frames as a sequence of observable state changes.

Reads the frames written by scripts/tui_demo.sh and prints one line per change
of (system status, backend label, probable root cause, anomalous nodes, starved
nodes). Also asserts that no evaluation ground truth ever appeared on screen.
"""
import glob
import os
import re
import sys

FORBIDDEN = ('ground_truth', 'true_root', 'accepted_roots', 'observable_chain',
             'physical_chain', 'injected fault')


def state(txt):
    m = re.search(r'SYSTEM (\w+)', txt)
    sysst = m.group(1) if m else '?'
    b = re.search(r'backend (\w+)', txt)
    backend = b.group(1) if b else '?'
    roots = re.findall(r'\[\*\] (\w+)', txt)
    starved = re.findall(r'\[X\] (\w+)', txt)
    anom = sorted(set(re.findall(r'\[!\] (\w+)', txt)))
    return sysst, backend, roots[:1], anom, sorted(set(starved))


def main(out_dir):
    frames = sorted(glob.glob(os.path.join(out_dir, 'frame_*.txt')))
    if not frames:
        print('no frames captured')
        return 1
    leaks = []
    prev = None
    changes = 0
    for f in frames:
        txt = open(f, errors='ignore').read()
        for word in FORBIDDEN:
            if word in txt:
                leaks.append(f'{os.path.basename(f)}: {word}')
        s = state(txt)
        if s != prev:
            print(f'{os.path.basename(f)}: system={s[0]} backend={s[1]} root={s[2]} '
                  f'anomalous={s[3]} starved={s[4]}')
            prev = s
            changes += 1
    print(f'\nframes={len(frames)} distinct_states={changes}')
    print('ground-truth leak check:', 'FAIL ' + str(leaks) if leaks else 'PASS (nothing leaked)')
    return 1 if leaks else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else '.'))

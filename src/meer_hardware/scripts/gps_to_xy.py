#!/usr/bin/env python3
"""
gps_to_xy.py — convert competition GPS waypoints (lat,lon) into rover x,y meters
for simple_mission.py.

The rover has no working GPS/compass, so we fix the frame by HOW YOU PLACE IT:
  --face-first (default): put the rover at the base station POINTING STRAIGHT
                          AT WAYPOINT 1. Then x = toward waypoint 1, y = left.
  --heading DEG         : rover points to this compass heading (0 = North,
                          90 = East), e.g. read from a phone compass app.

Usage:
  python3 gps_to_xy.py BASE_LAT,BASE_LON WP1_LAT,WP1_LON [WP2_LAT,WP2_LON ...]
  python3 gps_to_xy.py 25.8915,89.4342 25.89146247,89.43447568 25.89139628,89.43472287
"""

import argparse
import math
import sys

R = 6378137.0  # earth radius (m)


def to_en(lat0, lon0, lat, lon):
    """Local East/North meters of (lat,lon) relative to (lat0,lon0)."""
    east = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
    north = math.radians(lat - lat0) * R
    return east, north


def parse(s):
    try:
        a, b = s.split(',')
        return float(a), float(b)
    except ValueError:
        sys.exit(f'Bad point "{s}" — use lat,lon  e.g. 25.89146247,89.43447568')


def main():
    p = argparse.ArgumentParser(description='GPS waypoints -> rover x,y')
    p.add_argument('base', help='base station lat,lon (where the rover starts)')
    p.add_argument('waypoints', nargs='+', help='waypoint lat,lon ...')
    p.add_argument('--heading', type=float, default=None,
                   help='compass heading the rover faces at start (deg, 0=N, 90=E). '
                        'Default: rover faces waypoint 1.')
    a = p.parse_args()

    lat0, lon0 = parse(a.base)
    wps = [parse(w) for w in a.waypoints]
    en = [to_en(lat0, lon0, la, lo) for la, lo in wps]

    if a.heading is None:
        e1, n1 = en[0]
        heading = math.degrees(math.atan2(e1, n1)) % 360.0
        how = 'point the rover STRAIGHT AT WAYPOINT 1'
    else:
        heading = a.heading % 360.0
        how = f'point the rover to compass heading {heading:.0f} deg'

    h = math.radians(heading)
    # forward unit = (sin h, cos h) in (E,N); left unit = (-cos h, sin h)
    out = []
    print(f'\nRover start: {how} (= {heading:.0f} deg from North)\n')
    for i, (e, n) in enumerate(en, 1):
        x = e * math.sin(h) + n * math.cos(h)
        y = -e * math.cos(h) + n * math.sin(h)
        out.append((x, y))
        print(f'  WP{i}: x = {x:7.2f} m (forward)   y = {y:7.2f} m (left+)   '
              f'distance = {math.hypot(x, y):6.2f} m')
    for i in range(1, len(en)):
        d = math.hypot(en[i][0] - en[i - 1][0], en[i][1] - en[i - 1][1])
        print(f'  WP{i} -> WP{i + 1}: {d:.2f} m')

    args = ' '.join(f'{x:.2f},{y:.2f}' for x, y in out)
    print('\nRun mission:\n')
    print(f'  cd ~/ogrodoot_meer && source install/setup.bash && '
          f'python3 src/meer_hardware/scripts/simple_mission.py {args}\n')


if __name__ == '__main__':
    main()

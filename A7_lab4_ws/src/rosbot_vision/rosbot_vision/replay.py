"""Run the block detection on what vision_node saved for a detect_blocks call. Needs no ROS and no robot.

    ros2 run rosbot_vision replay_capture                      # newest capture in ~/.ros/vision_captures
    ros2 run rosbot_vision replay_capture <dir or *_meta.json>
    python -m rosbot_vision.replay <dir or *_meta.json> --probe 320,240 --masks

Prints where each color was found (or why not) with the saved camera pose, and writes <capture>_replay.png.
--probe u,v prints the HSV at that pixel (for tuning hsv_<color>), --masks also writes one mask image per color.
"""
import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

from rosbot_vision.findBlock import HSV_RANGES, colorMask, maskPixels
from rosbot_vision.locateBlock import locate_block


def newest_meta(path):
    path = os.path.expanduser(path)
    if os.path.isdir(path):
        metas = sorted(glob.glob(os.path.join(path, '*_meta.json')))
        if not metas:
            sys.exit(f'No *_meta.json captures in {path}')
        return metas[-1]
    return path


def replay(metaPath, probes=(), writeMasks=False):
    stem = metaPath[:-len('_meta.json')]
    with open(metaPath) as f:
        meta = json.load(f)
    bgr = cv2.imread(stem + '_color.png')
    if bgr is None:
        sys.exit(f'Cannot read {stem}_color.png')
    depth = cv2.imread(stem + '_depth.png', cv2.IMREAD_UNCHANGED)
    depthM = np.zeros(bgr.shape[:2]) if depth is None else depth.astype(float) * 0.001

    rotation = np.array(meta['rotation'])
    translation = np.array(meta['translation'])
    plane = (np.array(meta['plane_normal']), meta['plane_distance'])
    ranges = {color: [(tuple(v[i]), tuple(v[i + 1])) for i in range(0, len(v), 2)] for color, v in meta['hsv'].items()}

    print(f'{os.path.basename(stem)}: camera at {np.round(translation, 3).tolist()} from the {meta["pose_source"]}, '
          f'floor from {meta["plane_source"]}, depth {"missing" if depth is None else "present"}')
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    for u, v in probes:
        patch = hsv[max(0, v - 2):v + 3, max(0, u - 2):u + 3].reshape(-1, 3)
        print(f'  pixel ({u}, {v}): HSV median {np.median(patch, axis=0).astype(int).tolist()}, '
              f'depth {depthM[v, u]:.3f} m')

    debug = bgr.copy()
    for color in HSV_RANGES:
        reasons = []
        det = locate_block(bgr, depthM, color, meta['intrinsics'], plane, rotation, translation,
                           meta['block_height'], meta['min_area'], meta['max_block_height'],
                           ranges.get(color), reasons, meta['require_raised'])
        if writeMasks:
            cv2.imwrite(f'{stem}_mask_{color}.png', colorMask(bgr, color, ranges.get(color)))
        if det is None:
            print(f'  {color}: not found ({maskPixels(bgr, color, ranges.get(color))} px match; {"; ".join(reasons)})')
            continue
        u, v = det['pixel']
        print(f'  {color}: arm ({det["x"]:.3f}, {det["y"]:.3f}) m, yaw {det["yaw_deg"]:.1f} deg, footprint '
              f'{1000 * det["size"][0]:.0f}x{1000 * det["size"][1]:.0f} mm, pixel ({u:.0f}, {v:.0f}); '
              f'on the robot it reported {meta["reported"].get(color)}')
        cv2.circle(debug, (int(u), int(v)), 6, (255, 255, 255), 2)
        cv2.putText(debug, f'{color} ({det["x"]:.3f}, {det["y"]:.3f})', (int(u) + 8, int(v) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.imwrite(stem + '_replay.png', debug)
    print(f'  wrote {stem}_replay.png')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('capture', nargs='?', default='~/.ros/vision_captures',
                        help='capture directory (newest capture is used) or one *_meta.json')
    parser.add_argument('--probe', action='append', default=[], metavar='U,V', help='print the HSV at this pixel')
    parser.add_argument('--masks', action='store_true', help='write one mask image per color')
    args = parser.parse_args()
    probes = [tuple(int(v) for v in p.split(',')) for p in args.probe]
    replay(newest_meta(args.capture), probes, args.masks)


if __name__ == '__main__':
    main()

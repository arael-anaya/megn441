"""Pure numpy occupancy-grid updater (no ROS) so it can be tested off-robot.

State grid: int8, row 0 = lowest y (same layout as nav_msgs/OccupancyGrid),
values 0 = free, 1 = occupied, -1 = unknown.

A cell only changes once the lidar has *continuously* disagreed with it for
`persist_time` seconds (default 4 s), so people walking through are ignored.
"""
import math
import os

import numpy as np

FREE, OCC, UNKNOWN = 0, 1, -1
_NONE = -2  # "no pending change"


# ---------------------------------------------------------------- map file I/O
def _read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:  # magic, width, height, maxval
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while data[pos:pos + 1] != b'\n':
                pos += 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos].decode())
    pos += 1  # single whitespace after maxval
    if tokens[0] != 'P5' or int(tokens[3]) > 255:
        raise ValueError('only 8-bit binary PGM (P5) maps are supported')
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data, dtype=np.uint8, count=w * h, offset=pos).reshape(h, w)


def _write_pgm(path, img):
    h, w = img.shape
    with open(path, 'wb') as f:
        f.write(b'P5\n# written by nav_manager map_updater\n%d %d\n255\n' % (w, h))
        f.write(np.ascontiguousarray(img, dtype=np.uint8).tobytes())


def _read_yaml(path):
    out = {}
    with open(path) as f:
        for line in f:
            line = line.split('#')[0].strip()
            if ':' in line:
                k, v = line.split(':', 1)
                out[k.strip()] = v.strip()
    return out


def load_map(yaml_path):
    """Returns (state int8 grid, resolution, (origin_x, origin_y), yaml dict)."""
    meta = _read_yaml(yaml_path)
    img = _read_pgm(os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta['image']))
    negate = int(meta.get('negate', 0))
    occ_t = float(meta.get('occupied_thresh', 0.65))
    free_t = float(meta.get('free_thresh', 0.25))
    p = img / 255.0 if negate else (255 - img.astype(np.float64)) / 255.0
    state = np.full(img.shape, UNKNOWN, dtype=np.int8)
    state[p > occ_t] = OCC
    state[p < free_t] = FREE
    state = np.flipud(state).copy()  # image row 0 is the top (max y)
    origin = [float(v) for v in meta['origin'].strip('[]').split(',')]
    return state, float(meta['resolution']), (origin[0], origin[1]), meta


def save_map(state, resolution, origin, yaml_path, meta=None):
    """Writes <yaml_path> and the PGM next to it. Pixel values: occ 0, free 254, unknown 128."""
    img = np.full(state.shape, 128, dtype=np.uint8)
    img[state == OCC] = 0
    img[state == FREE] = 254
    base = os.path.splitext(os.path.abspath(yaml_path))[0]
    _write_pgm(base + '.pgm', np.flipud(img))
    meta = meta or {}
    with open(yaml_path, 'w') as f:
        f.write('image: %s\n' % (os.path.basename(base) + '.pgm'))
        f.write('mode: %s\n' % meta.get('mode', 'trinary'))
        f.write('resolution: %s\n' % resolution)
        f.write('origin: [%s, %s, 0]\n' % (origin[0], origin[1]))
        f.write('negate: 0\n')
        f.write('occupied_thresh: %s\n' % meta.get('occupied_thresh', 0.65))
        f.write('free_thresh: %s\n' % meta.get('free_thresh', 0.25))


# --------------------------------------------------------------- map updating
class PersistentMapUpdater:
    def __init__(self, state, resolution, origin, persist_time=4.0, pending_timeout=1.0,
                 min_range=0.3, max_range=5.0, clear_margin=0.10):
        self.state = state
        self.res = resolution
        self.ox, self.oy = origin
        self.h, self.w = state.shape
        self.persist_time = persist_time
        self.pending_timeout = pending_timeout  # a pending change is dropped if unseen this long
        self.min_range = min_range
        self.max_range = max_range
        self.clear_margin = clear_margin  # never free the last bit of a ray: pose error would eat walls
        n = self.h * self.w
        self._flat = state.reshape(-1)  # view
        self._pend = np.full(n, _NONE, dtype=np.int8)
        self._since = np.zeros(n)
        self._seen = np.zeros(n)
        self.dirty = False

    def integrate(self, ranges, angles, sx, sy, syaw, t):
        """One scan. angles are in the sensor frame; (sx, sy, syaw) is the sensor pose in the map
        frame; t is a time in seconds. Returns the number of cells that changed."""
        ranges = np.asarray(ranges, dtype=np.float64)
        angles = np.asarray(angles, dtype=np.float64) + syaw
        ok = np.isfinite(ranges) & (ranges >= self.min_range)
        ranges, angles = ranges[ok], angles[ok]
        if ranges.size == 0:
            return 0
        hit = ranges <= self.max_range
        free_len = np.minimum(ranges, self.max_range) - self.clear_margin
        cos, sin = np.cos(angles), np.sin(angles)

        step = self.res * 0.5
        ks = np.arange(int(self.max_range / step) + 1) * step
        along = ks[None, :] <= free_len[:, None]
        px = (sx + ks[None, :] * cos[:, None])[along]
        py = (sy + ks[None, :] * sin[:, None])[along]
        free_idx = self._cells(px, py)

        hx = sx + ranges[hit] * cos[hit]
        hy = sy + ranges[hit] * sin[hit]
        hit_idx = self._cells(hx, hy)

        free_idx = np.setdiff1d(free_idx, hit_idx, assume_unique=True)
        idx = np.concatenate([free_idx, hit_idx])
        val = np.concatenate([np.full(free_idx.size, FREE, np.int8),
                              np.full(hit_idx.size, OCC, np.int8)])
        return self._apply(idx, val, t)

    def _cells(self, x, y):
        ix = np.floor((x - self.ox) / self.res).astype(np.int64)
        iy = np.floor((y - self.oy) / self.res).astype(np.int64)
        ok = (ix >= 0) & (ix < self.w) & (iy >= 0) & (iy < self.h)
        return np.unique(iy[ok] * self.w + ix[ok])

    def _apply(self, idx, val, t):
        cur = self._flat[idx]
        agree = cur == val
        self._pend[idx[agree]] = _NONE  # the map was right after all: forget it

        d_idx, d_val = idx[~agree], val[~agree]
        if d_idx.size == 0:
            return 0
        stale = (t - self._seen[d_idx]) > self.pending_timeout
        restart = (self._pend[d_idx] != d_val) | stale
        r_idx = d_idx[restart]
        self._pend[r_idx] = d_val[restart]
        self._since[r_idx] = t
        self._seen[d_idx] = t

        commit = (t - self._since[d_idx]) >= self.persist_time
        c_idx = d_idx[commit]
        if c_idx.size:
            self._flat[c_idx] = d_val[commit]
            self._pend[c_idx] = _NONE
            self.dirty = True
        return int(c_idx.size)

    def to_occupancy_data(self):
        """int8 list for nav_msgs/OccupancyGrid.data (-1 unknown, 0 free, 100 occupied)."""
        out = np.full(self._flat.shape, -1, dtype=np.int8)
        out[self._flat == FREE] = 0
        out[self._flat == OCC] = 100
        return out


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

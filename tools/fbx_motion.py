"""Reads the motion out of a binary FBX file endorphin exported, as CSV, so its run can
be compared with NeuroMotion's (`dump_scene_motion`) and measured for
`src/physics/reference.rs`.

    python3 -I tools/fbx_motion.py endorphin-export.fbx joints.csv

Writes one row per frame and joint: `frame,joint,parent,x,y,z,qx,qy,qz,qw`, the joint's
world position in metres and world rotation, in the file's own axes (endorphin's
scene axes, Y up: the same as NeuroMotion's). The FBX's key times are written as frame
numbers at its frame rate; where the character starts animated, endorphin's first key
is its bind pose and scene frame f is key f + 1 (docs/endorphin-formats.md, "Exported
motion").

Handles binary FBX 6 (FBX SDK 2005, as endorphin 2.5 writes it) with `Takes` curves,
which is all endorphin's export needs. Uses only the standard library. Treat the file
as untrusted data: it is parsed, never executed. Keep endorphin's files, and these
CSVs, out of the repository; only numbers measured from them belong in it.
"""

import math
import struct
import sys
import zlib

TICKS_PER_SECOND = 46186158000


def parse(data):
    """The file's top-level nodes as (name, properties, children)."""
    if data[:21] != b"Kaydara FBX Binary  \x00":
        raise ValueError("not a binary FBX file")
    version = struct.unpack_from("<I", data, 23)[0]
    wide = version >= 7500  # 64-bit record headers

    def node(off):
        if wide:
            end, count, _ = struct.unpack_from("<QQQ", data, off)
            off += 24
        else:
            end, count, _ = struct.unpack_from("<III", data, off)
            off += 12
        length = data[off]
        off += 1
        if end == 0:
            return None, off
        name = data[off:off + length].decode("latin1")
        off += length
        props = []
        for _ in range(count):
            kind = chr(data[off])
            off += 1
            if kind in "YCIFDL":
                fmt = {"Y": "<h", "C": "<?", "I": "<i", "F": "<f", "D": "<d", "L": "<q"}[kind]
                value = struct.unpack_from(fmt, data, off)[0]
                off += struct.calcsize(fmt)
                # Keep booleans apart from integers: key lists mix both.
                props.append((kind, value))
            elif kind in "SR":
                n = struct.unpack_from("<I", data, off)[0]
                off += 4
                raw = data[off:off + n]
                off += n
                props.append((kind, raw.decode("latin1") if kind == "S" else raw))
            elif kind in "fdlib":
                n, encoding, size = struct.unpack_from("<III", data, off)
                off += 12
                raw = data[off:off + size]
                off += size
                if encoding == 1:
                    raw = zlib.decompress(raw)
                item = {"f": "f", "d": "d", "l": "q", "i": "i", "b": "?"}[kind]
                props.append((kind, list(struct.unpack("<%d%s" % (n, item), raw))))
            else:
                raise ValueError("unknown property type %r at byte %d" % (kind, off))
        children = []
        while off < end:
            child, off = node(off)
            if child is None:
                break
            children.append(child)
        return (name, props, children), end

    nodes, off = [], 27
    while off < len(data):
        n, off = node(off)
        if n is None:
            break
        nodes.append(n)
    return version, nodes


def child(n, name):
    return next((c for c in n[2] if c[0] == name), None)


def children(n, name):
    return [c for c in n[2] if c[0] == name]


def value(prop):
    return prop[1]


def short(name):
    """'LThigh\\x00\\x01Model' -> 'LThigh'."""
    return name.split("\x00")[0]


def curve(channel):
    """A curve's keys as [(ticks, value)], or its default as one key."""
    keys = child(channel, "Key")
    if keys is None:
        default = child(channel, "Default")
        return [(0, float(value(default[1][0])))] if default else []
    out, props, i = [], keys[1], 0
    while i < len(props):
        # Each key: time (L), value, then interpolation flags (not L).
        out.append((value(props[i]), float(value(props[i + 1]))))
        i += 2
        while i < len(props) and props[i][0] != "L":
            i += 1
    return out


def sample(keys, t):
    if t <= keys[0][0]:
        return keys[0][1]
    for (t0, v0), (t1, v1) in zip(keys, keys[1:]):
        if t <= t1:
            return v1 if t1 == t0 else v0 + (v1 - v0) * (t - t0) / (t1 - t0)
    return keys[-1][1]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def local_matrix(t, r):
    """Translation then rotation, Euler XYZ in degrees (R = Rz Ry Rx)."""
    x, y, z = (math.radians(v) for v in r)
    cx, sx, cy, sy, cz, sz = math.cos(x), math.sin(x), math.cos(y), math.sin(y), math.cos(z), math.sin(z)
    rot = [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ]
    return [rot[0] + [t[0]], rot[1] + [t[1]], rot[2] + [t[2]], [0, 0, 0, 1]]


def quaternion(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1) * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, s / 4)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2
        return (s / 4, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][1] + m[1][0]) / s, s / 4, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, s / 4, (m[1][0] - m[0][1]) / s)


def motion(path):
    """Yields (frame, joint, parent, world 4x4 matrix in metres)."""
    version, nodes = parse(open(path, "rb").read())
    if version >= 7000:
        raise ValueError("FBX %d: only FBX 6 takes are read (endorphin writes 6000)" % version)
    top = {n[0]: n for n in nodes}
    rest = {}
    for model in children(top["Objects"], "Model"):
        kind = child(model, "Type")
        if kind is None or value(kind[1][0]) not in ("Root", "LimbNode"):
            continue  # cameras and the like
        props = {}
        for p in children(child(model, "Properties60"), "Property"):
            props[value(p[1][0])] = [value(x) for x in p[1][3:]]
        for name in ("PreRotation", "PostRotation"):
            if any(abs(v) > 1e-6 for v in props.get(name, [0, 0, 0])):
                raise ValueError("%s has a %s, not handled" % (short(value(model[1][0])), name))
        if props.get("RotationOrder", [0])[0] != 0:
            raise ValueError("rotation order other than XYZ, not handled")
        rest[short(value(model[1][0]))] = props
    parent = {}
    for c in children(top["Connections"], "Connect"):
        a, b = short(value(c[1][1])), short(value(c[1][2]))
        if a in rest and b in rest:
            parent[a] = b
    fps = float(value(child(child(top["Version5"], "Settings"), "FrameRate")[1][0]))
    take = children(top["Takes"], "Take")[0]
    start, end = (value(p) for p in child(take, "LocalTime")[1])
    curves = {}
    for model in children(take, "Model"):
        transform = child(model, "Channel")
        curves[short(value(model[1][0]))] = {
            value(c[1][0]) + value(axis[1][0]): curve(axis)
            for c in children(transform, "Channel")
            for axis in children(c, "Channel")
        }
    order = []

    def visit(j):
        if j not in order:
            if j in parent:
                visit(parent[j])
            order.append(j)

    for j in rest:
        visit(j)
    step = TICKS_PER_SECOND / fps
    for frame in range(round(start / step), round(end / step) + 1):
        t = frame * step
        world = {}
        for j in order:
            c = curves.get(j, {})

            def channel(name, default):
                keys = c.get(name)
                return sample(keys, t) if keys else default

            lt = rest[j].get("Lcl Translation", [0, 0, 0])
            lr = rest[j].get("Lcl Rotation", [0, 0, 0])
            local = local_matrix(
                [channel("T" + a, lt[i]) for i, a in enumerate("XYZ")],
                [channel("R" + a, lr[i]) for i, a in enumerate("XYZ")],
            )
            world[j] = mat_mul(world[parent[j]], local) if j in parent else local
            yield frame, j, parent.get(j, ""), world[j]


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    with open(sys.argv[2], "w") as out:
        out.write("frame,joint,parent,x,y,z,qx,qy,qz,qw\n")
        for frame, joint, parent, m in motion(sys.argv[1]):
            q = quaternion(m)
            out.write("%d,%s,%s,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f\n" % (
                frame, joint, parent, m[0][3] / 100, m[1][3] / 100, m[2][3] / 100, *q))


if __name__ == "__main__":
    main()

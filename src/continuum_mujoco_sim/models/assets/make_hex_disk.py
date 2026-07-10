import math
import struct
from pathlib import Path

R = 0.080   # 六边形外接圆半径，单位 m
H = 0.005   # 半厚度，单位 m

verts_bottom = []
verts_top = []

for i in range(6):
    a = i * math.pi / 3.0
    x = R * math.cos(a)
    y = R * math.sin(a)
    verts_bottom.append((x, y, -H))
    verts_top.append((x, y, H))

verts = verts_bottom + verts_top

faces = []

# bottom face
for i in range(1, 5):
    faces.append((0, i + 1, i))

# top face
for i in range(1, 5):
    faces.append((6, 6 + i, 6 + i + 1))

# side faces
for i in range(6):
    j = (i + 1) % 6

    b0 = i
    b1 = j
    t0 = i + 6
    t1 = j + 6

    faces.append((b0, b1, t1))
    faces.append((b0, t1, t0))


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def sub(a, b):
    return (
        a[0] - b[0],
        a[1] - b[1],
        a[2] - b[2],
    )


def normalize(v):
    n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def normal_of_triangle(v0, v1, v2):
    a = sub(v1, v0)
    b = sub(v2, v0)
    return normalize(cross(a, b))


out = Path("hex_disk.stl")

with out.open("wb") as f:
    # binary STL header: 80 bytes
    header = b"binary_hex_disk_for_mujoco"
    f.write(header[:80].ljust(80, b" "))

    # triangle count: uint32
    f.write(struct.pack("<I", len(faces)))

    for tri in faces:
        v0 = verts[tri[0]]
        v1 = verts[tri[1]]
        v2 = verts[tri[2]]

        n = normal_of_triangle(v0, v1, v2)

        f.write(struct.pack("<3f", *n))
        f.write(struct.pack("<3f", *v0))
        f.write(struct.pack("<3f", *v1))
        f.write(struct.pack("<3f", *v2))
        f.write(struct.pack("<H", 0))

print("generated binary STL:", out.resolve())
print("faces:", len(faces))

"""Write the raw GNSS fixes out as a TUM trajectory.

    python3 tools/gnss_to_tum.py <exported.txt> <out.tum>

So the unfiltered baseline can be scored by the SAME scorer as a fused trajectory,
through the same alignment and the same association tolerance. Scoring GNSS with one
tool and the filter with another produced a table on 2026-10-18 that compared two
alignment implementations and called it a comparison.

Orientation is identity: a position-only baseline has no attitude, and ate_tum.py
scores position.
"""
import sys

if len(sys.argv) < 3:
    sys.exit(__doc__)
n = 0
with open(sys.argv[2], "w") as g:
    for ln in open(sys.argv[1]):
        if not ln.startswith("GNSS "):
            continue
        p = ln.split()
        x, y, z = float(p[2]), float(p[3]), float(p[4])
        if x == 0.0 and y == 0.0 and z == 0.0:
            continue          # pre-origin placeholder, not a fix
        g.write("%s %.6f %.6f %.6f 0 0 0 1\n" % (p[1], x, y, z))
        n += 1
print("wrote %d fixes" % n)

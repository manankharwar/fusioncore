import sys, bisect
traj, times_src, out = sys.argv[1], sys.argv[2], sys.argv[3]
ts=[]
for ln in open(times_src):
    if ln.startswith('GNSS '): ts.append(float(ln.split()[1]))
ts.sort()
rows=[]
for ln in open(traj):
    p=ln.split()
    if len(p)<8: continue
    rows.append((float(p[0]), ln))
rows.sort()
rt=[r[0] for r in rows]
kept=0
with open(out,'w') as g:
    for t in ts:
        i=bisect.bisect_left(rt,t)
        best=None
        for j in (i-1,i):
            if 0<=j<len(rt):
                if best is None or abs(rt[j]-t)<abs(rt[best]-t): best=j
        if best is not None and abs(rt[best]-t)<0.05:
            g.write(rows[best][1]); kept+=1
print(kept)

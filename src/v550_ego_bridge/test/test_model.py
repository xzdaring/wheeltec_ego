# 验证独立的物理不变量：横移轮向、正逆解、轮速/轮加速度和最短角插值。
import math,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mecanum_model import wheels,chassis,limit,schedule,wrap
arm=.17709
assert wheels(0,1,0,arm)==[-1,1,1,-1]
for v in [(.1,.2,.3),(-.2,.1,-.5),(0,0,.6)]:
 assert all(abs(a-b)<1e-12 for a,b in zip(v,chassis(wheels(*v,arm),arm)))
old=[0.]*4
for i in range(100):
 cmd,new=limit(.4,-.3,.6,old,.05,arm,.3,.4)
 assert max(abs(w) for w in new)<=.3000001
 assert max(abs(a-b) for a,b in zip(old,new))<=.4*.05+1e-9
 old=new
poses,times=schedule([(0,0),(0,.2),(0,.4)],math.radians(179),math.radians(-179),arm,.3,.4,.6)
assert abs(poses[-1][2]-poses[0][2])<math.radians(3)
assert all(b>a for a,b in zip(times,times[1:]))
assert abs(wrap(poses[-1][2]-math.radians(-179)))<1e-9
print('PASS: mecanum inverse/forward, lateral motion, wheel bounds, shortest yaw and timing')

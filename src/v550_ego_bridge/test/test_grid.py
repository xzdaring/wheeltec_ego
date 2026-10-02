import sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import numpy as np
from grid_safety import Grid,inflate
from nav_msgs.msg import OccupancyGrid
m=OccupancyGrid();m.info.width=100;m.info.height=100;m.info.resolution=.05
raw=np.zeros((100,100),bool);raw[25:75,50]=True
m.data=inflate(raw,.05,.20,.25).ravel().tolist();g=Grid(m)
a=(1.,2.5);b=(4.,2.5);p=g.route(a,b)
assert not g.safe([a,b]);assert p and g.safe(p);print('PASS wall detour',p)
raw[:,50]=True;m.data=inflate(raw,.05,.20,.25).ravel().tolist();g=Grid(m);assert not g.route(a,b);print('PASS sealed wall rejected')
# 位于膨胀余量但未实体碰撞时可单调远离，不允许继续往墙里钻。
raw[:]=False;raw[:,50]=True;m.data=inflate(raw,.05,.20,.30).ravel().tolist();g=Grid(m)
start=next(g.center((x,50)) for x in range(40,50) if 0<g.cost((x,50))<100)
p=g.route(start,(1.,2.5));assert p and g.safe(p);assert not g.safe([start,(2.48,2.5)])
print('PASS inflation escape',start,p)

# float32是EGO内部坐标精度，回退折线转换后仍必须安全。
raw[:]=False;raw[25:75,50]=True;m.data=inflate(raw,.05,.20,.25).ravel().tolist();g=Grid(m)
p=g.route(a,b);assert g.safe([tuple(float(np.float32(v)) for v in q) for q in p])
print('PASS float32 route safety')

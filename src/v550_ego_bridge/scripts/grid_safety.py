"""全局与控制共用栅格规则：100不可进入，1..99膨胀余量，0可行驶。
地图来自在线SLAM和实时激光；未知区域不标为障碍，有限地图边界仍禁止越出。
"""
import math,heapq
import numpy as np
from scipy.ndimage import distance_transform_edt

def inflate(raw, resolution, physical, margin):
    # 栅格障碍占一整个方格，补半对角线，避免仅按格心距离低估障碍边缘。
    clear=distance_transform_edt(np.pad(raw,1,constant_values=True)==0)[1:-1,1:-1]*resolution-resolution*math.sqrt(2)/2
    out=np.zeros(raw.shape,dtype=np.int8)
    belt=(clear<=margin)&(clear>physical)
    out[belt]=np.clip(np.ceil(99*(margin-clear[belt])/(margin-physical)),1,99).astype(np.int8)
    out[clear<=physical]=100
    return out

class Grid:
    def __init__(self,msg):
        self.w=msg.info.width;self.h=msg.info.height;self.r=msg.info.resolution
        self.ox=msg.info.origin.position.x;self.oy=msg.info.origin.position.y
        self.a=np.array(msg.data,dtype=np.int16).reshape(self.h,self.w)
    def cell(self,x,y):return math.floor((x-self.ox)/self.r),math.floor((y-self.oy)/self.r)
    def center(self,c):return self.ox+(c[0]+.5)*self.r,self.oy+(c[1]+.5)*self.r
    def cost(self,c):
        x,y=c
        return int(self.a[y,x]) if 0<=x<self.w and 0<=y<self.h else 100
    def allowed(self,a,b):
        ca=self.cost(a);cb=self.cost(b)
        # 已进入余量区只可沿代价不增加方向退出来；禁止穿入实体/未知格。
        return cb<100 and (cb==0 if ca==0 else cb<=ca)
    def safe(self,points):
        if not points:return False
        prev=self.cell(*points[0])
        if self.cost(prev)>=100:return False
        for a,b in zip(points,points[1:]):
            steps=max(1,math.ceil(math.dist(a,b)/(self.r*.4)))
            for i in range(1,steps+1):
                c=self.cell(a[0]+(b[0]-a[0])*i/steps,a[1]+(b[1]-a[1])*i/steps)
                if not self.allowed(prev,c):return False
                # 对角线不能从两个障碍格之间的角缝穿过去。
                if c[0]!=prev[0] and c[1]!=prev[1]:
                    if not self.allowed(prev,(c[0],prev[1])) or not self.allowed(prev,(prev[0],c[1])):return False
                prev=c
        return True
    def route(self,start,goal):
        s=self.cell(*start);g=self.cell(*goal)
        if self.cost(s)>=100 or self.cost(g)!=0:return []
        q=[(0.,s)];dist={s:0.};parent={}
        while q:
            value,c=heapq.heappop(q)
            if value!=dist[c]:continue
            if c==g:
                cells=[g]
                while cells[-1]!=s:cells.append(parent[cells[-1]])
                points=[start]+[self.center(c) for c in reversed(cells)][1:-1]+[goal]
                # 保留格心拐点：视线简化会贴着障碍角点，float32转换后可能落入相邻膨胀格。
                # 平滑交给EGO，并在输出处再次检查整条样条；安全折线始终可供回退。
                return points if self.safe(points) else []
            for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                nxt=(c[0]+dx,c[1]+dy)
                if not self.allowed(c,nxt):continue
                if dx and dy and (not self.allowed(c,(c[0]+dx,c[1])) or not self.allowed(c,(c[0],c[1]+dy))):continue
                nd=value+math.hypot(dx,dy)*(1+self.cost(nxt)/50.)
                if nd<dist.get(nxt,float('inf')):dist[nxt]=nd;parent[nxt]=c;heapq.heappush(q,(nd,nxt))
        return []

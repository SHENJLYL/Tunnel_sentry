#!/usr/bin/env python3
"""Render the actual URDF primitive geometry at two lift heights."""
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/sentry_matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import xml.etree.ElementTree as E
import xacro
import yaml

from model_geometry import Model
root=Path(__file__).resolve().parents[1]
model=Model()

def polygons(geometry):
    if geometry.tag=='box':
        sx,sy,sz=[float(v)/2 for v in geometry.get('size').split()]
        v=np.array([[-sx,-sy,-sz],[sx,-sy,-sz],[sx,sy,-sz],[-sx,sy,-sz],[-sx,-sy,sz],[sx,-sy,sz],[sx,sy,sz],[-sx,sy,sz]])
        return [v[idx] for idx in [[0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]]
    r=float(geometry.get('radius'));h=float(geometry.get('length'))/2
    a=np.linspace(0,2*np.pi,25)[:-1];b=np.column_stack([r*np.cos(a),r*np.sin(a),np.full_like(a,-h)])
    t=b.copy();t[:,2]=h
    return [b[::-1],t]+[np.array([b[i],b[(i+1)%24],t[(i+1)%24],t[i]]) for i in range(24)]

fig=plt.figure(figsize=(13,7),facecolor='#f4f6f8')
for index,lift in enumerate([0.,model.params['lift_stroke']]):
    ax=fig.add_subplot(1,2,index+1,projection='3d');ax.set_facecolor('#f4f6f8')
    q=model.pose(lift=lift)
    mounts={'front_right':0,'rear_right':-np.pi/2,'rear_left':np.pi,'front_left':np.pi/2}
    for prefix,mount in mounts.items():q[prefix+'_steering_joint']=np.pi/2-mount
    for part in model.pieces(q,tag='visual'):
        matrix=part['transform'];faces=[]
        for poly in polygons(part['geometry']):
            faces.append((matrix@np.column_stack([poly,np.ones(len(poly))]).T).T[:,:3])
        color=part['color'];colors=[]
        for face in faces:
            normal=np.cross(face[1]-face[0],face[2]-face[0]);normal/=max(1e-9,np.linalg.norm(normal))
            brightness=.65+.35*abs(normal@np.array([.3,-.4,.866]))
            colors.append([*(np.array(color[:3])*brightness),color[3]])
        ax.add_collection3d(Poly3DCollection(faces,facecolor=colors,edgecolor=(.05,.08,.12,.18),linewidth=.4))
    sensor=model.frames(q)['odin_lidar_frame'];position=sensor[:3,3];direction=sensor[:3,0]
    ax.quiver(*position,*direction,length=.10,color='#d97706',linewidth=2,arrow_length_ratio=.25)
    ax.set(xlim=(-.4,.5),ylim=(-.4,.4),zlim=(0,.50),xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)')
    ax.set_box_aspect((.9,.8,.50));ax.view_init(elev=22,azim=40)
    ax.set_title(f"Lift {lift*1000:.1f} mm: {model.bounds(q)[1][2]*100:.1f} cm",pad=12,color='#223344',fontsize=15)
fig.suptitle('Tunnel sentinel / URDF geometry',fontsize=20,color='#172b40',y=.96)
fig.text(.5,.06,'Odin1 on Yaw platform, left of barrel · orange arrow: forward scan direction',ha='center',fontsize=12,color='#46576a')
fig.subplots_adjust(left=.03,right=.97,bottom=.12,top=.87,wspace=.02)
output=root/'docs/model_preview.png';output.parent.mkdir(exist_ok=True)
fig.savefig(output,dpi=160)
print(output)

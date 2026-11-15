"""URDF transforms, exact primitive bounds and conservative shape intersection checks."""
from pathlib import Path
import math
import xml.etree.ElementTree as E
import numpy as np
import xacro
import yaml

ROOT=Path(__file__).resolve().parents[1]

def rotation(axis,angle):
    axis=np.array(axis,dtype=float);axis/=np.linalg.norm(axis)
    x,y,z=axis;k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*k+(1-math.cos(angle))*(k@k)

def origin(node):
    t=np.eye(4)
    if node is not None:
        t[:3,3]=[float(v) for v in node.get('xyz','0 0 0').split()]
        r,p,y=[float(v) for v in node.get('rpy','0 0 0').split()]
        t[:3,:3]=rotation([0,0,1],y)@rotation([0,1,0],p)@rotation([1,0,0],r)
    return t

class Model:
    def __init__(self):
        path=ROOT/'src/tunnel_sentinel_description'
        self.params=yaml.safe_load((path/'config/model.yaml').read_text())
        self.robot=E.fromstring(xacro.process_file(str(path/'urdf/tunnel_sentinel.urdf.xacro'),mappings={
            'model_file':str(path/'config/model.yaml'),'source_file':str(path/'urdf/tunnel_sentinel_lift.urdf'),'controllers_file':'unused','namespace':'test'}).toxml())
        self.primitives=yaml.safe_load((path/'meshes/primitives.yaml').read_text())
        self.links={l.get('name'):l for l in self.robot.findall('link')}
        self.joints=self.robot.findall('joint')
        self.adjacent={frozenset((j.find('parent').get('link'),j.find('child').get('link'))) for j in self.joints}
        self.adjacent.add(frozenset(('base_footprint','base_link')))

    def frames(self,q,base=None):
        start=np.eye(4) if base is None else base.copy()
        if base is None:start[2,3]=self.params['root_height']
        frames={'base_link':start}
        remaining=list(self.joints)
        while remaining:
            count=len(remaining)
            for j in list(remaining):
                parent=j.find('parent').get('link')
                if parent not in frames:continue
                delta=np.eye(4);value=q.get(j.get('name'),0.)
                if j.get('name')=='lift_stage_joint' and 'lift_stage_joint' not in q:
                    value=.5*q.get('lift_joint',0.)
                if j.get('type')!='fixed':
                    axis=np.array([float(v) for v in j.find('axis').get('xyz').split()])
                    if j.get('type')=='prismatic':delta[:3,3]=axis*value
                    else:delta[:3,:3]=rotation(axis,value)
                frames[j.find('child').get('link')]=frames[parent]@origin(j.find('origin'))@delta
                remaining.remove(j)
            if len(remaining)==count:raise ValueError('Joint tree has unresolved parents.')
        return frames

    def pieces(self,q,tag='collision',base=None):
        frames=self.frames(q,base);pieces=[]
        for name,link in self.links.items():
            for obj in link.findall(tag):
                mesh=list(obj.find('geometry'))[0]
                for index,spec in enumerate(self.primitives[Path(mesh.get('filename')).name]):
                    o=E.Element('origin',xyz=' '.join(map(str,spec.get('xyz',[0,0,0]))),rpy=' '.join(map(str,spec.get('rpy',[0,0,0]))))
                    t=frames[name]@origin(obj.find('origin'))@origin(o);center=t[:3,3];axes=t[:3,:3]
                    geom=E.Element(spec['type'])
                    for key in ['size','radius','length']:
                        if key in spec:geom.set(key,' '.join(map(str,spec[key])) if isinstance(spec[key],list) else str(spec[key]))
                    if geom.tag=='box':
                        half=np.array([float(v)/2 for v in geom.get('size').split()]);extent=np.abs(axes)@half
                    else:
                        radius=float(geom.get('radius'));length=float(geom.get('length'));axis=axes[:,2]
                        extent=length*.5*np.abs(axis)+radius*np.sqrt(np.maximum(0,1-axis*axis))
                        reference=np.array([0,0,1] if abs(axis[2])<.9 else [1,0,0])
                        u=np.cross(reference,axis);u/=np.linalg.norm(u)
                        axes=np.column_stack([u,np.cross(axis,u),axis]);half=np.array([radius,radius,length/2])
                    material=obj.find('material/color')
                    color=[float(v) for v in material.get('rgba').split()] if material is not None else spec['color']
                    pieces.append({'link':name,'name':obj.get('name')+'_'+str(index),'center':center,'axes':axes,'half':half,
                                   'lower':center-extent,'upper':center+extent,'transform':t,'geometry':geom,'color':color})
        return pieces

    def bounds(self,q,base=None):
        parts=self.pieces(q,base=base)
        return np.min([p['lower'] for p in parts],axis=0),np.max([p['upper'] for p in parts],axis=0)

    def intersections(self,q):
        parts=self.pieces(q);hits=[]
        for i,a in enumerate(parts):
            for b in parts[i+1:]:
                if a['link']==b['link'] or frozenset((a['link'],b['link'])) in self.adjacent:continue
                if np.any(np.minimum(a['upper'],b['upper'])-np.maximum(a['lower'],b['lower'])<1e-6):continue
                axes=[*a['axes'].T,*b['axes'].T]
                axes.extend(np.cross(x,y) for x in a['axes'].T for y in b['axes'].T)
                overlapping=True
                for axis in axes:
                    norm=np.linalg.norm(axis)
                    if norm<1e-8:continue
                    axis=axis/norm
                    def radius(part):
                        if part['geometry'].tag=='cylinder':
                            cosine=abs(part['axes'][:,2]@axis)
                            return part['half'][2]*cosine+part['half'][0]*math.sqrt(max(0,1-cosine*cosine))
                        return np.abs(part['axes'].T@axis)@part['half']
                    radii=radius(a)+radius(b)
                    if radii-abs((b['center']-a['center'])@axis)<1e-6:
                        overlapping=False;break
                if overlapping:hits.append((a['name'],b['name']))
        return hits

    def pose(self,pitch=0.,yaw=0.,lift=0.):
        return {'gimbal_pitch_joint':pitch,'gimbal_yaw_joint':yaw,'lift_joint':lift}

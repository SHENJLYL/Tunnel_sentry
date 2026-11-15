#!/usr/bin/env python3
"""Write the original resource approximations and slide sleeve meshes."""
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import yaml
from model_geometry import ROOT,origin

def faces(spec):
    if spec['type']=='box':
        x,y,z=np.array(spec['size'])/2
        v=np.array([[-x,-y,-z],[x,-y,-z],[x,y,-z],[-x,y,-z],[-x,-y,z],[x,-y,z],[x,y,z],[-x,y,z]])
        return [v[i] for i in [[3,2,1,0],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]]
    r,h=spec['radius'],spec['length']/2;n=48
    a=np.linspace(0,2*np.pi,n,endpoint=False);b=np.column_stack([r*np.cos(a),r*np.sin(a),np.full(n,-h)]);t=b.copy();t[:,2]=h
    return [b[::-1],t]+[np.array([b[i],b[(i+1)%n],t[(i+1)%n],t[i]]) for i in range(n)]

if __name__=='__main__':
    folder=ROOT/'src/tunnel_sentinel_description/meshes'
    for name,parts in yaml.safe_load((folder/'primitives.yaml').read_text()).items():
        output=['solid '+Path(name).stem]
        for spec in parts:
            o=E.Element('origin',xyz=' '.join(map(str,spec.get('xyz',[0,0,0]))),rpy=' '.join(map(str,spec.get('rpy',[0,0,0]))))
            transform=origin(o)
            for face in faces(spec):
                face=(transform@np.column_stack([face,np.ones(len(face))]).T).T[:,:3]
                for i in range(1,len(face)-1):
                    tri=face[[0,i,i+1]];normal=np.cross(tri[1]-tri[0],tri[2]-tri[0]);normal/=np.linalg.norm(normal)
                    output.append('facet normal '+' '.join(map(str,normal)));output.append('outer loop')
                    output.extend('vertex '+' '.join(map(str,v)) for v in tri)
                    output.extend(['endloop','endfacet'])
        output.append('endsolid '+Path(name).stem);(folder/name).write_text('\n'.join(output)+'\n')
        print(name)

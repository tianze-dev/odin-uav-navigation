#!/usr/bin/env python3
"""Seeded, stratified forest benchmark with physical collision geometry.
Inspired by EGO's random cylinder maps; no ground-truth map is sent to the planner.
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from workspace_paths import ROOT
import argparse,copy,json,random,math,hashlib
import xml.etree.ElementTree as E
SPACING={'sparse':4.4,'medium':3.3,'dense':2.6}
def prepare(density='medium',seed=17):
 if density not in SPACING:raise ValueError('density must be sparse, medium or dense')
 rng=random.Random(seed);spacing=SPACING[density]
 tree=E.parse(ROOT/'simulation/worlds/test.sdf');w=tree.find('world')
 for m in list(w.findall('model')):
  if m.get('name')=='obstacle':w.remove(m)
 def resize(name,pos,size):
  m=w.find(f"model[@name='{name}']");m.find('pose').text=' '.join(map(str,pos))+' 0 0 0'
  for s in m.findall('link/*/geometry/box/size'):s.text=' '.join(map(str,size))
 resize('ground',[12,0,-.1],[40,30,.2]);resize('west',[-3,0,2.5],[.2,20,5])
 resize('east',[29,0,2.5],[.2,20,5]);resize('north',[13,10,2.5],[32,.2,5]);resize('south',[13,-10,2.5],[32,.2,5])
 pads=[(0,0),(24,0),(24,6),(0,6)];centres=[];obstacles=[]
 nx=math.floor(24/spacing);ny=math.floor(17/spacing)
 sx=24/nx;sy=17/ny
 for ix in range(nx):
  for iy in range(ny):
   cx=1.5+(ix+.5)*sx;cy=-8.5+(iy+.5)*sy
   for attempt in range(100):
    x=cx+rng.uniform(-.18,.18)*sx;y=cy+rng.uniform(-.18,.18)*sy
    if all(math.hypot(x-a,y-b)>1.9 for a,b in centres):break
   else:raise RuntimeError('Could not satisfy minimum separation')
   if any(math.hypot(x-a,y-b)<1.35 for a,b in pads):continue
   centres.append((x,y));radius=rng.uniform(.18,.30);height=rng.uniform(3.8,4.6)
   name=f'tree_{ix:02d}_{iy:02d}';m=E.SubElement(w,'model',name=name)
   E.SubElement(m,'static').text='true';E.SubElement(m,'pose').text=f'{x:.8f} {y:.8f} {height/2:.8f} 0 0 0'
   link=E.SubElement(m,'link',name='link');g=E.Element('geometry');c=E.SubElement(g,'cylinder')
   E.SubElement(c,'radius').text=f'{radius:.8f}';E.SubElement(c,'length').text=f'{height:.8f}'
   for tag in ('collision','visual'):
    v=E.SubElement(link,tag,name=tag);v.append(copy.deepcopy(g))
    if tag=='visual':
     mat=E.SubElement(v,'material')
     for k in ('ambient','diffuse'):E.SubElement(mat,k).text='0.36 0.25 0.14 1'
   obstacles.append(dict(name=name,x=x,y=y,radius=radius,height=height,cell=[ix,iy]))
 E.indent(tree);out=ROOT/'generated/worlds'/f'benchmark_{density}_{seed}.sdf';out.parent.mkdir(parents=True,exist_ok=True)
 tree.write(out,encoding='unicode')
 manifest=dict(seed=seed,density=density,count=len(obstacles),sampling='one jittered sample per equal-area cell',grid=[nx,ny],cell_size=[sx,sy],minimum_centre_separation=1.9,reserved_pads=pads,pad_radius=1.35,obstacles=obstacles,sdf_sha256=hashlib.sha256(out.read_bytes()).hexdigest())
 out.with_suffix('.json').write_text(json.dumps(manifest,indent=2)+'\n');return out
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--density',choices=SPACING,default='medium');p.add_argument('--seed',type=int,default=17);a=p.parse_args();print(prepare(a.density,a.seed))

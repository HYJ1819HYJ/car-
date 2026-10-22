#!/usr/bin/env python3
"""Paired six-box maps: the same two boxes move after the prior map is saved."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageDraw
import yaml
import generate_final_competition_map as base


def main():
    base.main()
    pkg=base.PKG
    specification=yaml.safe_load((pkg/'config/relocated_obstacle_scenarios.yaml').read_text())
    geometry=json.loads((pkg/'maps/geometry.json').read_text())
    fixed=np.asarray(Image.open(pkg/'maps/final_competition.pgm'))
    original=geometry['box_rectangles_pixels'][specification['reference_obstacle_index']]
    _,components,stats,_=cv2.connectedComponentsWithStats((fixed==0).astype('uint8'))
    ref=base.xy(((original[0]+original[2])/2,(original[1]+original[3])/2))
    label=components[fixed.shape[0]-1-int((ref[1]+base.MARGIN)/base.RES),int((ref[0]+base.MARGIN)/base.RES)]
    size=[float(stats[label,cv2.CC_STAT_WIDTH]*base.RES),float(stats[label,cv2.CC_STAT_HEIGHT]*base.RES),float(specification['height_m'])]
    origin=np.array([-base.MARGIN,-base.MARGIN])
    def pixels(points):
        p=np.floor((np.asarray(points)-origin)/base.RES).astype('int32')
        p[:,1]=fixed.shape[0]-1-p[:,1]
        return p
    def boxes_at(centers):
        return [{'id':i+1,'center':center,'size':size,'yaw':0.,
                 'corners':(np.asarray(center)+np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*np.asarray(size[:2])/2).tolist(),
                 'color':[1,.4,.05] if i==0 else [.65,.1,.85]} for i,center in enumerate(centers)]
    def raster(boxes):
        truth=fixed.copy(); labels=np.zeros(fixed.shape,np.uint8)
        for box in boxes:
            mask=np.zeros(fixed.shape,np.uint8)
            cv2.fillPoly(mask,[pixels(box['corners'])],1)
            if np.any(truth[mask!=0]!=254):raise ValueError('Box overlaps fixed geometry or another box')
            truth[mask!=0]=0;labels[mask!=0]=box['id']
        return truth,labels
    def save_map(directory,stem,grid):
        Image.fromarray(grid).save(directory/(stem+'.pgm'))
        (directory/(stem+'.yaml')).write_text(yaml.safe_dump({'image':stem+'.pgm','resolution':base.RES,
             'origin':[*origin.tolist(),0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196},sort_keys=False))
    template=(pkg/'worlds/final_competition.world').read_text().replace('../maps/final_competition_stage.png','../../maps/final_competition_stage.png')
    def world(boxes):
        result=template
        for b in boxes:
            result+=f'''model(\n  name "relocated_box_{b['id']}"\n  size [{size[0]} {size[1]} {size[2]}]\n  pose [{b['center'][0]} {b['center'][1]} 0 0]\n  color "{'orange' if b['id']==1 else 'purple'}"\n  gui_move 0 gui_nose 0 obstacle_return 1 ranger_return 1\n)\n'''
        return result
    def preview(boxes,path,name):
        picture=Image.fromarray(fixed).convert('RGB').resize((fixed.shape[1]*3,fixed.shape[0]*3),Image.Resampling.NEAREST)
        draw=ImageDraw.Draw(picture)
        for b in mapped:
            pts=[tuple(p*3) for p in pixels(b['corners'])]
            draw.line(pts+[pts[0]],fill='#e53935',width=4)
            draw.text(pts[0],'old '+str(b['id']),fill='#c00000')
        for b in boxes:
            pts=[tuple(p*3) for p in pixels(b['corners'])]
            draw.polygon(pts,fill='#ee7b24' if b['id']==1 else '#a12dcb')
            draw.text(pts[0],str(b['id']),fill='white')
        draw.text((15,10),name,fill='#175ab3');picture.save(path)
    if len(specification['mapped_centers'])!=2:raise ValueError('Exactly two movable boxes required')
    mapped=boxes_at(specification['mapped_centers'])
    prior,mask=raster(mapped)
    save_map(pkg/'maps','mapping_before_move',prior)
    Image.fromarray(mask).save(pkg/'maps/movable_prior_mask.png')
    prior_hash=hashlib.sha256((pkg/'maps/mapping_before_move.pgm').read_bytes()).hexdigest()
    cases=specification['scenarios']
    catalogue=[]
    for name,case in cases.items():
        title=case['title']
        directory=pkg/'relocated_scenarios'/name;directory.mkdir(parents=True,exist_ok=True)
        if len(case['centers'])!=2:raise ValueError(name+': exactly two boxes required')
        if any(np.allclose(a,b) for a,b in zip(specification['mapped_centers'],case['centers'])):
            raise ValueError(name+': both box IDs must change position')
        boxes=boxes_at(case['centers']);truth,_=raster(boxes)
        save_map(directory,'environment',truth)
        save_map(directory,'before',prior)
        (directory/'world.world').write_text(world(boxes))
        (directory/'before.world').write_text(world(mapped))
        distance=cv2.distanceTransform((truth==254).astype('uint8'),cv2.DIST_L2,5)*base.RES
        _,labels=cv2.connectedComponents((distance>.37).astype('uint8'),connectivity=8)
        start,goal=pixels([geometry['start'][:2],geometry['finish'][:2]])
        connected=bool(labels[start[1],start[0]] and labels[start[1],start[0]]==labels[goal[1],goal[0]])
        record={'id':name,**case,'title':title,'semantics':'same_two_boxes_relocated_before_navigation',
                'boxes':boxes,'mapped_boxes':mapped,'reference_size_m':size,'known_boxes':6,'physical_boxes':6,
                'relocated_boxes':2,'navigation_map':'../../maps/mapping_before_move.yaml',
                'navigation_map_sha256':prior_hash,'truth_map':'environment.yaml',
                'offline_circle_radius_m':.37,'offline_circle_route_exists':connected}
        (directory/'scenario.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
        preview(boxes,directory/'preview.png',name+' AFTER | red outlines: old positions')
        preview(mapped,directory/'before.png','BEFORE mapping | 4 fixed + same 2 movable boxes')
        catalogue.append(record)
    root=pkg/'relocated_scenarios'
    (root/'catalogue.json').write_text(json.dumps(catalogue,ensure_ascii=False,indent=2)+'\n')
    w,h=564,476
    overview=Image.new('RGB',(w*2,h*6+40),'white')
    ImageDraw.Draw(overview).text((15,10),'LEFT: saved six-box map | RIGHT: actual six-box scene after relocation',fill='#175ab3')
    for i,record in enumerate(catalogue):
        for j,file in enumerate(['before.png','preview.png']):
            overview.paste(Image.open(root/record['id']/file).resize((w,h)),(j*w,i*h+40))
    overview.save(root/'overview.png')
    print(json.dumps([{'id':r['id'],'route_exists':r['offline_circle_route_exists'],'size':size} for r in catalogue],indent=2))

if __name__=='__main__':main()

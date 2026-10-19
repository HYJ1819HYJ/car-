#!/usr/bin/env python3
"""Build reproducible worlds with two stationary, previously unmapped boxes."""
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
    pkg = base.PKG
    specification = yaml.safe_load((pkg/'config/added_obstacle_scenarios.yaml').read_text())
    geometry = json.loads((pkg/'maps/geometry.json').read_text())
    original = geometry['box_rectangles_pixels'][specification['reference_obstacle_index']]
    nav_image = np.asarray(Image.open(pkg/'maps/final_competition.pgm'))
    _,components,stats,_=cv2.connectedComponentsWithStats((nav_image==0).astype('uint8'))
    reference_xy=base.xy(((original[0]+original[2])/2,(original[1]+original[3])/2))
    column=int((reference_xy[0]+base.MARGIN)/base.RES)
    row=nav_image.shape[0]-1-int((reference_xy[1]+base.MARGIN)/base.RES)
    reference_component=components[row,column]
    # Clone the actual Stage bitmap body, including map raster quantization.
    size=[stats[reference_component,cv2.CC_STAT_WIDTH]*base.RES,
          stats[reference_component,cv2.CC_STAT_HEIGHT]*base.RES,specification['height_m']]
    world_template = (pkg/'worlds/final_competition.world').read_text().replace(
        '../maps/final_competition_stage.png','../../maps/final_competition_stage.png')
    origin = np.array([-base.MARGIN,-base.MARGIN])
    def pixels(points):
        p = np.floor((np.asarray(points)-origin)/base.RES).astype('int32')
        p[:,1] = nav_image.shape[0]-1-p[:,1]
        return p
    catalogue = []
    for name, case in specification['scenarios'].items():
        directory = pkg/'scenarios'/name
        directory.mkdir(parents=True, exist_ok=True)
        truth = nav_image.copy()
        world = world_template
        boxes = []
        for i, center in enumerate(case['centers']):
            x,y = center
            half = np.asarray(size[:2])/2
            corners = np.array(center)+np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*half
            mask = np.zeros(nav_image.shape,dtype='uint8')
            cv2.fillPoly(mask,[pixels(corners)],1)
            if np.any(nav_image[mask!=0]!=254) or np.any(truth[mask!=0]!=254):
                raise ValueError(f'{name}: box {i+1} overlaps original geometry or another box')
            truth[mask!=0]=0
            boxes.append({'id':i+1,'center':center,'size':size,'yaw':0.0,
                          'corners':corners.tolist(),'color':[1,.4,.05] if i==0 else [.65,.1,.85]})
            color = 'orange' if i==0 else 'purple'
            world += f'''model(
  name "added_box_{i+1}"
  size [{size[0]:.9f} {size[1]:.9f} {size[2]:.9f}]
  pose [{x} {y} 0 0]
  color "{color}"
  gui_move 0 gui_nose 0 obstacle_return 1 ranger_return 1
)
'''
        Image.fromarray(truth).save(directory/'environment.pgm')
        (directory/'environment.yaml').write_text(yaml.safe_dump({
            'image':'environment.pgm','resolution':base.RES,'origin':[*origin.tolist(),0.0],
            'negate':0,'occupied_thresh':.65,'free_thresh':.196},sort_keys=False))
        (directory/'world.world').write_text(world)
        # Circular full-body clearance is an offline reachability sanity check,
        # never loaded as privileged obstacle information by ROS navigation.
        distance=cv2.distanceTransform((truth==254).astype('uint8'),cv2.DIST_L2,5)*base.RES
        free=(distance>.37).astype('uint8')
        _,labels=cv2.connectedComponents(free,connectivity=8)
        start,goal=pixels([geometry['start'][:2],geometry['finish'][:2]])
        connected=bool(labels[start[1],start[0]] and
                       labels[start[1],start[0]]==labels[goal[1],goal[0]])
        record={'id':name,**case,'boxes':boxes,'reference_size_m':size,
                'navigation_map':'../../maps/final_competition.yaml',
                'navigation_map_sha256':hashlib.sha256((pkg/'maps/final_competition.pgm').read_bytes()).hexdigest(),
                'truth_map':'environment.yaml','known_boxes':4,'added_boxes':2,
                'offline_circle_radius_m':.37,'offline_circle_route_exists':connected}
        (directory/'scenario.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
        preview=Image.fromarray(nav_image).convert('RGB').resize(
            (nav_image.shape[1]*3,nav_image.shape[0]*3),Image.Resampling.NEAREST)
        draw=ImageDraw.Draw(preview)
        for box in boxes:
            polygon=[tuple(p*3) for p in pixels(box['corners'])]
            draw.polygon(polygon,fill='#ee7b24' if box['id']==1 else '#a12dcb')
            draw.text((polygon[0][0],polygon[0][1]+3),str(box['id']),fill='white')
        draw.text((15,10),name,fill='#175ab3')
        preview.save(directory/'preview.png')
        catalogue.append(record)
    (pkg/'scenarios/catalogue.json').write_text(json.dumps(catalogue,ensure_ascii=False,indent=2)+'\n')
    width,height=676,571
    overview=Image.new('RGB',(width*2,height*3+45),'white')
    drawing=ImageDraw.Draw(overview)
    drawing.text((15,10),'Navigation: original 4-box map | Orange + purple: boxes added AFTER mapping',fill='#175ab3')
    for i,case in enumerate(catalogue):
        tile=Image.open(pkg/'scenarios'/case['id']/'preview.png').resize((width,height))
        overview.paste(tile,((i%2)*width,(i//2)*height+45))
    overview.save(pkg/'scenarios/overview.png')
    print(json.dumps([{'id':r['id'],'size_m':r['reference_size_m'],
                       'route_exists':r['offline_circle_route_exists']} for r in catalogue],indent=2))


if __name__=='__main__':
    main()

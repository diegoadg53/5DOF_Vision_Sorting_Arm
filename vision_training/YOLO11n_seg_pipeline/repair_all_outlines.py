"""Replace color-only outlines with object-aware full outlines for green parts.

Class labels remain those verified by the user. This stage was added after the
user found missing shiny/dark regions in the color-only Pieza_05/Pieza_08 masks.
"""
from pathlib import Path
import json
import time
import cv2
import numpy as np
import torch
from PIL import Image,ImageDraw,ImageFont
from ultralytics import SAM
from refine_masks import refine_assembly

ROOT=Path(__file__).resolve().parent


if __name__=='__main__':
    cv2.setNumThreads(2);torch.set_num_threads(4)
    source=ROOT/'dataset_initial_color_masks'
    records=json.loads((source/'source_manifest.json').read_text(encoding='utf-8'))
    path=ROOT/'review'/'all_parts_refined.json'
    cached=json.loads(path.read_text(encoding='utf-8')) if path.exists() else []
    cache={(r['frame'],r['object_index']):r for r in cached}
    model=SAM(str(ROOT/'sam2.1_t.pt'));results=[];start=time.monotonic();new=0
    for row in records:
        frame=None
        for oi,obj in enumerate(row['objects']):
            if obj['class_id']==0:continue
            key=(row['frame'],oi)
            if key in cache:results.append(cache[key]);continue
            if frame is None:frame=cv2.imread(str(source/'images'/row['split']/row['frame']))
            original=np.asarray(obj['polygon'],np.int32)
            inp={'polygon':obj['polygon'],'bbox':list(cv2.boundingRect(original)),
                 'track_id':obj['track_ids'][0]}
            refined,extra=refine_assembly(model,frame,[inp],pad=40)
            result={'frame':row['frame'],'time':row['time'],'object_index':oi,
                    'class_id':obj['class_id'],'track_ids':obj['track_ids'],
                    'original_polygon':obj['polygon'],'status':'pending_visual_review'}
            if refined:
                poly=np.asarray(refined['polygon'],np.int32)
                original_area=cv2.contourArea(original);new_area=cv2.contourArea(poly)
                seed=np.zeros(frame.shape[:2],np.uint8);mask=np.zeros_like(seed)
                cv2.fillPoly(seed,[original],255);cv2.fillPoly(mask,[poly],255)
                union=np.logical_or(seed>0,mask>0).sum();intersection=np.logical_and(seed>0,mask>0).sum()
                result.update(polygon=refined['polygon'],bbox=refined['bbox'],
                              area_ratio=float(new_area/max(1,original_area)),
                              seed_coverage=refined['green_seed_coverage'],
                              old_new_iou=float(intersection/max(1,union)),
                              partial=refined['partial'],method='SAM2.1_t_crop_prompt_full_part')
                if refined['partial'] or result['seed_coverage']<.88 or result['area_ratio']>2.8 or result['area_ratio']<.65:
                    result['status']='flagged'
            else:result.update(status='failed',reason=extra.get('reason',''))
            results.append(result);new+=1
            if new%100==0:
                path.write_text(json.dumps(results,indent=2),encoding='utf-8')
                print(f'{len(results)} refined green objects; {time.monotonic()-start:.1f}s; flagged {sum(r["status"] in ["flagged","failed"] for r in results)}',flush=True)
    path.write_text(json.dumps(results,indent=2),encoding='utf-8')
    print(f'Saved {len(results)} outlines in {time.monotonic()-start:.1f}s',flush=True)
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',14)
    # Review the largest changes plus a balanced set from every class.
    by_frame={row['frame']:row for row in records}
    selected=[r for r in results if r['status'] in ['flagged','failed']]
    selected.extend(sorted([r for r in results if 'polygon' in r],key=lambda r:r['old_new_iou'])[:72])
    for cid in range(1,8):
        candidates=[r for r in results if r['class_id']==cid and 'polygon' in r]
        for index in np.linspace(0,len(candidates)-1,min(12,len(candidates)),dtype=int):selected.append(candidates[index])
    unique={(r['frame'],r['object_index']):r for r in selected};tiles=[]
    for result in unique.values():
        row=by_frame[result['frame']];frame=cv2.imread(str(source/'images'/row['split']/row['frame']))
        poly=np.asarray(result.get('polygon',result['original_polygon']),np.int32)
        cv2.polylines(frame,[poly],True,(255,0,255),2)
        x,y,w,h=cv2.boundingRect(poly);pad=12
        im=Image.fromarray(cv2.cvtColor(frame[max(0,y-pad):min(720,y+h+pad),max(0,x-pad):min(1280,x+w+pad)],cv2.COLOR_BGR2RGB))
        scale=min(285/im.width,230/im.height);im=im.resize((round(im.width*scale),round(im.height*scale)),Image.Resampling.LANCZOS)
        tile=Image.new('RGB',(300,275),'#202329');tile.paste(im,((300-im.width)//2,42+(230-im.height)//2));draw=ImageDraw.Draw(tile)
        draw.text((4,2),f'{result["frame"]} o{result["object_index"]} Pieza_{result["class_id"]+1:02d}',font=font,fill='white')
        draw.text((4,20),f'IoU={result.get("old_new_iou",0):.2f} area={result.get("area_ratio",0):.2f} {result["status"]}',font=font,fill='#ffe077')
        tiles.append(tile)
    for page in range((len(tiles)+23)//24):
        subset=tiles[page*24:(page+1)*24];sheet=Image.new('RGB',(1200,275*((len(subset)+3)//4)),'#202329')
        for i,tile in enumerate(subset):sheet.paste(tile,((i%4)*300,(i//4)*275))
        sheet.save(ROOT/'review'/f'repaired_outline_review_{page+1:02d}.jpg',quality=94)
    print(f'Created {(len(tiles)+23)//24} review sheets',flush=True)

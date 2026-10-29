"""Refine the user-confirmed full Pieza_01 assembly with crop-prompted SAM2.

Other parts retain conservative external contours from the reviewed color masks.
Every result is still checked by visual galleries before dataset construction.
"""
from pathlib import Path
import json
import time
import cv2
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from ultralytics import SAM

ROOT=Path(__file__).resolve().parent


def refine_assembly(model,frame,objects,pad=85):
    x=min(o['bbox'][0] for o in objects);y=min(o['bbox'][1] for o in objects)
    x1=max(o['bbox'][0]+o['bbox'][2] for o in objects);y1=max(o['bbox'][1]+o['bbox'][3] for o in objects)
    x0=max(0,x-pad);y0=max(0,y-pad)
    crop=frame[y0:min(frame.shape[0],y1+pad),x0:min(frame.shape[1],x1+pad)]
    seed=np.zeros(crop.shape[:2],np.uint8)
    for obj in objects:cv2.fillPoly(seed,[np.asarray(obj['polygon'],np.int32)-[x0,y0]],255)
    hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
    belt=crop[(hsv[:,:,1]<70)&(hsv[:,:,2]>70)]
    if len(belt)<20:return None,{'reason':'insufficient_background_reference'}
    base=np.median(belt,axis=0)
    difference=np.linalg.norm(crop.astype(float)-base,axis=2)
    gc=np.full(crop.shape[:2],cv2.GC_PR_BGD,np.uint8)
    gc[difference>45]=cv2.GC_PR_FGD;gc[seed>0]=cv2.GC_FGD
    gc[:3]=cv2.GC_BGD;gc[-3:]=cv2.GC_BGD;gc[:,:3]=cv2.GC_BGD;gc[:,-3:]=cv2.GC_BGD
    cv2.grabCut(crop,gc,None,np.zeros((1,65),np.float64),np.zeros((1,65),np.float64),3,cv2.GC_INIT_WITH_MASK)
    foreground=np.uint8((gc==cv2.GC_FGD)|(gc==cv2.GC_PR_FGD))*255
    count,components,stats,_=cv2.connectedComponentsWithStats(foreground)
    best=max(range(1,count),key=lambda k:int(((components==k)&(seed>0)).sum()))
    bx,by,bw,bh=map(int,stats[best,:4])
    dt=cv2.distanceTransform(seed,cv2.DIST_L2,5);py,px=np.unravel_index(np.argmax(dt),dt.shape)
    box=[max(0,bx-4),max(0,by-4),min(crop.shape[1]-1,bx+bw+4),min(crop.shape[0]-1,by+bh+4)]
    result=model(crop,bboxes=[box],points=[[[int(px),int(py)]]],labels=[[1]],device=0,verbose=False)[0]
    if result.masks is None:return None,{'reason':'SAM_no_mask'}
    mask=(result.masks.data[0].cpu().numpy()>0).astype(np.uint8)*255
    if mask.shape!=crop.shape[:2]:mask=cv2.resize(mask,(crop.shape[1],crop.shape[0]),interpolation=cv2.INTER_NEAREST)
    count,components,stats,_=cv2.connectedComponentsWithStats(mask)
    if count<2:return None,{'reason':'empty_mask'}
    best=max(range(1,count),key=lambda k:int(((components==k)&(seed>0)).sum()))
    mask=np.uint8(components==best)*255
    coverage=float(((seed>0)&(mask>0)).sum()/max(1,(seed>0).sum()))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contour=max(contours,key=cv2.contourArea)
    approx=cv2.approxPolyDP(contour,max(.6,.0015*cv2.arcLength(contour,True)),True).reshape(-1,2)
    poly=(approx+[x0,y0]).tolist()
    full=np.zeros(frame.shape[:2],np.uint8)
    full[y0:y0+mask.shape[0],x0:x0+mask.shape[1]]=mask
    bx,by,bw,bh=cv2.boundingRect(contour)
    cropped=bx<=2 or by<=2 or bx+bw>=crop.shape[1]-2 or by+bh>=crop.shape[0]-2
    return {'class_id':0,'polygon':poly,'source_tracks':[o['track_id'] for o in objects],
            'bbox':[bx+x0,by+y0,bw,bh],'method':'SAM2.1_t_crop_prompt',
            'green_seed_coverage':coverage,'partial':cropped}, {'mask':full,'coverage':coverage}


if __name__=='__main__':
    data=json.loads((ROOT/'candidates'/'annotations_proposed.json').read_text(encoding='utf-8'))
    reviewed=json.loads((ROOT/'review'/'reviewed_tracks.json').read_text(encoding='utf-8'))
    labels={r['track_id']:r for r in reviewed['tracks']}
    # Start from the candidate classes to cover tiny green fragments of the assembly.
    proposed=json.loads((ROOT/'review'/'class_proposals.json').read_text(encoding='utf-8'))
    tentative={r['track_id']:r['class_id'] for r in proposed['proposals']}
    model=SAM(str(ROOT/'sam2.1_t.pt'))
    masks_dir=ROOT/'review'/'pieza01_masks';masks_dir.mkdir(exist_ok=True)
    frames=[];tiles=[];start=time.monotonic();font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',15)
    for ix,record in enumerate(data['frames']):
        relevant=[o for o in record['objects'] if labels[o['track_id']]['status']=='accepted' and labels[o['track_id']]['class_id']==0]
        if not relevant:continue
        # Green components within the same assembly become one instance mask.
        centers=np.asarray([o['center'] for o in relevant])
        for obj in record['objects']:
            if obj not in relevant and labels[obj['track_id']]['status']!='accepted' and tentative[obj['track_id']]==0 and np.min(np.linalg.norm(centers-np.asarray(obj['center']),axis=1))<170:
                relevant.append(obj)
        frame=cv2.imread(str(ROOT/'candidates'/'frames'/record['frame']))
        refined,extra=refine_assembly(model,frame,relevant)
        entry={'frame':record['frame'],'time':record['time'],'object':refined,
               'status':'pending_visual_review' if refined else 'failed',
               'reason':extra.get('reason','')}
        if refined:
            cv2.imwrite(str(masks_dir/(Path(record['frame']).stem+'.png')),extra['mask'])
            x,y,w,h=refined['bbox'];pad=18
            g=frame.copy();cv2.polylines(g,[np.asarray(refined['polygon'],np.int32)],True,(255,0,255),2)
            crop=g[max(0,y-pad):min(720,y+h+pad),max(0,x-pad):min(1280,x+w+pad)]
            im=Image.fromarray(cv2.cvtColor(crop,cv2.COLOR_BGR2RGB));scale=min(285/im.width,235/im.height)
            im=im.resize((round(im.width*scale),round(im.height*scale)),Image.Resampling.LANCZOS)
            tile=Image.new('RGB',(300,270),'#202329');tile.paste(im,((300-im.width)//2,30+(235-im.height)//2))
            draw=ImageDraw.Draw(tile);draw.text((5,4),f'{record["frame"]} {record["time"]:.1f}s {extra["coverage"]:.2f}',font=font,fill='white');tiles.append(tile)
        frames.append(entry)
        if len(frames)%20==0:print(f'{len(frames)} assembly masks; {time.monotonic()-start:.1f}s',flush=True)
        if len(frames)%20==0:(ROOT/'review'/'pieza01_refined.json').write_text(json.dumps(frames,indent=2),encoding='utf-8')
    (ROOT/'review'/'pieza01_refined.json').write_text(json.dumps(frames,indent=2),encoding='utf-8')
    for page in range((len(tiles)+23)//24):
        subset=tiles[page*24:(page+1)*24];sheet=Image.new('RGB',(1200,270*((len(subset)+3)//4)),'#202329')
        for i,tile in enumerate(subset):sheet.paste(tile,((i%4)*300,(i//4)*270))
        sheet.save(ROOT/'review'/f'pieza01_masks_{page+1:02d}.jpg',quality=94)
    print(f'Saved {len(frames)} assembly proposals',flush=True)

"""Cache rotation-only visual descriptors to assist class review.

Descriptors support proposals; nearest-neighbour classes are never ground truth.
"""
from pathlib import Path
import json
import time
import cv2
import numpy as np
import torch
from ultralytics import YOLO

ROOT=Path(__file__).resolve().parent


def object_canvas(frame, obj):
    x,y,w,h=obj['bbox']
    pad=8
    x0=max(0,x-pad);y0=max(0,y-pad)
    crop=frame[y0:min(frame.shape[0],y+h+pad),x0:min(frame.shape[1],x+w+pad)].copy()
    polygon=np.asarray(obj['polygon'],dtype=np.int32)-[x0,y0]
    mask=np.zeros(crop.shape[:2],np.uint8)
    cv2.fillPoly(mask,[polygon],255)
    mask=cv2.dilate(mask,np.ones((3,3),np.uint8))
    crop[mask==0]=(114,114,114)
    scale=160/max(crop.shape[:2])
    crop=cv2.resize(crop,(round(crop.shape[1]*scale),round(crop.shape[0]*scale)))
    canvas=np.full((224,224,3),114,np.uint8)
    yy=(224-crop.shape[0])//2;xx=(224-crop.shape[1])//2
    canvas[yy:yy+crop.shape[0],xx:xx+crop.shape[1]]=crop
    return canvas


if __name__=='__main__':
    data=json.loads((ROOT/'candidates'/'annotations_proposed.json').read_text(encoding='utf-8'))
    model=YOLO(str(ROOT/'yolo11n-seg.pt')).model.to('cuda').float().eval()
    pending=[];keys=[];vectors=[]
    start=time.monotonic()
    def flush():
        if not pending:return
        arr=np.ascontiguousarray(np.stack(pending)[:,:,:,::-1].transpose(0,3,1,2),dtype=np.float32)/255.0
        with torch.inference_mode():
            out=model(torch.from_numpy(arr).to('cuda'),embed=[4,6,10])
            batch=torch.stack(out).cpu().numpy()
        batch/=np.maximum(1e-9,np.linalg.norm(batch,axis=1,keepdims=True))
        vectors.extend(batch)
        pending.clear()
    for ix,track in enumerate(data['tracks']):
        obs=track['observations']
        valid=[o for o in obs if not o['partial']] or obs
        choices=sorted(set(int((len(valid)-1)*q) for q in (.2,.5,.8)))
        for oi in choices:
            item=valid[oi]
            frame=cv2.imread(str(ROOT/'candidates'/'frames'/item['frame']))
            canvas=object_canvas(frame,item)
            for angle in range(0,360,45):
                rot=cv2.warpAffine(canvas,cv2.getRotationMatrix2D((112,112),angle,1),(224,224),borderValue=(114,114,114))
                pending.append(rot);keys.append([track['id'],item['frame'],angle])
                if len(pending)>=32:flush()
        if ix%30==0:print(f'{ix}/{len(data["tracks"])} tracks; {time.monotonic()-start:.1f}s',flush=True)
    flush()
    np.savez_compressed(ROOT/'review'/'track_embeddings.npz',vectors=np.asarray(vectors),keys=np.asarray(keys,dtype=str))
    print(f'Saved {len(keys)} descriptors; {time.monotonic()-start:.1f}s',flush=True)

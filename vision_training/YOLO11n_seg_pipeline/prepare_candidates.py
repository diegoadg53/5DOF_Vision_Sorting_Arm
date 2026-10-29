"""Extract full video frames, candidate polygons and temporal object tracks.

Candidates are proposals, never accepted ground truth until reviewed.
"""
from pathlib import Path
import argparse
import json
import time

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
VIDEO = ROOT.parent / "OriginalVideo" / "19700101_080621_recovered.mp4"


def proposals(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (35, 100, 25), (85, 255, 255))
    # Remove the conveyor rails from proposal search, retaining the original image.
    mask[:45] = 0
    mask[640:] = 0
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result = []
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        if not 220 <= area <= 60000 or min(w, h) < 9:
            continue
        epsilon = max(0.6, 0.0015 * cv2.arcLength(contour, True))
        poly = cv2.approxPolyDP(contour, epsilon, True).reshape(-1, 2)
        if len(poly) < 3:
            continue
        moments = cv2.moments(contour)
        center = [moments['m10'] / moments['m00'], moments['m01'] / moments['m00']]
        result.append({
            'bbox': [int(x), int(y), int(w), int(h)], 'center': center,
            'area': float(area), 'polygon': poly.tolist(),
            'partial': x <= 3 or x + w >= frame.shape[1] - 3 or y <= 47 or y + h >= 638,
        })
    return sorted(result, key=lambda item: item['center'][0])


def scan(fps):
    frames_dir = ROOT / 'candidates' / 'frames'
    frames_dir.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(str(VIDEO))
    if not cap.isOpened():
        raise RuntimeError(f'Cannot open {VIDEO}')
    source_fps = cap.get(cv2.CAP_PROP_FPS)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    step = max(1, round(source_fps / fps))
    records, tracks, active = [], {}, {}
    next_track = 1
    start = time.monotonic()
    for frame_id in range(0, total, step):
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_id)
        ok, frame = cap.read()
        if not ok:
            continue
        timestamp = frame_id / source_fps
        objects = proposals(frame)
        live = [tid for tid in active if timestamp - active[tid]['time'] <= 1.1]
        matched = {}
        if live and objects:
            costs = np.full((len(live), len(objects)), 1e6)
            for i, tid in enumerate(live):
                old = active[tid]
                for j, obj in enumerate(objects):
                    distance = np.linalg.norm(np.array(old['center']) - obj['center'])
                    ratio = max(old['area'], obj['area']) / max(1, min(old['area'], obj['area']))
                    if distance < 180 and ratio < 3.5:
                        costs[i, j] = distance + 25 * abs(np.log(ratio))
            rows, cols = linear_sum_assignment(costs)
            for i, j in zip(rows, cols):
                if costs[i, j] < 1e5:
                    matched[j] = live[i]
        filename = f'frame_{frame_id:06d}.jpg'
        # Also retain empty frames for review and eventual background examples.
        if objects or frame_id % (step * 8) == 0:
            cv2.imwrite(str(frames_dir / filename), frame, [cv2.IMWRITE_JPEG_QUALITY, 94])
            for j, obj in enumerate(objects):
                tid = matched.get(j)
                if tid is None:
                    tid = next_track
                    next_track += 1
                    tracks[tid] = {'id': tid, 'observations': [], 'class_id': None, 'review_status': 'pending'}
                obj['track_id'] = tid
                tracks[tid]['observations'].append({'frame': filename, 'time': timestamp, **obj})
                active[tid] = {'time': timestamp, **obj}
            records.append({'frame': filename, 'frame_id': frame_id, 'time': timestamp, 'objects': objects})
        if frame_id % (step * 100) == 0:
            print(f'{frame_id}/{total} frames; {len(records)} images; {next_track-1} tracks; {time.monotonic()-start:.1f}s', flush=True)
    cap.release()
    data = {'video': str(VIDEO), 'source_fps': source_fps, 'source_frames': total,
            'sampling_step': step, 'classes': {i: f'Pieza_{i+1:02d}' for i in range(8)},
            'proposal_parameters': {'hue': [35,85], 'saturation_min': 100, 'value_min': 25},
            'frames': records, 'tracks': list(tracks.values())}
    (ROOT / 'candidates' / 'annotations_proposed.json').write_text(json.dumps(data, indent=2), encoding='utf-8')
    print(f'Saved {len(records)} frames, {len(tracks)} tracks', flush=True)
    return data


def gallery(data):
    dest = ROOT / 'review'
    dest.mkdir(exist_ok=True)
    crops_dir = dest / 'tracks'
    crops_dir.mkdir(exist_ok=True)
    tracks = [t for t in data['tracks'] if len(t['observations']) >= 3]
    tiles = []
    try:
        font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 15)
    except OSError:
        font = ImageFont.load_default()
    for track in tracks:
        obs = track['observations']
        valid = [o for o in obs if not o['partial']] or obs
        picked = [valid[int((len(valid)-1)*q)] for q in (.2,.5,.8)]
        tile = Image.new('RGB', (420, 190), '#202329')
        draw = ImageDraw.Draw(tile)
        draw.text((6, 4), f"T{track['id']:03d} {obs[0]['time']:.1f}-{obs[-1]['time']:.1f}s n={len(obs)}", fill='white', font=font)
        for j, item in enumerate(picked):
            frame = cv2.imread(str(ROOT / 'candidates' / 'frames' / item['frame']))
            x,y,w,h = item['bbox']
            pad=14
            crop=frame[max(0,y-pad):min(frame.shape[0],y+h+pad),max(0,x-pad):min(frame.shape[1],x+w+pad)]
            im=Image.fromarray(cv2.cvtColor(crop,cv2.COLOR_BGR2RGB))
            im.thumbnail((136,140))
            tile.paste(im,(j*140+(140-im.width)//2,30+(140-im.height)//2))
        tile.save(crops_dir / f"T{track['id']:03d}.jpg", quality=94)
        tiles.append(tile)
    for page in range((len(tiles)+23)//24):
        subset=tiles[page*24:(page+1)*24]
        sheet=Image.new('RGB',(1260,190*((len(subset)+2)//3)),'#202329')
        for i,tile in enumerate(subset):sheet.paste(tile,((i%3)*420,(i//3)*190))
        path=dest / f'track_gallery_{page+1:02d}.jpg'
        sheet.save(path,quality=94)
        print(path,flush=True)
    summary=[{'track_id':t['id'],'start_seconds':t['observations'][0]['time'],
              'end_seconds':t['observations'][-1]['time'],'observations':len(t['observations']),
              'class_id':None,'face':None,'status':'pending'} for t in data['tracks']]
    (dest / 'track_review.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--fps',type=float,default=2)
    parser.add_argument('--gallery-only',action='store_true')
    args=parser.parse_args()
    data=json.loads((ROOT/'candidates'/'annotations_proposed.json').read_text(encoding='utf-8')) if args.gallery_only else scan(args.fps)
    gallery(data)

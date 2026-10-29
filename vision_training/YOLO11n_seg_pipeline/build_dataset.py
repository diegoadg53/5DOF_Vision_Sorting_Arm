"""Build and audit a YOLO segmentation dataset from reviewed video annotations.

Full frames are retained. Any unresolved object causes the whole frame to be
excluded. Splits use temporal blocks, guard intervals and disjoint object tracks.
"""
from pathlib import Path
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import shutil
import cv2
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parent


def make_records(block_seconds,guard):
    source=json.loads((ROOT/'candidates'/'annotations_proposed.json').read_text(encoding='utf-8'))
    reviewed=json.loads((ROOT/'review'/'reviewed_tracks.json').read_text(encoding='utf-8'))
    tracks={r['track_id']:r for r in reviewed['tracks']}
    refinements={r['frame']:r for r in json.loads((ROOT/'review'/'pieza01_refined.json').read_text(encoding='utf-8'))}
    negatives=set(json.loads((ROOT/'review'/'negatives_accepted.json').read_text(encoding='utf-8')))
    outline_repairs={(r['frame'],r['object_index']):r for r in json.loads((ROOT/'review'/'all_parts_refined.json').read_text(encoding='utf-8'))}
    records=[];excluded=Counter()
    # Tracks crossing a temporal boundary are omitted completely.
    safe_tracks={tid for tid,t in tracks.items()
                 if int(t['start_seconds']/block_seconds)==int(t['end_seconds']/block_seconds)}
    for row in source['frames']:
        time=row['time'];group=int(time/block_seconds);offset=time%block_seconds
        if offset<guard or offset>block_seconds-guard:
            excluded['temporal_guard']+=1;continue
        objects=[];covered_tracks=set();bad=False
        assembly=refinements.get(row['frame'])
        if assembly and assembly['status']=='accepted':
            obj=assembly['object']
            if obj['partial'] or obj['green_seed_coverage']<.9:
                excluded['assembly_partial_or_incomplete']+=1;continue
            tids=obj['source_tracks']
            if any(tid not in safe_tracks for tid in tids):
                excluded['track_crosses_temporal_block']+=1;continue
            objects.append({'class_id':0,'polygon':obj['polygon'],'track_ids':tids,'method':obj['method']})
            covered_tracks.update(tids)
        for obj in row['objects']:
            tid=obj['track_id'];label=tracks[tid]
            if tid in covered_tracks:continue
            # A tiny component already inside the full assembly mask is covered.
            if objects and objects[0]['class_id']==0:
                inside=cv2.pointPolygonTest(np.asarray(objects[0]['polygon'],np.float32),tuple(obj['center']),False)
                if inside>=0:
                    covered_tracks.add(tid);continue
            if label['status']!='accepted' or tid not in safe_tracks:
                excluded['unresolved_or_crossing_track']+=1;bad=True;break
            if label['class_id']==0:
                excluded['assembly_mask_not_accepted']+=1;bad=True;break
            if obj['partial']:
                excluded['partial_color_outline']+=1;bad=True;break
            objects.append({'class_id':label['class_id'],'polygon':obj['polygon'],'track_ids':[tid],'method':'reviewed_green_external_contour'})
        if bad:continue
        if not row['objects'] and row['frame'] not in negatives:continue
        if row['objects'] and not objects:
            excluded['no_complete_label']+=1;continue
        for oi,obj in enumerate(objects):
            if obj['class_id']==0:continue
            repair=outline_repairs.get((row['frame'],oi))
            if not repair or repair['status']!='accepted' or repair['class_id']!=obj['class_id']:
                excluded['full_outline_not_accepted']+=1;bad=True;break
            obj['original_color_polygon']=obj['polygon']
            obj['polygon']=repair['polygon']
            obj['method']=repair['method']
        if bad:continue
        records.append({'frame':row['frame'],'frame_id':row['frame_id'],'time':time,
                        'group':group,'objects':objects,'track_ids':sorted(covered_tracks|{tid for o in objects for tid in o['track_ids']})})
    return records,dict(excluded)


def choose_splits(records,seed):
    grouped=defaultdict(list)
    for record in records:grouped[record['group']].append(record)
    groups=sorted(grouped)
    counts=np.zeros((len(groups),8),int)
    images=np.asarray([len(grouped[g]) for g in groups])
    for i,g in enumerate(groups):
        for row in grouped[g]:
            for obj in row['objects']:counts[i,obj['class_id']]+=1
    totals=counts.sum(axis=0)
    if np.any(totals<15):raise RuntimeError(f'Insufficient reviewed examples: {totals.tolist()}')
    rng=np.random.default_rng(seed);ratios=np.asarray([.7,.15,.15]);best=None
    # Repeated stratified candidate search balances every class at the group level.
    for trial in range(25000):
        assign=rng.choice(3,size=len(groups),p=ratios)
        actual=np.stack([counts[assign==s].sum(axis=0) for s in range(3)])
        if np.any(actual[0]<15) or np.any(actual[1:]<8):continue
        proportions=actual/np.maximum(1,totals)
        score=float(np.square(proportions-ratios[:,None]).sum())
        image_ratio=np.asarray([images[assign==s].sum() for s in range(3)])/max(1,images.sum())
        score+=.2*float(np.square(image_ratio-ratios).sum())
        if best is None or score<best[0]:best=(score,assign.copy(),actual.copy())
    if best is None:raise RuntimeError('Cannot give every class independent train/val/test temporal groups.')
    names=['train','val','test'];mapping={g:names[s] for g,s in zip(groups,best[1])}
    for row in records:row['split']=mapping[row['group']]
    return {'group_assignments':mapping,'class_instances':{names[i]:best[2][i].tolist() for i in range(3)},
            'ratio_objective':best[0]}


def audit_dataset(root,records,split_info,excluded,block_seconds,guard):
    errors=[];hash_splits={};track_splits={};group_splits={};backgrounds=Counter();images=Counter();instances={s:Counter() for s in ['train','val','test']}
    for row in records:
        split=row['split'];image=root/'images'/split/row['frame'];label=root/'labels'/split/(Path(row['frame']).stem+'.txt')
        if not image.exists() or not label.exists():errors.append(f'Missing pair: {row["frame"]}');continue
        digest=hashlib.sha256(image.read_bytes()).hexdigest()
        if digest in hash_splits and hash_splits[digest]!=split:errors.append(f'Identical image across splits: {row["frame"]}')
        hash_splits[digest]=split
        for tid in row['track_ids']:
            if tid in track_splits and track_splits[tid]!=split:errors.append(f'Track {tid} spans splits')
            track_splits[tid]=split
        group=row['group']
        if group in group_splits and group_splits[group]!=split:errors.append(f'Group {group} spans splits')
        group_splits[group]=split
        images[split]+=1
        if not row['objects']:backgrounds[split]+=1
        lines=label.read_text(encoding='utf-8').splitlines()
        if len(lines)!=len(row['objects']):errors.append(f'Object count mismatch: {label.name}')
        for line in lines:
            fields=line.split();cid=int(fields[0]);coordinates=np.asarray(fields[1:],float)
            if cid not in range(8) or len(coordinates)<6 or len(coordinates)%2 or not np.isfinite(coordinates).all() or np.any(coordinates<0) or np.any(coordinates>1):
                errors.append(f'Invalid label: {label.name}')
            elif cv2.contourArea(coordinates.reshape(-1,2).astype(np.float32))<=0:
                errors.append(f'Degenerate polygon: {label.name}')
            instances[split][cid]+=1
    for split in ['train','val','test']:
        for cid in range(8):
            if instances[split][cid]<4:errors.append(f'{split} lacks class {cid}')
    report={'ok':not errors,'errors':errors,'images_by_split':dict(images),
            'background_images_by_split':dict(backgrounds),
            'class_instances_by_split':{s:{f'Pieza_{cid+1:02d}':c[cid] for cid in range(8)} for s,c in instances.items()},
            'temporal_block_seconds':block_seconds,'boundary_guard_seconds':guard,
            'disjoint_source_frames':True,'disjoint_tracks':not any('Track ' in e for e in errors),
            'disjoint_temporal_groups':not any('Group ' in e for e in errors),
            'excluded_candidate_counts':excluded,'split_selection':split_info,
            'annotation_definition':'Reviewed full-part SAM2.1 silhouette polygons; internal holes are filled. Pieza_01 includes the housing and mechanism as one instance.',
            'scope':'One camera recording. Test groups are held out, but the same physical specimens and recording conditions occur across splits.'}
    (root/'audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--block-seconds',type=float,default=20)
    parser.add_argument('--guard',type=float,default=1.5);parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args()
    records,excluded=make_records(args.block_seconds,args.guard)
    info=choose_splits(records,42)
    print(f'{len(records)} selected frames; instances by split: {info["class_instances"]}',flush=True)
    if args.dry_run:print('Excluded',excluded);raise SystemExit(0)
    dataset=ROOT/'dataset'
    if dataset.exists():raise RuntimeError('Dataset exists. Preserve it and choose a new output before rebuilding.')
    for split in ['train','val','test']:
        (dataset/'images'/split).mkdir(parents=True);(dataset/'labels'/split).mkdir(parents=True)
    for row in records:
        frame=cv2.imread(str(ROOT/'candidates'/'frames'/row['frame']));h,w=frame.shape[:2]
        lines=[]
        for obj in row['objects']:
            polygon=np.asarray(obj['polygon'],float)/[w,h]
            lines.append(str(obj['class_id'])+' '+' '.join(f'{v:.6f}' for v in polygon.ravel()))
        shutil.copy2(ROOT/'candidates'/'frames'/row['frame'],dataset/'images'/row['split']/row['frame'])
        (dataset/'labels'/row['split']/(Path(row['frame']).stem+'.txt')).write_text('\n'.join(lines)+('\n' if lines else ''),encoding='utf-8')
    config={'path':str(dataset.resolve()).replace('\\','/'),'train':'images/train','val':'images/val','test':'images/test',
            'names':{i:f'Pieza_{i+1:02d}' for i in range(8)}}
    (dataset/'data.yaml').write_text(yaml.safe_dump(config,sort_keys=False),encoding='utf-8')
    (dataset/'source_manifest.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
    report=audit_dataset(dataset,records,info,excluded,args.block_seconds,args.guard)
    print(json.dumps(report,indent=2),flush=True)
    if not report['ok']:raise RuntimeError('Dataset audit failed.')

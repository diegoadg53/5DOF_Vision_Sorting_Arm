"""Propose classes from user-verified examples and recording sections.

The user confirmed the part pairs. Section cut times were refined from visual
track transitions. Proposals remain pending until the labeled galleries are reviewed.
"""
from pathlib import Path
import json
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parent
PHASES=[(0,600,[1]),(600,781,[2,4]),(781,1022,[3,7]),
        (1022,1180,[6,7]),(1180,1274,[5,7]),(1274,1500,[0,5])]


if __name__=='__main__':
    data=json.loads((ROOT/'candidates'/'annotations_proposed.json').read_text(encoding='utf-8'))
    seeds=json.loads((ROOT/'review'/'verified_class_seeds.json').read_text(encoding='utf-8'))
    seed_map={s['track_id']:s['class_id'] for s in seeds}
    # This distinct screen/electronics assembly is visibly the same Pieza_01
    # identified by the user; it adds a complete-looking appearance to seed O.
    seed_map[444]=0
    cache=np.load(ROOT/'review'/'track_embeddings.npz')
    v=cache['vectors'];tids=cache['keys'][:,0].astype(int)
    prototypes={cid:v[np.isin(tids,[tid for tid,label in seed_map.items() if label==cid])] for cid in range(8)}
    results=[]
    for track in data['tracks']:
        tid=track['id'];obs=track['observations'];start=obs[0]['time'];end=obs[-1]['time'];mid=(start+end)/2
        allowed=set()
        for lo,hi,cids in PHASES:
            if lo-1.5<=mid<hi+1.5:allowed.update(cids)
        features=v[tids==tid]
        scores={}
        for cid in sorted(allowed):
            similarities=features@prototypes[cid].T
            nearest=np.max(similarities,axis=1)
            # Crop rotations and some slightly occluded observations can score lower.
            scores[cid]=float(np.mean(np.sort(nearest)[len(nearest)//4:]))
        ranking=sorted(scores,key=scores.get,reverse=True)
        chosen=seed_map.get(tid,ranking[0])
        margin=scores[ranking[0]]-scores[ranking[1]] if len(ranking)>1 else 1.0
        score=scores.get(chosen,0)
        status='user_verified' if tid in seed_map and tid!=444 else 'pending'
        results.append({'track_id':tid,'class_id':chosen,'class_name':f'Pieza_{chosen+1:02d}',
                        'start_seconds':start,'end_seconds':end,'observations':len(obs),
                        'similarity':score,'margin':margin,'candidate_scores':scores,
                        'status':status,'face':None})
    path=ROOT/'review'/'class_proposals.json'
    path.write_text(json.dumps({'phases':PHASES,'phase_pairs_verified_by':'user',
                                'proposals':results},indent=2),encoding='utf-8')
    print('Proposed track counts:',{cid:sum(r['class_id']==cid for r in results) for cid in range(8)})
    ambiguous=[r for r in results if r['margin']<.006 or r['similarity']<.85]
    (ROOT/'review'/'ambiguous_classes.json').write_text(json.dumps(ambiguous,indent=2),encoding='utf-8')
    print('Ambiguous tracks',len(ambiguous))
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',15)
    galleries=ROOT/'review'/'labeled_tracks';galleries.mkdir(exist_ok=True)
    tiles=[]
    for r in results:
        crop_path=ROOT/'review'/'tracks'/f'T{r["track_id"]:03d}.jpg'
        if not crop_path.exists():continue
        tile=Image.new('RGB',(420,220),'#202329');tile.paste(Image.open(crop_path),(0,30))
        d=ImageDraw.Draw(tile)
        color='#ffe077' if r in ambiguous else '#8ff0ac'
        d.text((6,5),f'{r["class_name"]} score={r["similarity"]:.3f} margin={r["margin"]:.3f}',font=font,fill=color)
        tiles.append(tile)
    for page in range((len(tiles)+23)//24):
        subset=tiles[page*24:(page+1)*24];sheet=Image.new('RGB',(1260,220*((len(subset)+2)//3)),'#202329')
        for i,tile in enumerate(subset):sheet.paste(tile,((i%3)*420,(i//3)*220))
        sheet.save(galleries/f'classes_{page+1:02d}.jpg',quality=94)
    print('Saved',path)

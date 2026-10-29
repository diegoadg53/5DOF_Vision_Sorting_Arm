"""Evaluate the final ONNX with square black padding used by the DFRobot converter.

Confidence is chosen using validation frames only. Test frames remain a final
evaluation, and no result here substitutes for UINT8 camera verification.
"""
from pathlib import Path
from collections import Counter
import json
import cv2
import numpy as np
import yaml
from ultralytics import YOLO

ROOT=Path(__file__).resolve().parent


def camera_style_dataset():
    original=ROOT/'dataset'
    target=ROOT/'evaluation'/'camera_style_dataset'
    names={i:f'Pieza_{i+1:02d}' for i in range(8)}
    for split in ['val','test']:
        (target/'images'/split).mkdir(parents=True,exist_ok=True)
        (target/'labels'/split).mkdir(parents=True,exist_ok=True)
        for image in sorted((original/'images'/split).glob('*.jpg')):
            out=target/'images'/split/(image.stem+'.png')
            if out.exists() and (target/'labels'/split/(image.stem+'.txt')).exists():continue
            frame=cv2.imread(str(image));h,w=frame.shape[:2]
            side=max(h,w);top=(side-h)//2;left=(side-w)//2
            padded=cv2.copyMakeBorder(frame,top,side-h-top,left,side-w-left,
                                     cv2.BORDER_CONSTANT,value=(0,0,0))
            cv2.imwrite(str(out),cv2.resize(padded,(640,640)))
            lines=[]
            label=original/'labels'/split/(image.stem+'.txt')
            for line in label.read_text(encoding='utf-8').splitlines():
                fields=line.split();points=np.asarray(fields[1:],float).reshape(-1,2)
                points=(points*[w,h]+[left,top])/side
                lines.append(fields[0]+' '+' '.join(f'{v:.6f}' for v in points.ravel()))
            (target/'labels'/split/(image.stem+'.txt')).write_text('\n'.join(lines)+('\n' if lines else ''),encoding='utf-8')
    config={'path':str(target).replace('\\','/'),'train':'images/val','val':'images/val','test':'images/test','names':names}
    path=target/'data_eval.yaml';path.write_text(yaml.safe_dump(config,sort_keys=False),encoding='utf-8')
    return target,path


def metric_summary(metrics):
    summary={'overall':metrics.results_dict,'classes':{}}
    for index,cid in enumerate(metrics.box.ap_class_index.tolist()):
        summary['classes'][f'Pieza_{cid+1:02d}']={
            'box_ap50':float(metrics.box.ap50[index]),
            'box_ap50_95':float(metrics.box.ap[index]),
            'mask_ap50':float(metrics.seg.ap50[index]),
            'mask_ap50_95':float(metrics.seg.ap[index]),
            'box_precision':float(metrics.box.p[index]),'box_recall':float(metrics.box.r[index]),
            'mask_precision':float(metrics.seg.p[index]),'mask_recall':float(metrics.seg.r[index])}
    return summary


def operating_point(model,target,split,confidence):
    counts={cid:Counter() for cid in range(8)}
    backgrounds=0;background_false_positive_images=0
    # A list of paths is treated as one in-memory batch by LoadPilAndNumpy.
    # A directory source respects batch=1, required by this static ONNX export.
    for result in model.predict(source=str(target/'images'/split),stream=True,imgsz=640,
                                rect=False,batch=1,device='cpu',conf=confidence,
                                iou=.7,retina_masks=True,verbose=False):
        image=Path(result.path);label=target/'labels'/split/(image.stem+'.txt')
        truth=[]
        for line in label.read_text(encoding='utf-8').splitlines():
            fields=line.split();cid=int(fields[0]);poly=np.asarray(fields[1:],float).reshape(-1,2)*640
            mask=np.zeros((640,640),np.uint8);cv2.fillPoly(mask,[np.rint(poly).astype(np.int32)],1)
            truth.append((cid,mask));counts[cid]['ground_truth']+=1
        predicted=[]
        if result.masks is not None:
            masks=result.masks.data.cpu().numpy()>0
            for cid,score,mask in zip(result.boxes.cls.cpu().numpy().astype(int),result.boxes.conf.cpu().numpy(),masks):
                if mask.shape!=(640,640):mask=cv2.resize(mask.astype(np.uint8),(640,640),interpolation=cv2.INTER_NEAREST)>0
                predicted.append((cid,float(score),mask))
        used=set()
        for cid,score,mask in sorted(predicted,key=lambda x:x[1],reverse=True):
            choices=[]
            for index,(gtcid,gtmask) in enumerate(truth):
                if index in used or cid!=gtcid:continue
                inter=np.logical_and(mask,gtmask).sum();union=np.logical_or(mask,gtmask).sum()
                choices.append((float(inter/max(1,union)),index))
            best=max(choices,default=(0,-1))
            if best[0]>=.5:used.add(best[1]);counts[cid]['true_positive']+=1
            else:counts[cid]['false_positive']+=1
        for index,(cid,mask) in enumerate(truth):
            if index not in used:counts[cid]['false_negative']+=1
        if not truth:
            backgrounds+=1;background_false_positive_images+=bool(predicted)
    report={}
    for cid,c in counts.items():
        tp=c['true_positive'];fp=c['false_positive'];fn=c['false_negative']
        report[f'Pieza_{cid+1:02d}']={**dict(c),'precision':tp/max(1,tp+fp),'recall':tp/max(1,tp+fn)}
    return {'confidence':confidence,'mask_match_iou':.5,'per_class':report,
            'background_images':backgrounds,'background_false_positive_images':background_false_positive_images}


def prediction_preview(model,target,confidence):
    """Show deterministic early/late examples per class, without selecting by score."""
    options={cid:[] for cid in range(8)}
    for label in sorted((target/'labels'/'test').glob('*.txt')):
        for line in label.read_text(encoding='utf-8').splitlines():
            fields=line.split();cid=int(fields[0])
            points=np.asarray(fields[1:],float).reshape(-1,2)*640
            options[cid].append((label.stem,points))
    sheet=np.full((4*300,4*400,3),32,np.uint8)
    chosen=[]
    for cid in range(8):
        candidates=options[cid]
        for column,index in enumerate([0,len(candidates)-1]):
            stem,points=candidates[index]
            path=target/'images'/'test'/(stem+'.png')
            result=model.predict(str(path),imgsz=640,rect=False,device='cpu',
                                 conf=confidence,retina_masks=True,verbose=False)[0]
            overlay=result.plot(labels=False,boxes=False,line_width=2,font_size=10)
            gtmask=np.zeros((640,640),np.uint8)
            cv2.fillPoly(gtmask,[np.rint(points).astype(np.int32)],1)
            matches=[]
            if result.masks is not None:
                for prediction_index,mask in enumerate(result.masks.data.cpu().numpy()>0):
                    overlap=np.logical_and(mask,gtmask).sum()/max(1,np.logical_or(mask,gtmask).sum())
                    matches.append((float(overlap),prediction_index))
            overlap,prediction_index=max(matches,default=(0,-1))
            prediction='no overlapping prediction'
            if overlap>.1:
                predicted_class=int(result.boxes.cls[prediction_index])
                score=float(result.boxes.conf[prediction_index])
                prediction=f'Pred Pieza_{predicted_class+1:02d} {score:.2f} | IoU {overlap:.2f}'
            low=np.floor(points.min(0)-55).astype(int).clip(0,640)
            high=np.ceil(points.max(0)+55).astype(int).clip(0,640)
            crop=overlay[low[1]:high[1],low[0]:high[0]]
            scale=min(390/crop.shape[1],245/crop.shape[0])
            crop=cv2.resize(crop,None,fx=scale,fy=scale)
            row=cid//2;col=(cid%2)*2+column
            x=col*400+(400-crop.shape[1])//2;y=row*300+45
            sheet[y:y+crop.shape[0],x:x+crop.shape[1]]=crop
            cv2.putText(sheet,f'GT Pieza_{cid+1:02d} | {stem}',(col*400+10,row*300+18),
                        cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1,cv2.LINE_AA)
            cv2.putText(sheet,prediction,(col*400+10,row*300+38),
                        cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1,cv2.LINE_AA)
            chosen.append({'ground_truth_class':cid,'image':path.name,
                           'predicted_classes':result.boxes.cls.cpu().numpy().astype(int).tolist(),
                           'confidences':result.boxes.conf.cpu().numpy().tolist()})
    cv2.imwrite(str(ROOT/'DFRobot_Ready'/'test_predictions.jpg'),sheet)
    (ROOT/'DFRobot_Ready'/'preview_samples.json').write_text(json.dumps(chosen,indent=2),encoding='utf-8')


def write_delivery_report(report):
    counts=report['test_at_suggested_confidence']
    overall=report['test']['overall']
    text=[
        '# YOLO11n-seg delivery report', '',
        'The eight-class segmentation model is trained and exported for the DFRobot packaging stage.', '',
        '## Files to use', '',
        '- ONNX: `best.onnx` in this folder.',
        '- Dataset: `../dataset/` (contains `data.yaml`, `images/`, and `labels/`).',
        '- Packaging settings: YOLO data source, Segmentation task, YOLO11n model, 640 input size.',
        '- Class IDs 0 through 7 correspond to Pieza_01 through Pieza_08.', '',
        '## Final evaluation', '',
        'The final ONNX was evaluated on 160 test images from held-out temporal groups. '
        'Inputs use the square black padding and resizing implemented in the local DFRobot converter.', '',
        f"Mask mAP@50: {overall['metrics/mAP50(M)']:.4f}. "
        f"Mask mAP@50-95: {overall['metrics/mAP50-95(M)']:.4f}.",
        f"Validation-selected reference confidence: {report['suggested_confidence_from_validation']:.2f}, "
        'selected on validation frames using mean segmentation F1.', '',
        'The following precision/recall counts use that confidence and require the correct class '
        'and mask IoU of at least 0.50.', '',
        '| Part | Test instances | Mask AP50 | Mask AP50-95 | Precision | Recall | TP | FP | FN |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for name,metrics in report['test']['classes'].items():
        c=counts['per_class'][name]
        text.append(f"| {name} | {c['ground_truth']} | {metrics['mask_ap50']:.3f} | "
                    f"{metrics['mask_ap50_95']:.3f} | {c['precision']:.3f} | {c['recall']:.3f} | "
                    f"{c.get('true_positive',0)} | {c.get('false_positive',0)} | {c.get('false_negative',0)} |")
    text.extend(['',f"Empty-belt test images with a false positive: {counts['background_false_positive_images']} "
                 f"of {counts['background_images']}.", '',
                 'See `test_predictions.jpg` for deterministic early/late test examples of all eight classes. '
                 'The image captions show expected and predicted classes and mask overlap.', '',
                 '## Identity recognition limit', '',
                 'The export is suitable for packaging and camera trials, but reliable recognition '
                 'of all eight identities is not established. Pieza_05/Pieza_07 is the main weak pair: '
                 'the deterministic preview shows Pieza_07 being classified as Pieza_05 despite '
                 'good silhouette overlap. At the validation-selected reference confidence, '
                 f"Pieza_07 recall is {counts['per_class']['Pieza_07']['recall']:.1%}. "
                 'An overall mAP value must not be read as an eight-class recognition success rate.', '',
                 'Further data should include independent recordings with randomized part pairings, '
                 'all rotations, and both faces. For true mirrored parts, verify that each identity '
                 'has a camera-visible distinction with either face up; if two views look identical, '
                 'training cannot reliably separate them from those images alone. Distinguishing '
                 'marks visible on both faces or a controlled face-up presentation can resolve '
                 'that ambiguity if no existing visual feature does.', '',
                 '## Scope and remaining verification', '',
                 'All data comes from one recording with the same physical specimens. '
                 'These results do not establish performance on new specimens or changed conditions. '
                 'Masks were generated with SAM2.1-t, reviewed, and incomplete outlines excluded; '
                 'internal holes are filled in the external-polygon representation.', '',
                 'The ONNX graph, tensor shapes, and numerical agreement with PyTorch are checked in '
                 '`export_verification.json`. DFRobot packaging, UINT8 quantization, camera import, '
                 'camera latency, and camera accuracy have not been tested here. '
                 'On the camera, test all eight parts with either face up, particularly the mirrored '
                 'Pieza_02/Pieza_06 and Pieza_05/Pieza_07 pairs, as well as simultaneous parts.', '',
                 'Packaging reference: https://wiki.dfrobot.com/sen0638/docs/22604', ''])
    (ROOT/'DFRobot_Ready'/'MODEL_REPORT.md').write_text('\n'.join(text),encoding='utf-8')


if __name__=='__main__':
    target,data=camera_style_dataset()
    model=YOLO(str(ROOT/'DFRobot_Ready'/'best.onnx'),task='segment')
    common={'data':str(data),'imgsz':640,'batch':1,'device':'cpu','workers':0,
            'rect':False,'project':str(ROOT/'evaluation'),'exist_ok':True,'plots':True,'verbose':True}
    validation=model.val(split='val',name='onnx_black_padding_val',**common)
    macro=np.mean(validation.seg.f1_curve,axis=0)
    axis=validation.seg.px
    allowed=(axis>=.1)&(axis<=.8)
    confidence=float(axis[np.argmax(np.where(allowed,macro,-1))])
    confidence=round(confidence,2)
    test=model.val(split='test',name='onnx_black_padding_test',**common)
    report={'input_size':640,'padding_value':0,'runtime':'ONNX Runtime CPU FP32',
            'validation':metric_summary(validation),'test':metric_summary(test),
            'suggested_confidence_from_validation':confidence,
            'test_at_suggested_confidence':operating_point(model,target,'test',confidence),
            'limitations':'Same recording and physical specimens. No nncase UINT8 conversion or physical camera inference was performed.'}
    (ROOT/'DFRobot_Ready'/'accuracy_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    prediction_preview(model,target,confidence)
    write_delivery_report(report)
    print(json.dumps(report,indent=2),flush=True)

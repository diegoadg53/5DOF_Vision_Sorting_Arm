"""Create figures from the preserved run; do not train or change its artifacts.

Run from any directory with the recorded Python environment. A short CPU ONNX
inference pass reconstructs mask matches at the already selected confidence.
Counts must reproduce accuracy_report.json before new diagnostics are saved.
"""
from pathlib import Path
from collections import Counter
import csv
import hashlib
import importlib.metadata
import json
import shutil
import zipfile

import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.ticker import PercentFormatter, MaxNLocator

OUT = Path(__file__).resolve().parent
ROOT = OUT.parent
PIPE = ROOT / 'YOLO11n_seg_pipeline'
READY = PIPE / 'DFRobot_Ready'
REPORT = json.loads((READY / 'accuracy_report.json').read_text(encoding='utf-8'))
AUDIT = json.loads((PIPE / 'dataset/audit.json').read_text(encoding='utf-8'))
EXPORT = json.loads((READY / 'export_verification.json').read_text(encoding='utf-8'))
NAMES = [f'Pieza_{i+1:02d}' for i in range(8)]
CONF = REPORT['test_at_suggested_confidence']['confidence']
BLUE, TEAL, ORANGE, RED = '#2563a6', '#008578', '#d48806', '#c03446'
GRAY, INK = '#687383', '#1d2939'

plt.rcParams.update({
    'font.family': 'DejaVu Sans', 'font.size': 10.5,
    'axes.titlesize': 12, 'axes.labelsize': 10.5,
    'axes.spines.top': False, 'axes.spines.right': False,
    'axes.edgecolor': '#ccd3dc', 'text.color': INK,
    'axes.labelcolor': INK, 'xtick.color': GRAY, 'ytick.color': INK,
    'figure.facecolor': 'white', 'axes.facecolor': 'white',
    'svg.fonttype': 'none', 'savefig.facecolor': 'white',
})


def save(fig, stem, footer, *, bottom=.12, top=.86, left=.10):
    fig.subplots_adjust(left=left, right=.96, bottom=bottom, top=top,
                        hspace=.40, wspace=.28)
    fig.text(.05, .025, footer, fontsize=9, color=GRAY, va='bottom')
    fig.savefig(OUT / f'{stem}.png', dpi=220)
    fig.savefig(OUT / f'{stem}.svg')
    plt.close(fig)


def training_history():
    with (PIPE / 'runs/yolo11n_seg_640/results.csv').open(newline='') as f:
        rows = [{k.strip(): float(v) for k, v in row.items()}
                for row in csv.DictReader(f)]
    epochs = np.array([r['epoch'] for r in rows])
    fitness = np.array([r['metrics/mAP50-95(B)'] + r['metrics/mAP50-95(M)']
                        for r in rows])
    best = rows[int(np.argmax(fitness))]
    selected = int(best['epoch'])
    assert selected == 81 and int(epochs[-1]) == 106
    fig, axes = plt.subplots(2, 2, figsize=(12.8, 8.4))
    fig.suptitle('Training improved rapidly; epoch 81 was retained',
                 fontsize=18, fontweight='bold', x=.05, ha='left', y=.97)
    fig.text(.05, .915, 'YOLO11n-seg | 106 completed epochs | early stopping patience: 25',
             color=GRAY, fontsize=11)
    for ax, loss, title in zip(axes.flat[:3], ['box', 'seg', 'cls'],
                              ['Bounding-box loss', 'Segmentation loss', 'Classification loss']):
        ax.plot(epochs, [r[f'train/{loss}_loss'] for r in rows], color=BLUE,
                label='Training', lw=1.7)
        ax.plot(epochs, [r[f'val/{loss}_loss'] for r in rows], color=ORANGE,
                label='Validation', lw=1.6, alpha=.9)
        ax.set(title=title, xlabel='Epoch', ylabel='Loss (lower is better)')
        ax.legend(frameon=False, fontsize=9)
    ax = axes.flat[3]
    ax.plot(epochs, [r['metrics/mAP50(M)'] for r in rows], color=BLUE,
            label='Mask mAP@50', lw=1.8)
    ax.plot(epochs, [r['metrics/mAP50-95(M)'] for r in rows], color=TEAL,
            label='Mask mAP@50-95', lw=1.8)
    ax.set(title='Training-time validation mask metrics', xlabel='Epoch',
           ylabel='mAP (higher is better)', ylim=(0, 1.04))
    ax.yaxis.set_major_formatter(PercentFormatter(1))
    ax.legend(frameon=False, fontsize=9, loc='lower right')
    for ax in axes.flat:
        ax.axvline(selected, color=RED, ls='--', lw=1.2)
        ax.axvspan(selected, epochs[-1], color='#f0f3f7', zorder=-1)
        ax.grid(axis='y', color='#e9edf2', lw=.7)
        ax.set_xlim(1, epochs[-1])
    save(fig, '01_training_history',
         'Dashed line: selected checkpoint. Selection used combined box + mask mAP@50-95.\n'
         'These curves use training-time validation preprocessing; they are not the final black-padding ONNX evaluation.',
         bottom=.13, top=.83)
    return {'epochs_completed': len(rows), 'best_epoch': selected,
            'best_training_time_validation': best}


def dataset_balance():
    fig, axes = plt.subplots(1, 2, figsize=(13.5, 6.5),
                             gridspec_kw={'width_ratios': [2.3, 1]})
    fig.suptitle('The dataset has unequal coverage of the mirrored identities',
                 fontsize=17, fontweight='bold', x=.05, ha='left', y=.97)
    y = np.arange(8); left = np.zeros(8)
    for split, color, label in zip(['train', 'val', 'test'], [BLUE, TEAL, ORANGE],
                                   ['Train', 'Validation', 'Test']):
        values = np.array([AUDIT['class_instances_by_split'][split][n] for n in NAMES])
        axes[0].barh(y, values, left=left, height=.62, color=color, label=label)
        left += values
    for i, total in enumerate(left):
        axes[0].text(total+5, i, f'{int(total)}', va='center', fontsize=10)
    axes[0].set(yticks=y, yticklabels=NAMES, xlabel='Labeled object instances',
                xlim=(0, max(left)*1.15), title='Instances by class and split')
    axes[0].invert_yaxis(); axes[0].legend(frameon=False, loc='lower right')
    axes[0].grid(axis='x', color='#e9edf2'); axes[0].set_axisbelow(True)
    splits = ['Train', 'Validation', 'Test']
    values = [AUDIT['images_by_split'][s] for s in ['train', 'val', 'test']]
    axes[1].bar(splits, values, color=[BLUE, TEAL, ORANGE], width=.58)
    for i, value in enumerate(values):
        axes[1].text(i, value+20, str(value), ha='center', fontweight='bold')
    axes[1].set(ylabel='Full images', title='1,224 images in total', ylim=(0, 1000))
    axes[1].grid(axis='y', color='#e9edf2'); axes[1].set_axisbelow(True)
    save(fig, '02_dataset_balance',
         '1,387 labeled instances; 56 empty-belt images. An instance is not an independent physical specimen.\n'
         '20-second temporal blocks, 2-second guards, and disjoint tracks; every split still comes from one recording.',
         bottom=.17, top=.84)


def generalization():
    fig, axes = plt.subplots(1, 2, figsize=(13.5, 6.6))
    fig.suptitle('Pieza_05 and Pieza_07 lose accuracy on held-out test views',
                 fontsize=17, fontweight='bold', x=.05, ha='left', y=.97)
    fig.text(.05, .91, 'Final ONNX | black square padding | validation: 209 images | test: 160 images',
             fontsize=11, color=GRAY)
    y = np.arange(8)
    for ax, key, title in zip(axes, ['mask_ap50', 'mask_ap50_95'],
                              ['Mask AP50 by class', 'Mask AP50-95 by class']):
        for split, offset, color, label in [('validation', -.17, BLUE, 'Validation'),
                                            ('test', .17, TEAL, 'Test')]:
            values = [REPORT[split]['classes'][n][key]*100 for n in NAMES]
            ax.barh(y+offset, values, height=.29, color=color, label=label)
            for i, value in enumerate(values):
                ax.text(value+.7, i+offset, f'{value:.1f}', va='center', fontsize=8.7)
        ax.set(yticks=y, yticklabels=NAMES, xlim=(0, 112), xlabel='Average precision (%)', title=title)
        ax.invert_yaxis(); ax.set_xticks([0, 25, 50, 75, 100])
        ax.grid(axis='x', color='#e9edf2'); ax.set_axisbelow(True)
        for i in [4, 6]:
            ax.axhspan(i-.43, i+.43, color='#fff0ec', zorder=-1)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', bbox_to_anchor=(.55,.075),
               ncol=2, frameon=False, fontsize=10)
    save(fig, '03_validation_vs_test',
         'AP summarizes predictions across confidence thresholds; it is not the operating recognition success rate.\n'
         'Held-out groups share the original recording and specimens. Orange shading identifies the weaker mirrored pair.',
         bottom=.20, top=.83)


def operating_metrics():
    fig, ax = plt.subplots(figsize=(12.5, 6.7))
    fig.suptitle('Only 2 of 14 Pieza_07 instances were correctly identified at confidence 0.56',
                 fontsize=15.8, fontweight='bold', x=.05, ha='left', y=.97)
    fig.text(.05, .915, 'Final ONNX test | correct class AND mask IoU >= 0.50 | NMS IoU: 0.70',
             fontsize=11, color=GRAY)
    counts = REPORT['test_at_suggested_confidence']['per_class']
    y = np.arange(8)
    for key, offset, color, label in [('precision', -.17, BLUE, 'Precision'),
                                     ('recall', .17, TEAL, 'Recall')]:
        vals = [counts[n][key]*100 for n in NAMES]
        ax.barh(y+offset, vals, height=.29, color=color, label=label)
        for i, value in enumerate(vals):
            ax.text(value+.8, i+offset, f'{value:.1f}%', va='center', fontsize=10)
    labels = [f'{n}   (n={counts[n]["ground_truth"]})' for n in NAMES]
    ax.set(yticks=y, yticklabels=labels, xlim=(0, 113), xlabel='Precision / recall (%)')
    ax.invert_yaxis(); ax.set_xticks([0, 20, 40, 60, 80, 100])
    fig.legend(*ax.get_legend_handles_labels(), loc='lower center',
               bbox_to_anchor=(.55,.07), ncol=2, frameon=False, fontsize=10)
    ax.grid(axis='x', color='#e9edf2')
    ax.set_axisbelow(True)
    for i in [4, 6]:ax.axhspan(i-.43, i+.43, color='#fff0ec', zorder=-1)
    save(fig, '04_precision_recall',
         'n = test ground-truth instances. Confidence 0.56 was selected using validation segmentation F1, not test labels.\n'
         'This operating point exposes poor Pieza_07 recall despite the overall test mask mAP@50 of 90.5%.',
         bottom=.18, top=.83, left=.20)


def reconstruct_matches():
    from ultralytics import YOLO
    import torch
    torch.set_num_threads(4); cv2.setNumThreads(2)
    model_path = READY/'best.onnx'
    assert hashlib.sha256(model_path.read_bytes()).hexdigest() == EXPORT['onnx_sha256']
    target = PIPE/'evaluation/camera_style_dataset'
    assert len(list((target/'images/test').glob('*.png'))) == 160
    model = YOLO(str(model_path), task='segment')
    preview = json.loads((READY/'preview_samples.json').read_text())
    desired = {}
    for cid in [1, 4]:
        candidates = [r for r in preview if r['ground_truth_class'] == cid]
        desired[(candidates[-1 if cid == 4 else 0]['image'], cid)] = None
    matrix = np.zeros((9, 9), dtype=int)
    samples = {}; image_results = []; backgrounds = false_backgrounds = 0
    for result in model.predict(str(target/'images/test'), stream=True, imgsz=640,
                                rect=False, batch=1, device='cpu', conf=CONF,
                                iou=.7, retina_masks=True, verbose=False):
        filename = Path(result.path).name
        truth = []
        label = target/'labels/test'/(Path(filename).stem+'.txt')
        for line in label.read_text(encoding='utf-8').splitlines():
            fields = line.split(); cid = int(fields[0])
            points = np.array(fields[1:], float).reshape(-1, 2)*640
            mask = np.zeros((640, 640), np.uint8)
            cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], 1)
            truth.append({'class':cid, 'mask':mask, 'points':points})
        predicted = []
        if result.masks is not None:
            for cid, score, mask in zip(result.boxes.cls.cpu().numpy().astype(int),
                                        result.boxes.conf.cpu().numpy(),
                                        result.masks.data.cpu().numpy()>0):
                assert mask.shape == (640, 640)
                predicted.append({'class':int(cid), 'score':float(score), 'mask':mask})
        overlaps = np.zeros((len(predicted), len(truth)))
        for pi, pred in enumerate(predicted):
            for gi, gt in enumerate(truth):
                overlaps[pi,gi] = (np.logical_and(pred['mask'],gt['mask']).sum()
                                   / max(1,np.logical_or(pred['mask'],gt['mask']).sum()))
        used_p = set(); used_g = set(); pairs = []
        # Exactly reproduce the original class-aware TP pass first.
        for pi in sorted(range(len(predicted)), key=lambda i:predicted[i]['score'], reverse=True):
            choices = [(float(overlaps[pi,gi]),gi) for gi in range(len(truth))
                       if gi not in used_g and truth[gi]['class']==predicted[pi]['class']]
            overlap, gi = max(choices, default=(0,-1))
            if overlap >= .5:
                used_p.add(pi); used_g.add(gi); pairs.append((pi,gi))
        # Attribute remaining FP/FN to wrong identities where masks overlap.
        choices = [(float(overlaps[pi,gi]),pi,gi) for pi in range(len(predicted))
                   for gi in range(len(truth)) if pi not in used_p and gi not in used_g
                   and overlaps[pi,gi]>=.5]
        for overlap, pi, gi in sorted(choices, reverse=True):
            if pi in used_p or gi in used_g:continue
            used_p.add(pi); used_g.add(gi); pairs.append((pi,gi))
        for pi,gi in pairs:matrix[predicted[pi]['class'],truth[gi]['class']]+=1
        # Keep the first attributed error for each weak mirrored identity,
        # without choosing by confidence or visual quality.
        for pi,gi in pairs:
            cid=truth[gi]['class']
            if cid in [5,6] and cid not in samples and predicted[pi]['class']=={5:1,6:4}[cid]:
                samples[cid]={'image':filename,'frame':result.orig_img.copy(),
                             'truth':truth[gi],'prediction':predicted[pi],
                             'iou':float(overlaps[pi,gi])}
        for pi,pred in enumerate(predicted):
            if pi not in used_p:matrix[pred['class'],8]+=1
        for gi,gt in enumerate(truth):
            if gi not in used_g:matrix[8,gt['class']]+=1
        if not truth:
            backgrounds += 1; false_backgrounds += bool(predicted)
        for gi, gt in enumerate(truth):
            key = (filename,gt['class'])
            if key not in desired:continue
            overlap,pi = max([(float(overlaps[i,gi]),i) for i in range(len(predicted))],default=(0,-1))
            samples[gt['class']] = {'image':filename, 'frame':result.orig_img.copy(),
                'truth':gt, 'prediction':predicted[pi] if overlap>.1 else None, 'iou':overlap}
        image_results.append({'image':filename,
            'truth_classes':[g['class'] for g in truth],
            'predictions':[{'class':p['class'],'confidence':p['score']} for p in predicted],
            'matches':[{'predicted_class':predicted[pi]['class'], 'true_class':truth[gi]['class'],
                        'mask_iou':float(overlaps[pi,gi])} for pi,gi in pairs]})
    expected = REPORT['test_at_suggested_confidence']['per_class']
    for cid,name in enumerate(NAMES):
        tp = matrix[cid,cid]
        fp = matrix[cid,:].sum()-tp; fn = matrix[:,cid].sum()-tp
        assert (int(tp),int(fp),int(fn)) == tuple(expected[name].get(k,0) for k in
                                                 ['true_positive','false_positive','false_negative']),name
    assert backgrounds == 7 and false_backgrounds == 0
    assert len(samples)==4 and len(image_results)==160
    data = {'confidence':CONF,'mask_iou':.5,'nms_iou':.7,
            'rows':'predicted class; final row = unmatched ground truth',
            'columns':'true class; final column = unmatched prediction',
            'matching':'Correct-class matches first, descending confidence, then remaining wrong-class masks by descending IoU. This preserves the original TP/FP/FN counts.',
            'names':NAMES+['Background'], 'matrix':matrix.tolist(),
            'original_operating_counts_reproduced':True,'images':image_results}
    (OUT/'mask_confusion_data.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
    return matrix,samples


def confusion_figure(matrix):
    fig, ax = plt.subplots(figsize=(10.7, 9.2))
    fig.suptitle('Where test instances go at confidence 0.56',
                 fontsize=18,fontweight='bold',x=.05,ha='left',y=.975)
    fig.text(.05,.93,'Mask-based matches | correct class first, then remaining wrong identities | IoU >= 0.50',
             fontsize=10.5,color=GRAY)
    image=ax.imshow(matrix,cmap='Blues',vmin=0,vmax=max(1,matrix.max()))
    labels=NAMES+['Background']
    ax.set(xticks=np.arange(9),xticklabels=labels,yticks=np.arange(9),yticklabels=labels,
           xlabel='Ground-truth class',ylabel='Predicted class')
    plt.setp(ax.get_xticklabels(),rotation=40,ha='right',rotation_mode='anchor')
    for row in range(9):
        for col in range(9):
            value=matrix[row,col]
            ax.text(col,row,str(value) if value else '\u00b7',ha='center',va='center',
                    fontsize=12,color='white' if value>matrix.max()*.48 else INK if value else '#b5bdc7')
            if row!=col and value:
                ax.add_patch(Rectangle((col-.5,row-.5),1,1,fill=False,edgecolor=RED,lw=1.5))
    fig.colorbar(image,ax=ax,fraction=.04,pad=.025,label='Instances')
    save(fig,'05_mask_confusion_matrix',
         'Rows = predictions; columns = truth. Red borders show errors.\n'
         'Background column: unmatched predictions. Background row: unmatched truth.\n'
         'Derived from FP32 ONNX test inference; this is not the native box-based validator confusion matrix.',
         bottom=.22,top=.87)


def mirrored_examples(samples):
    fig,axes=plt.subplots(2,4,figsize=(13.8,7.4))
    fig.suptitle('A good outline can still have the wrong identity',fontsize=19,
                 fontweight='bold',x=.05,ha='left',y=.97)
    fig.text(.05,.915,'Top: camera image crop. Bottom: predicted mask. Ground-truth boundary is drawn in white.',
             fontsize=11,color=GRAY)
    examples=[]
    for col,cid in enumerate([1,5,4,6]):
        sample=samples[cid]; gt=sample['truth']; pred=sample['prediction']
        points=gt['points']; low=np.floor(points.min(0)-16).astype(int).clip(0,640)
        high=np.ceil(points.max(0)+16).astype(int).clip(0,640)
        rgb=sample['frame'][:,:,::-1].copy(); overlay=rgb.copy()
        correct=pred is not None and pred['class']==cid
        if pred is not None:
            color=np.array([0,133,120] if correct else [192,52,70])
            overlay[pred['mask']] = (.6*overlay[pred['mask']]+.4*color).astype(np.uint8)
        cv2.polylines(overlay,[np.rint(points).astype(np.int32)],True,(255,255,255),1)
        for ax,img in zip(axes[:,col],[rgb,overlay]):
            ax.imshow(img[low[1]:high[1],low[0]:high[0]],interpolation='nearest')
            ax.axis('off')
        axes[0,col].set_title(f'Expected: {NAMES[cid]}',fontweight='bold',fontsize=12,pad=8)
        predicted=NAMES[pred['class']] if pred else 'No prediction'
        score=f" | confidence {pred['score']:.2f}" if pred else ''
        axes[1,col].set_title(f'Predicted: {predicted}\nMask IoU {sample["iou"]:.2f}{score}',
                              fontsize=11,color=TEAL if correct else RED,pad=8)
        examples.append({'true_class':NAMES[cid],'image':sample['image'],
                         'predicted_class':predicted,'confidence':pred['score'] if pred else None,
                         'mask_iou':sample['iou']})
    save(fig,'06_mirrored_pair_examples',
         'Crops are enlarged for visibility. All examples are from the preserved held-out test set at confidence 0.56.\n'
         'Pieza_02/Pieza_05 are reference successes; Pieza_06/Pieza_07 are the first attributed test errors for each pair.',
         bottom=.12,top=.81)
    (OUT/'example_sources.json').write_text(json.dumps(examples,indent=2),encoding='utf-8')


def tables():
    cells=[]
    counts=REPORT['test_at_suggested_confidence']['per_class']
    for name in NAMES:
        m=REPORT['test']['classes'][name]; c=counts[name]
        cells.append([name,str(c['ground_truth']),f'{m["mask_ap50"]*100:.1f}',
            f'{m["mask_ap50_95"]*100:.1f}',f'{c["precision"]*100:.1f}',f'{c["recall"]*100:.1f}',
            *[str(c.get(k,0)) for k in ['true_positive','false_positive','false_negative']]])
    headers=['Part','Test n','Mask AP50\n(%)','Mask AP50-95\n(%)','Precision\n(%)','Recall\n(%)','TP','FP','FN']
    fig,ax=plt.subplots(figsize=(13.8,4.9));ax.axis('off')
    fig.suptitle('Final ONNX results by part',fontsize=19,fontweight='bold',x=.05,ha='left',y=.96)
    table=ax.table(cellText=cells,colLabels=headers,cellLoc='center',loc='center',
                   colWidths=[.14,.09,.13,.15,.13,.12,.07,.07,.07])
    table.auto_set_font_size(False);table.set_fontsize(11);table.scale(1,2.0)
    for (row,col),cell in table.get_celld().items():
        cell.set_edgecolor('white')
        if row==0:
            cell.set_facecolor(BLUE)
            cell.set_height(cell.get_height()*1.65)
            cell.get_text().set_color('white'); cell.get_text().set_weight('bold')
        else:cell.set_facecolor('#fff0ec' if row in [5,7] else '#eef3f8' if row%2 else '#f8fafc')
    save(fig,'08_per_class_results_table',
         'AP spans confidence thresholds. Precision/recall/TP/FP/FN: confidence 0.56, correct class and mask IoU >= 0.50.\n'
         '190 test instances in 160 images; FP32 ONNX CPU inference; same recording and physical specimens.',
         bottom=.16,top=.82)
    return cells


def write_gallery(run,cells,matrix):
    dataset_rows=[]
    for name in NAMES:
        vals=[AUDIT['class_instances_by_split'][s][name] for s in ['train','val','test']]
        dataset_rows.append('| '+name+' | '+' | '.join(map(str,vals))+' | '+str(sum(vals))+' |')
    values=REPORT['test']['overall']
    lines=[
        '# Training results: figures and tables','',
        'Prepared on 2026-10-28 from the preserved YOLO11n-seg experiment. '
        'No training, model selection, or camera deployment was performed to make these figures. '
        'A CPU inference pass reproduced the original test TP/FP/FN counts before deriving the mask confusion matrix.','',
        '**Main result:** the model exports correctly and segments many parts well, but '
        'mirrored identities are confused. At confidence 0.56, mask matching attributes '
        f'{matrix[4,6]} of 14 Pieza_07 instances to Pieza_05, and '
        f'{matrix[1,5]} of 13 Pieza_06 instances to Pieza_02. '
        'These are FP32 ONNX test results, not measured camera error rates.','',
        '## Experiment at a glance','',
        '| Item | Result |','|---|---|',
        '| Model / input | YOLO11n-seg / 640 x 640 |',
        '| Completed epochs / selected epoch | 106 / 81 |',
        '| Training / validation / test images | 855 / 209 / 160 |',
        '| Training / validation / test instances | 962 / 235 / 190 |',
        f'| Final ONNX test mask mAP@50 | {values["metrics/mAP50(M)"]*100:.2f}% |',
        f'| Final ONNX test mask mAP@50-95 | {values["metrics/mAP50-95(M)"]*100:.2f}% |',
        '| Reference confidence / mask matching IoU | 0.56 / 0.50 |',
        '| Pieza_07 true positives / ground truth | 2 / 14 at confidence 0.56 |',
        '| Empty-belt test images with false positives | 0 of 7 at confidence 0.56 |','',
        'mAP is average precision across classes and confidence levels, not the percentage '
        'of physical parts correctly identified in operation. Confidence 0.56 was selected '
        'on validation mean segmentation F1 and was not selected using the test results.','',
        '## Figures','',
    ]
    figures=[
        ('01_training_history','Training progress',
         'Training and validation losses plus training-time validation mask metrics. '
         'The dashed line marks epoch 81; checkpoint selection combined box and mask mAP@50-95. '
         'These training-time curves use different evaluation preprocessing from the final ONNX metrics.'),
        ('02_dataset_balance','Dataset coverage',
         'Labeled instances by class and split, and the full-image split. '
         'Multiple images from a tracked sequence are correlated; instance counts are not specimen counts.'),
        ('03_validation_vs_test','Validation versus test AP',
         'The exported model performs much worse on held-out Pieza_05 and Pieza_07 views than on validation views. '
         'Both evaluations use the final ONNX and black square padding; this is not a comparison of PyTorch against ONNX.'),
        ('04_precision_recall','Identity recognition at the reference threshold',
         'Precision and recall require both the correct class and mask IoU >= 0.50. '
         'The labels include test sample counts. Pieza_07 recall is 14.3% at confidence 0.56.'),
        ('05_mask_confusion_matrix','Mask-based error attribution',
         'Columns are ground truth and rows are predictions. Correct-class matches are assigned first '
         'in descending confidence, exactly as in the original operating-point evaluation. Remaining '
         'overlapping masks are then paired by descending IoU to attribute wrong identities. '
         'Unmatched predictions go in the background column; unmatched ground truth goes in the background row. '
         'This preserves the original per-class TP/FP/FN counts. It differs from the native Ultralytics '
         'validator matrix, which uses its own box-based matching settings.'),
        ('06_mirrored_pair_examples','Actual image examples',
         'Original RGB crops above; predicted masks below. Ground-truth boundaries are white. '
         'Pieza_02 and Pieza_05 use preserved deterministic reference successes. For Pieza_06 '
         'and Pieza_07, the first attributed test error for each mirrored pair is shown '
         'without selecting by prediction score or visual quality. '
         'Good outline overlap coexists with the wrong identity. These are test frames, not new camera captures.'),
    ]
    for stem,title,caption in figures:
        lines += [f'### {title}','',f'![{title}]({stem}.png)','',caption,'',
                  f'[High-resolution PNG]({stem}.png) | [Editable vector SVG]({stem}.svg)','']
    lines += ['### All eight classes: preserved test preview','',
              '![All eight test classes](07_all_class_predictions.jpg)','',
              'Copied unchanged from the original delivery. Deterministic early/late examples '
              'include their expected and predicted identities, confidence, and mask overlap.','',
              '## Per-class results table','',
              '| Part | Test n | Mask AP50 (%) | Mask AP50-95 (%) | Precision (%) | Recall (%) | TP | FP | FN |',
              '|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    lines += ['| '+' | '.join(row)+' |' for row in cells]
    lines += ['', 'AP columns span confidence thresholds; the remaining metrics use confidence 0.56 '
              'and mask IoU >= 0.50. A wrong identity contributes a false positive to the predicted '
              'class and a false negative to the true class. Zero errors in a small sample do not prove '
              'zero operational errors.','',
              '[Table as PNG](08_per_class_results_table.png) | [Table as SVG](08_per_class_results_table.svg)','',
              '## Dataset instances table','',
              '| Part | Train | Validation | Test | Total |',
              '|---|---:|---:|---:|---:|',*dataset_rows,'',
              'Full images: train 855, validation 209, test 160. Empty-belt images: 39, 10, and 7. '
              'The split uses 20-second blocks with two-second boundary guards and disjoint tracks. '
              'All sets still come from one video of the same specimens.','',
              '## Reproduction and evidence','',
              'Run `python generate_training_figures.py` in the recorded environment. '
              'The script reads the preserved pipeline and performs static batch-one ONNX inference '
              'on the already prepared black-padded test images. It writes only this summary folder '
              'and `../Training_Results.zip`; it does not retrain, reselect weights, or overwrite source reports. '
              'It verifies every test TP/FP/FN count against the original report before exporting new diagnostics.','',
              '- [Detailed training account](../README_ModelTraining.md).',
              '- [Original dataset audit](../YOLO11n_seg_pipeline/dataset/audit.json).',
              '- [Original epoch CSV](../YOLO11n_seg_pipeline/runs/yolo11n_seg_640/results.csv).',
              '- [Original final ONNX evaluation](../YOLO11n_seg_pipeline/DFRobot_Ready/accuracy_report.json).',
              '- [Original export verification](../YOLO11n_seg_pipeline/DFRobot_Ready/export_verification.json).',
              '- [New diagnostic matrix and image matches](mask_confusion_data.json).',
              '- [Example selection and scores](example_sources.json).',
              '- [Figure provenance](figure_manifest.json).','',
              'PNG figures are rendered at 220 dpi; SVG charts preserve editable vector text and shapes '
              '(photographs remain embedded raster images). The ZIP contains the figures, tables in this '
              'Markdown file, provenance, and generation script. Links to the original pipeline and '
              'regeneration require the surrounding project, which is not included in the ZIP.','']
    (OUT/'README.md').write_text('\n'.join(lines),encoding='utf-8')
    manifest={'prepared_date':'2026-10-28','architecture':'YOLO11n-seg',
              'source_onnx_sha256':EXPORT['onnx_sha256'], 'run':run,
              'reference_confidence':CONF,'mask_match_iou':.5,
              'test_counts_reproduced':True,
              'versions':{n:importlib.metadata.version(n) for n in
                          ['numpy','matplotlib','ultralytics','onnxruntime','torch']},
              'sources':['YOLO11n_seg_pipeline/runs/yolo11n_seg_640/results.csv',
                         'YOLO11n_seg_pipeline/dataset/audit.json',
                         'YOLO11n_seg_pipeline/DFRobot_Ready/accuracy_report.json',
                         'YOLO11n_seg_pipeline/DFRobot_Ready/export_verification.json'],
              'outputs':[p.name for p in sorted(OUT.iterdir()) if p.is_file()]}
    (OUT/'figure_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')


if __name__=='__main__':
    print('Building plots from preserved metrics...',flush=True)
    run=training_history(); dataset_balance(); generalization(); operating_metrics()
    print('Reconstructing mask matches from final ONNX (CPU)...',flush=True)
    matrix,samples=reconstruct_matches()
    print('All original TP/FP/FN counts reproduced.',flush=True)
    confusion_figure(matrix); mirrored_examples(samples); cells=tables()
    shutil.copy2(READY/'test_predictions.jpg',OUT/'07_all_class_predictions.jpg')
    write_gallery(run,cells,matrix)
    archive=ROOT/'Training_Results.zip'
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED) as z:
        for path in sorted(OUT.iterdir()):
            if path.is_file():z.write(path,arcname='Training_Results/'+path.name)
    print(json.dumps({'gallery':str(OUT/'README.md'),'zip':str(archive),
                      'files':len([p for p in OUT.iterdir() if p.is_file()]),
                      'matrix':matrix.tolist()},indent=2),flush=True)

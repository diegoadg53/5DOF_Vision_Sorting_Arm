"""Train the reviewed eight-class YOLO11n segmentation dataset and export ONNX.

Run only after build_dataset.py has passed its annotation and split checks.
"""
from pathlib import Path
import argparse
import hashlib
import json
import platform
import shutil

import cv2
import numpy as np
import torch
import ultralytics
from ultralytics import YOLO
import yaml

ROOT = Path(__file__).resolve().parent


def train(epochs, batch, resume):
    audit = json.loads((ROOT / 'dataset' / 'audit.json').read_text(encoding='utf-8'))
    if not audit['ok']:
        raise RuntimeError('Dataset audit has not passed. Review labels before training.')
    if resume:
        model = YOLO(str(ROOT / 'runs' / 'yolo11n_seg_640' / 'weights' / 'last.pt'))
        model.train(resume=True)
    else:
        run_dir = ROOT / 'runs' / 'yolo11n_seg_640'
        if (run_dir / 'weights').exists():
            raise RuntimeError('Training weights already exist. Use --resume or select a new run name.')
        model = YOLO(str(ROOT / 'yolo11n-seg.pt'))
        model.train(
            data=str(ROOT / 'dataset' / 'data.yaml'),
            project=str(ROOT / 'runs'), name='yolo11n_seg_640', exist_ok=True,
            epochs=epochs, imgsz=640, batch=batch, device=0,
            workers=4, cache='ram', seed=42, deterministic=True,
            patience=25, save=True, save_period=10,
            degrees=180.0, fliplr=0.0, flipud=0.0,
            scale=0.25, translate=0.1, shear=0.0, perspective=0.0,
            mosaic=0.5, close_mosaic=15, mixup=0.0, copy_paste=0.0,
            mask_ratio=2, plots=True, amp=True,
        )
    return Path(model.trainer.save_dir) / 'weights' / 'best.pt'


def validate_export(weights):
    model = YOLO(str(weights))
    if model.task != 'segment' or model.model.yaml.get('scale') != 'n':
        raise RuntimeError('Expected YOLO11 nano instance segmentation weights.')
    expected = {i: f'Pieza_{i+1:02d}' for i in range(8)}
    if model.names != expected:
        raise RuntimeError(f'Unexpected class mapping: {model.names}')
    eval_dir = ROOT / 'evaluation'
    eval_dir.mkdir(exist_ok=True)
    metrics = model.val(data=str(ROOT / 'dataset' / 'data.yaml'), split='test',
                        imgsz=640, batch=2, device=0, workers=0,
                        project=str(eval_dir), name='held_out_test',
                        exist_ok=True, plots=True)
    summary = {'metrics': metrics.results_dict,
               'box_map_by_class': metrics.box.maps.tolist(),
               'mask_map_by_class': metrics.seg.maps.tolist(),
               'box_ap50_by_present_class': metrics.box.ap50.tolist(),
               'mask_ap50_by_present_class': metrics.seg.ap50.tolist(),
               'precision_by_present_class': metrics.box.p.tolist(),
               'recall_by_present_class': metrics.box.r.tolist(),
               'present_class_indices': metrics.box.ap_class_index.tolist(),
               'names': model.names,
               'scope': 'Held-out temporal groups from one recording. Not an independent camera or quantized-device test.'}
    (eval_dir / 'test_metrics.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    onnx_path = Path(model.export(format='onnx', imgsz=640, batch=1,
                                 dynamic=False, nms=False, half=False,
                                 simplify=True, opset=12, device='cpu'))
    import onnx
    import onnxruntime as ort
    graph = onnx.load(str(onnx_path))
    onnx.checker.check_model(graph)
    session = ort.InferenceSession(str(onnx_path), providers=['CPUExecutionProvider'])
    input_info = session.get_inputs()[0]
    if input_info.shape != [1,3,640,640]:
        raise RuntimeError(f'Unexpected ONNX input: {input_info.shape}')
    samples = sorted((ROOT / 'dataset' / 'images' / 'test').glob('*.jpg'))[:3]
    comparisons=[]
    # Compare raw outputs with identical tensors, before thresholds and mask decoding.
    # Validation fuses layers under inference mode in this Ultralytics version.
    # Reload ordinary checkpoint tensors for CPU reference inference: MKLDNN
    # cannot use the version counter of those inference-mode fused tensors.
    reference = YOLO(str(weights))
    reference.model.cpu().float().eval()
    for sample in samples:
        frame = cv2.imread(str(sample))
        h,w = frame.shape[:2]
        ratio=min(640/w,640/h)
        resized=cv2.resize(frame,(round(w*ratio),round(h*ratio)))
        top=(640-resized.shape[0])//2;left=(640-resized.shape[1])//2
        padded=cv2.copyMakeBorder(resized,top,640-resized.shape[0]-top,
                                  left,640-resized.shape[1]-left,
                                  cv2.BORDER_CONSTANT,value=(114,114,114))
        tensor=np.ascontiguousarray(padded[:,:,::-1].transpose(2,0,1)[None],dtype=np.float32)/255.0
        outputs=session.run(None,{input_info.name:tensor})
        if len(outputs)!=2 or outputs[0].shape != (1,44,8400) or outputs[1].shape != (1,32,160,160):
            raise RuntimeError(f'Unexpected segmentation outputs: {[o.shape for o in outputs]}')
        if not all(np.isfinite(o).all() for o in outputs):
            raise RuntimeError('ONNX produced non-finite outputs.')
        with torch.no_grad():
            torch_out=reference.model(torch.from_numpy(tensor))
        if isinstance(torch_out[0], (tuple, list)):
            prediction=torch_out[0][0].detach().numpy()
            prototype=torch_out[0][1].detach().numpy()
        else:
            prediction=torch_out[0].detach().numpy()
            raw=torch_out[1]
            prototype=(raw['proto'] if isinstance(raw,dict) else raw[-1]).detach().numpy()
        errors=[float(np.max(np.abs(prediction-outputs[0]))),
                float(np.max(np.abs(prototype-outputs[1])))]
        if not np.allclose(prediction,outputs[0],atol=0.02,rtol=0.002) or not np.allclose(prototype,outputs[1],atol=0.002,rtol=0.002):
            raise RuntimeError(f'PyTorch / ONNX mismatch on {sample.name}: {errors}')
        comparisons.append({'image':sample.name,'max_abs_prediction_error':errors[0],
                            'max_abs_prototype_error':errors[1]})
    ready=ROOT / 'DFRobot_Ready'
    ready.mkdir(exist_ok=True)
    shutil.copy2(onnx_path,ready / 'best.onnx')
    shutil.copy2(weights,ready / 'best.pt')
    config=yaml.safe_load((ROOT / 'dataset' / 'data.yaml').read_text(encoding='utf-8'))
    config['path']=str((ROOT / 'dataset').resolve()).replace('\\','/')
    (ROOT / 'dataset' / 'data.yaml').write_text(yaml.safe_dump(config,sort_keys=False),encoding='utf-8')
    report={'architecture':'YOLO11n-seg','input_shape':input_info.shape,
            'output_shapes':[o.shape for o in session.get_outputs()],
            'opset':12,'dynamic':False,'nms':False,'classes':expected,
            'onnx_sha256':hashlib.sha256(onnx_path.read_bytes()).hexdigest(),
            'raw_output_comparisons':comparisons,'ultralytics':ultralytics.__version__,
            'torch':torch.__version__,'python':platform.python_version(),
            'dataset_directory':str((ROOT/'dataset').resolve()),
            'onnx_file':str((ready/'best.onnx').resolve()),
            'remaining':'Use the official DFRobot package generator, then validate the quantized model on HuskyLens 2.'}
    (ready/'export_verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    (ready/'PACKAGING_INSTRUCTIONS.txt').write_text(
        'DFRobot HUSKYLENS 2 Model Installation Package Generator\n\n'
        'Data source: YOLO\n'
        f'Dataset folder: {(ROOT / "dataset").resolve()}\n'
        f'ONNX model: {(ready / "best.onnx").resolve()}\n'
        'Task: Segmentation\nModel: YOLO11n\nInput size: 640\n'
        'Class IDs 0..7 correspond to Pieza_01..Pieza_08.\n'
        'Keep the generated installation ZIP filename and contents unchanged.\n'
        'Camera destination: Huskylens\\storage\\installation_package\n'
        'Then select Model Installation > Local Installation.\n\n'
        'The ONNX checks do not replace a quantized model test on the camera.\n',encoding='utf-8')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--epochs',type=int,default=120)
    parser.add_argument('--batch',type=int,default=2)
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--export-only',action='store_true')
    args=parser.parse_args()
    weights=ROOT/'runs'/'yolo11n_seg_640'/'weights'/'best.pt' if args.export_only else train(args.epochs,args.batch,args.resume)
    validate_export(weights)

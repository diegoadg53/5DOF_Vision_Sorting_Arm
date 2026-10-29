# Model training: eight conveyor parts with YOLO11n-seg

> **About this repository copy:** the source video, extracted frames, dataset images and labels, intermediate review files, training checkpoints and logs are not included because of their size, so links to them below do not resolve. The pipeline scripts, the configuration files, the delivered model in `YOLO11n_seg_pipeline/DFRobot_Ready/` and the result figures in `Training_Results/` are included.

Document prepared: **2026-10-28**. This document describes the saved training run and its supporting artifacts. Package versions and results below are the recorded versions and measurements for that run, rather than a description of whichever software is installed today.

## Contents

1. [Objective and outcome](#1-objective-and-outcome)
2. [Folder structure and files to use](#2-folder-structure-and-files-to-use)
3. [Why a separate pipeline was developed](#3-why-a-separate-pipeline-was-developed)
4. [Input media and class definitions](#4-input-media-and-class-definitions)
5. [Frame extraction and initial object proposals](#5-frame-extraction-and-initial-object-proposals)
6. [Class identification and visual verification](#6-class-identification-and-visual-verification)
7. [Complete-part segmentation and the outline repair](#7-complete-part-segmentation-and-the-outline-repair)
8. [Background images and complete scene labeling](#8-background-images-and-complete-scene-labeling)
9. [Dataset split and audit](#9-dataset-split-and-audit)
10. [Training environment and configuration](#10-training-environment-and-configuration)
11. [Checkpoint selection and completed training](#11-checkpoint-selection-and-completed-training)
12. [ONNX export and numerical verification](#12-onnx-export-and-numerical-verification)
13. [Evaluation using deployment-style preprocessing](#13-evaluation-using-deployment-style-preprocessing)
14. [DFRobot handoff and subsequent camera observations](#14-dfrobot-handoff-and-subsequent-camera-observations)
15. [Reproduction, inspection, and rerun boundaries](#15-reproduction-inspection-and-rerun-boundaries)
16. [Limitations and proposed improvements](#16-limitations-and-proposed-improvements)
17. [Evidence and references](#17-evidence-and-references)
18. [Figures and result tables](#18-figures-and-result-tables)

## 1. Objective and outcome

The project aimed to identify eight physical parts on a conveyor using a custom instance segmentation model on a **DFRobot HUSKYLENS 2**, targeting system version **1.4.1**. Multiple parts can be visible simultaneously, at different positions and rotations, and either face of a part can point upward.

The architecture requested for this work was **YOLO11n-seg**, also referred to in the discussion as YOLOv11n-seg. The selected model produces a class, bounding box, and instance mask for each detected object. Segmentation uses the RGB image for recognition; it does not classify only the binary outline.

The completed pipeline produced:

- A reviewed dataset containing **1,224 full images** and **1,387 labeled instances**.
- A trained eight-class YOLO11n-seg checkpoint, selected at **epoch 81** after training stopped at **epoch 106**.
- A verified, static **640 x 640, batch-one, FP32 ONNX** export.
- Dataset audits, annotation provenance, evaluation results, and packaging instructions.

**Reliable recognition of all eight identities was not achieved.** The main measured weakness was confusion between Pieza_05 and Pieza_07. Subsequent camera testing reported by the user also found Pieza_06 being classified as Pieza_02. Successful export and good segmentation outlines do not establish reliable identity recognition.

The pipeline did not implement physical coordinate calibration, orientation estimation, or persistent object tracking at camera inference time. The temporal tracks described below were used to prepare and split the training data.

## 2. Folder structure and files to use

All relative paths in this document are relative to `Entrenamiento_V3/`.

```text
Entrenamiento_V3/
|-- README_ModelTraining.md                 # This detailed account
|-- OriginalVideo/
|   |-- 19700101_080621_recovered.mp4
|   `-- Parts.jpeg
|-- yolo_auto_label_reviewer_v3/             # Original application, preserved
|-- yolo_auto_reviewer_4formats_v4_1/         # Original application, preserved
`-- YOLO11n_seg_pipeline/
    |-- README.md                           # Shorter pipeline overview
    |-- prepare_candidates.py
    |-- track_embeddings.py
    |-- propose_classes.py
    |-- refine_masks.py
    |-- repair_all_outlines.py
    |-- build_dataset.py
    |-- train_export.py
    |-- evaluate_delivery.py
    |-- environment_versions.json
    |-- yolo11n-seg.pt                       # Pretrained starting weights
    |-- sam2.1_t.pt                          # Annotation assistance model
    |-- candidates/                         # Proposed frames and tracks
    |-- review/                             # Review images and saved decisions
    |-- dataset/                            # FINAL dataset
    |-- dataset_initial_color_masks/        # Archived superseded dataset
    |-- runs/
    |   |-- yolo11n_seg_640/                 # FINAL training run
    |   `-- initial_color_masks_stopped/    # Superseded interrupted attempt
    |-- evaluation/
    |   |-- held_out_test/
    |   |-- test_metrics.json
    |   |-- camera_style_dataset/
    |   |-- onnx_black_padding_val/
    |   `-- onnx_black_padding_test/
    `-- DFRobot_Ready/
        |-- best.pt
        |-- best.onnx
        |-- export_verification.json
        |-- accuracy_report.json
        |-- MODEL_REPORT.md
        |-- PACKAGING_INSTRUCTIONS.txt
        |-- preview_samples.json
        `-- test_predictions.jpg
```

For packaging, use [the final ONNX](YOLO11n_seg_pipeline/DFRobot_Ready/best.onnx) and the **`YOLO11n_seg_pipeline/dataset/` folder**, which contains [data.yaml](YOLO11n_seg_pipeline/dataset/data.yaml), `images/`, and `labels/`. The pretrained `yolo11n-seg.pt` in the pipeline root is not the custom trained model. The initial color-mask dataset and interrupted run are historical records, not the delivered model.

## 3. Why a separate pipeline was developed

The two existing applications were inspected before preparing the final dataset:

| Application | Relevant behavior | Limitation for this recording |
|---|---|---|
| `yolo_auto_label_reviewer_v3` | Green-object proposals, manual review, detection and oriented-box dataset export | Did not provide the required instance segmentation training pipeline |
| `yolo_auto_reviewer_4formats_v4_1` | Added segmentation polygons and format validation | Selected the largest green contour, could automatically mark proposals approved, and did not solve complete labeling of multiple objects or the nongreen portions of Pieza_01 |

The v4.1 application stored segmentation contours separately from its editable oriented box: adjusting box corners was not a complete segmentation-mask editor. Its image-based split also did not provide the temporal separation needed for neighboring video frames.

The dedicated scripts handled multiple candidates, class verification, complete-part outlines, negative images, temporal splits, training, and export. Both original applications and the original media were preserved.

## 4. Input media and class definitions

The source video was [19700101_080621_recovered.mp4](OriginalVideo/19700101_080621_recovered.mp4).

| Property | Recorded value |
|---|---:|
| Resolution | 1280 x 720 pixels |
| Reported frame rate | Approximately 29.365484 frames/second |
| Reported frame count | 41,811 |
| Approximate duration from those values | 23 minutes 44 seconds |

Frame timestamps in the preparation records were calculated as `frame_id / source_fps`. They are positions within the recording, not wall-clock capture times. Unreadable frames were skipped by the extraction script.

The user confirmed that the recording used the intended camera, mounting position, lighting, and conveyor speed. The reference [Parts.jpeg](OriginalVideo/Parts.jpeg) shows Pieza_02 through Pieza_08 from left to right. Pieza_01 is the larger assembly with black and gray features and is not included in that reference image.

The class mapping is fixed throughout the YOLO dataset and exported model metadata:

| YOLO class ID | Physical identity | Definition |
|---:|---|---|
| 0 | Pieza_01 | Complete housing and attached mechanism, including nongreen regions |
| 1 | Pieza_02 | Distinct identity from its mirrored counterpart, Pieza_06 |
| 2 | Pieza_03 | Third part identity |
| 3 | Pieza_04 | Fourth part identity |
| 4 | Pieza_05 | Distinct identity from its mirrored counterpart, Pieza_07 |
| 5 | Pieza_06 | Mirrored counterpart of Pieza_02 |
| 6 | Pieza_07 | Mirrored counterpart of Pieza_05 |
| 7 | Pieza_08 | Eighth part identity |

Turning a physical part over does not change its class. Classes were not merged into mirrored-pair groups, and front/back views were not made separate output classes. Some reviewed tracks contain face descriptions, but the final split and evaluation were not explicitly stratified by face.

## 5. Frame extraction and initial object proposals

Implemented in [prepare_candidates.py](YOLO11n_seg_pipeline/prepare_candidates.py).

The requested sampling rate was approximately two frames per second. With this video's reported rate, the script selected every **15th source frame**, approximately 1.96 samples/second. It retained full-resolution RGB content as JPEG images at quality 94. Crops were used for review and annotation assistance, not as replacements for the final training scenes.

Initial proposals used OpenCV HSV color thresholding:

| Setting | Value |
|---|---|
| Hue interval | 35 through 85, in OpenCV's HSV scale |
| Minimum saturation | 100 |
| Minimum value/brightness | 25 |
| Proposal search area | Rows 45 through 639; original images retained in full |
| Morphological cleanup | One closing and one opening with a 3 x 3 elliptical kernel |
| Candidate contour area | 220 through 60,000 square pixels |
| Minimum bounding-box side | 9 pixels |
| Polygon simplification epsilon | Maximum of 0.6 pixels and 0.0015 x contour perimeter |

All qualifying external contours could become proposals, rather than retaining only the largest green object. Border-touching proposals were marked partial. Empty frames were also sampled periodically for later background review.

The scan retained **2,105 candidate images** and created **512 candidate tracks**, saved in [annotations_proposed.json](YOLO11n_seg_pipeline/candidates/annotations_proposed.json). These candidates were proposals, not accepted labels.

### Temporal association

Neighboring observations were associated with the Hungarian assignment algorithm, using centroid movement and area similarity:

- Maximum time since a previous observation: 1.1 seconds.
- Maximum candidate center displacement: 180 pixels.
- Maximum area ratio between observations: 3.5.
- Assignment cost: `distance + 25 * abs(log(area_ratio))`.

Track galleries showed several observations of each sufficiently long sequence. This made it easier to find identity changes, merged objects, partial views, and unstable proposals. These preparation tracks do not represent a deployed tracking model.

## 6. Class identification and visual verification

Class assignment combined user-confirmed reference examples, the recording sequence, feature-similarity proposals, and subsequent visual review.

### User-confirmed references

Two review sheets were presented to the user:

- [Examples A-P](YOLO11n_seg_pipeline/review/class_verification_A_P.jpg).
- [Examples Q-AB](YOLO11n_seg_pipeline/review/class_verification_Q_AB.jpg).

The **28 user-confirmed reference entries** are stored in [verified_class_seeds.json](YOLO11n_seg_pipeline/review/verified_class_seeds.json). The user also confirmed the sequence of part pairs. The boundaries below were refined from visible transitions; they are approximate annotation aids, not a camera operating rule.

| Time within recording | Expected identities |
|---|---|
| 0-600 seconds | Pieza_02 |
| 600-781 seconds | Pieza_03 and Pieza_05 |
| 781-1022 seconds | Pieza_04 and Pieza_08 |
| 1022-1180 seconds | Pieza_07 and Pieza_08 |
| 1180-1274 seconds | Pieza_06 and Pieza_08 |
| 1274 seconds to the end | Pieza_01 and Pieza_06 |

### Feature-based proposals

[track_embeddings.py](YOLO11n_seg_pipeline/track_embeddings.py) extracted descriptors from the pretrained YOLO11n segmentation backbone at layers 4, 6, and 10. For up to three observations per track, it:

1. Cropped the candidate with a small margin.
2. Replaced background outside a slightly dilated proposal mask with gray value 114.
3. Placed the object on a 224 x 224 canvas.
4. Evaluated eight rotations at 45-degree intervals, without reflections.
5. Normalized and cached the feature vectors in `review/track_embeddings.npz`.

[propose_classes.py](YOLO11n_seg_pipeline/propose_classes.py) compared those vectors with the confirmed reference tracks and limited class candidates using the recording section. Similarity and score margins flagged uncertain proposals. This was a nearest-reference review aid, not a separately trained eight-part classifier. A complete-looking Pieza_01 track was also added as an assistant-reviewed reference; it was not another independently user-identified sheet entry.

### Saved review decisions

The assistant reviewed labeled track galleries against the user's reference identities, corrected assignments, and excluded short or mixed tracks. [reviewed_tracks.json](YOLO11n_seg_pipeline/review/reviewed_tracks.json) records **321 accepted** and **191 excluded** tracks.

The user verified the reference identities and selected outline definitions; the user did not individually approve every propagated frame label or every repaired mask. The remaining visual review and JSON acceptance decisions were performed by the assistant. This distinction matters when assessing annotation confidence.

## 7. Complete-part segmentation and the outline repair

### Pieza_01 required a full-assembly mask

Green thresholding could find only portions of Pieza_01. The user confirmed that the segmentation should include its **complete screen housing and attached mechanism**, including black and gray regions, as a single instance.

[refine_masks.py](YOLO11n_seg_pipeline/refine_masks.py) used **SAM2.1-t** to propose the complete outline. SAM was used for dataset annotation; it is not part of the exported camera model.

The refinement procedure was:

1. Crop around the relevant green components; the Pieza_01 crop used an 85-pixel margin.
2. Estimate the local belt color from low-saturation pixels.
3. Initialize GrabCut with the green proposal as definite foreground, a belt-color difference cue, and definite-background crop borders.
4. Run three GrabCut iterations and retain the component overlapping the green seed.
5. Derive a SAM bounding-box prompt from that component.
6. Add a positive point inside the green seed, selected using its distance transform.
7. Run SAM, retain the connected component overlapping the seed, and convert its external contour back to source-image coordinates.
8. Flag crop-edge truncation and record how much of the green seed the result covered.

The stage generated **146 Pieza_01 mask proposals**. After visual review, **97 were accepted** and **49 excluded**, before later temporal and frame-level filtering. These are proposal counts, not the final number of Pieza_01 training instances.

### Why the first dataset and training attempt were superseded

The initial dataset used green-derived outlines for the seven green classes and SAM refinement for Pieza_01. It contained **1,290 images**: 950 train, 171 validation, and 169 test.

The user then identified incomplete outlines in both Pieza_05 preview images and the first Pieza_08 preview image. Shiny or dark regions belonged to the physical part but fell outside the green threshold. The initial training attempt was stopped. The dataset and run were preserved as `dataset_initial_color_masks/` and `runs/initial_color_masks_stopped/`.

### Repair applied to all seven green classes

[repair_all_outlines.py](YOLO11n_seg_pipeline/repair_all_outlines.py) reused the GrabCut/SAM helper with a **40-pixel crop margin** to refine every green-object annotation in the archived initial manifest. It preserved the reviewed class identities.

The repair processed **1,410 green-object outlines**. It recorded the original and replacement polygons, area ratio, seed coverage, old/new mask IoU, and truncation flags. Automatic review flags included:

- Crop-edge truncation.
- Seed coverage below 0.88.
- New/old area ratio below 0.65 or above 2.8.

Review sheets included flagged results, the largest changes, and examples from every green class. Additional visual checks looked for hands, rails, reflections, and incomplete silhouettes. The saved decisions in [all_parts_refined.json](YOLO11n_seg_pipeline/review/all_parts_refined.json) contain **1,340 accepted** and **70 excluded** proposals. Rejection of a green outline caused its entire scene to be omitted when needed; the final rebuild removed 66 images compared with the initial dataset.

[requested_outline_repairs.jpg](YOLO11n_seg_pipeline/review/requested_outline_repairs.jpg) shows the repaired versions of the specific examples originally questioned by the user. The saved conversation does not contain a later user approval of that repair sheet; acceptance was based on the assistant's subsequent review.

### Polygon convention

Each label represents an external silhouette. Internal holes are filled in this polygon representation. The RGB images still contain the actual holes, markings, shadows, and surface features. Filling a hole in the annotation does not remove that information from the model input.

## 8. Background images and complete scene labeling

Empty-belt candidates were visually reviewed. **68 images** were accepted as background candidates; **56** survived the final temporal filtering and split: 39 train, 10 validation, and 7 test. Their label files are empty.

One supposed negative containing a partially visible Pieza_01 was removed. Complete scene labeling was important: a retained frame with multiple parts needed a separate accepted polygon for every retained part. If a visible candidate remained unresolved, partial, or unaccepted, the builder excluded the whole frame rather than training it with an unlabeled part treated as background.

Small green fragments already covered by an accepted Pieza_01 assembly mask were not labeled as additional instances.

## 9. Dataset split and audit

Implemented in [build_dataset.py](YOLO11n_seg_pipeline/build_dataset.py).

Adjacent video frames are highly related. Randomly assigning individual images could put essentially the same passage in both training and testing. The final split therefore used:

- **20-second temporal blocks**.
- A **two-second guard interval** at both ends of each block.
- Exclusion of tracks crossing block boundaries.
- Assignment of each retained temporal block to only one split.

Using seed 42, the builder evaluated 25,000 random block assignments against target proportions of 70% train, 15% validation, and 15% test. Candidate assignments required at least 15 training instances and eight validation/test instances for each class. The objective balanced per-class proportions, with an additional image-count term. Because assignment was by block, final image proportions are not exactly 70/15/15.

### Final counts

| Split | Images | Empty-belt images | Labeled instances |
|---|---:|---:|---:|
| Train | 855 | 39 | 962 |
| Validation | 209 | 10 | 235 |
| Test | 160 | 7 | 190 |
| **Total** | **1,224** | **56** | **1,387** |

| Part | Train instances | Validation instances | Test instances |
|---|---:|---:|---:|
| Pieza_01 | 38 | 8 | 10 |
| Pieza_02 | 298 | 96 | 50 |
| Pieza_03 | 87 | 14 | 17 |
| Pieza_04 | 73 | 13 | 27 |
| Pieza_05 | 106 | 16 | 12 |
| Pieza_06 | 117 | 25 | 13 |
| Pieza_07 | 72 | 18 | 14 |
| Pieza_08 | 171 | 45 | 47 |

An instance is one labeled object in one image, not necessarily another physical specimen or independent passage.

### YOLO segmentation format

Each nonempty label line has this form:

```text
class_id x1 y1 x2 y2 x3 y3 ... xn yn
```

Coordinates are normalized using the original full image dimensions: `x_normalized = x / 1280`, `y_normalized = y / 720`. Each object occupies one line; this is a polygon label, not a five-field detection-box label.

[source_manifest.json](YOLO11n_seg_pipeline/dataset/source_manifest.json) preserves source frame IDs, timestamps, temporal groups, tracks, accepted polygons, methods, and split assignments. For repaired green objects it also retains the original color polygon.

[audit.json](YOLO11n_seg_pipeline/dataset/audit.json) passed with `ok: true` and no listed errors. The audit checked image/label pairs, label counts, valid class IDs, finite normalized coordinates, at least three polygon vertices, nonzero area, identical image hashes across splits, and track/group separation. These structural checks do not prove that every semantic label or boundary is correct.

The held-out sets still come from **one recording of the same physical specimens**. Temporal separation reduces leakage but does not establish generalization to new recordings, specimens, or all possible face/pose combinations.

## 10. Training environment and configuration

The recorded environment is in [environment_versions.json](YOLO11n_seg_pipeline/environment_versions.json).

| Component | Recorded version |
|---|---|
| Python | 3.13.5 |
| Ultralytics | 8.4.70 |
| PyTorch | 2.6.0+cu124 |
| OpenCV | 4.13.0.92 |
| NumPy | 2.3.1 |
| Pillow | 11.3.0 |
| SciPy | 1.16.3 |
| PyYAML | 6.0.2 |
| ONNX | 1.22.0 |
| ONNX Runtime | 1.27.0 |
| onnxslim | 0.1.94 |

Training ran on Windows using an **NVIDIA GeForce RTX 3050 Laptop GPU with 4 GB VRAM**. The model was initialized from pretrained `yolo11n-seg.pt` and fine-tuned on the eight classes, rather than trained from random weights.

The final run was launched from the pipeline folder with:

```powershell
python -u train_export.py --epochs 120 --batch 8 *> training.log
```

This is the historical command, not an instruction to overwrite the completed run.

### Important parameters

| Setting | Value used |
|---|---|
| Task | Instance segmentation |
| Training image size | 640 x 640 |
| Maximum epochs | 120 |
| Batch size | 8 |
| Device | CUDA device 0 |
| Data-loader workers | 4 |
| Image cache | RAM |
| Seed | 42 |
| Deterministic flag | Enabled |
| Automatic mixed precision | Enabled |
| Early-stopping patience | 25 epochs |
| Periodic checkpoint saving | Every 10 epochs |
| Mask ratio | 2 |
| Rotation augmentation | Up to +/-180 degrees |
| Translation | 0.1 |
| Scale augmentation | 0.25 |
| Mosaic probability | 0.5 |
| Close Mosaic | Last 15 scheduled epochs |
| Horizontal / vertical flips | Both 0 |
| MixUp / Copy-Paste | Both 0 |
| Shear / perspective augmentation | Both 0 |

The run also inherited Ultralytics defaults, including HSV augmentation settings `hsv_h=0.015`, `hsv_s=0.7`, and `hsv_v=0.4`. The complete resolved argument snapshot is [args.yaml](YOLO11n_seg_pipeline/runs/yolo11n_seg_640/args.yaml); not every field in that generic snapshot is active for segmentation.

The optimizer was set to `auto`. The training log records the effective choice as **AdamW with initial learning rate 0.000833 and momentum/beta1 0.9**, using weight decay 0.0005 on the decayed parameter group. Ultralytics explicitly ignored the nominal `lr0=0.01` and `momentum=0.937` defaults when making this automatic selection. Those defaults should not be reported as the effective optimizer settings.

Reflections were disabled because a reflected image of a chiral part can change the visual handedness while retaining the wrong class label. A physical opposite-face view must be captured and labeled with the same physical identity; it is not safely generated by simply reflecting a single-face image.

Although a seed and deterministic flag were recorded, RAM caching produced a nondeterminism warning. This documents the experiment but does not guarantee bit-identical retraining across runs, GPUs, or library versions.

## 11. Checkpoint selection and completed training

The final run is stored in `YOLO11n_seg_pipeline/runs/yolo11n_seg_640/`.

- Planned maximum: 120 epochs.
- Completed: **106 epochs**.
- Stop reason: no improvement in checkpoint fitness for 25 epochs.
- Best checkpoint: **epoch 81**.
- Logged training duration: **1.562 hours**, approximately 93.7 minutes, excluding data preparation and subsequent export/evaluation work.

The checkpoint was selected using validation metrics, not test-set performance. In the recorded Ultralytics implementation, segmentation fitness combines box mAP@50-95 and mask mAP@50-95. The final delivered weights are the best checkpoint, not automatically the last epoch.

Evidence is preserved in [results.csv](YOLO11n_seg_pipeline/runs/yolo11n_seg_640/results.csv), `training.log`, `args.yaml`, the run's plots, and `weights/`. The completed `best.pt` and `last.pt` have their optimizer state stripped by the training finalization; periodic training checkpoints are also retained.

## 12. ONNX export and numerical verification

Implemented in [train_export.py](YOLO11n_seg_pipeline/train_export.py).

Before export, the script checked that the loaded checkpoint was a nano segmentation model with exactly the expected eight class names. It evaluated the checkpoint on the held-out test split and saved `evaluation/test_metrics.json`.

The export settings were:

```python
model.export(
    format="onnx",
    imgsz=640,
    batch=1,
    dynamic=False,
    nms=False,
    half=False,
    simplify=True,
    opset=12,
    device="cpu",
)
```

The model is FP32. INT8/UINT8 conversion was left to DFRobot's packaging workflow. Nonmaximum suppression and mask decoding are not embedded as final detections in this ONNX graph; the consuming inference pipeline must perform the appropriate postprocessing.

| Tensor | Shape | Meaning |
|---|---|---|
| Input | `[1, 3, 640, 640]` | Batch-one RGB image, channels first, float input normalized to 0-1 |
| Prediction output | `[1, 44, 8400]` | Four box channels, eight class-score channels, and 32 mask coefficients |
| Prototype output | `[1, 32, 160, 160]` | Mask prototypes used with the coefficients to reconstruct instance masks |

Verification included ONNX graph checking, inference through ONNX Runtime's CPU provider, expected tensor shapes, and finite output values. Raw PyTorch and ONNX outputs were compared using identical tensors before confidence filtering and mask decoding.

The comparison used the first three sorted test filenames, all in the Pieza_02 portion of this dataset; it was not a balanced eight-class accuracy test. This particular comparison used square letterboxing with padding value **114**. Its purpose was numerical export agreement. The separate deployment-style evaluation below used **black padding**.

| Comparison image | Maximum absolute prediction error | Maximum absolute prototype error |
|---|---:|---:|
| `frame_006600.jpg` | 0.0016632080 | 0.0000383854 |
| `frame_006660.jpg` | 0.0014953613 | 0.0000354052 |
| `frame_006675.jpg` | 0.0015258789 | 0.0000330210 |

The configured `allclose` tolerances were prediction `atol=0.02, rtol=0.002` and prototype `atol=0.002, rtol=0.002`; the comparisons passed. This verifies those inputs and supports export integrity, rather than proving recognition accuracy on every possible image.

### Export-check repair

The first export itself succeeded, but the subsequent CPU reference check raised `RuntimeError: Inference tensors do not track version counter.` Validation had fused layers under inference mode in this environment. The reference check was corrected to reload the checkpoint into ordinary CPU tensors and use `torch.no_grad()` for comparison. The installed Ultralytics version's nested prediction/prototype return structure was also handled explicitly.

Export and verification were then rerun without retraining. The corrected check passed, and the delivery files were copied to `DFRobot_Ready/`. [export_verification.json](YOLO11n_seg_pipeline/DFRobot_Ready/export_verification.json) records the shapes, class mapping, package versions, comparison errors, and ONNX SHA-256:

```text
10baf9306cdc788171746e60c9060f5725f49aff273db56342b40c19acbfba23
```

## 13. Evaluation using deployment-style preprocessing

Implemented in [evaluate_delivery.py](YOLO11n_seg_pipeline/evaluate_delivery.py).

The local DFRobot converter source in the surrounding project used centered **black square padding**, followed by resize and RGB conversion. To evaluate that input convention, a separate dataset was generated under `evaluation/camera_style_dataset/`:

1. Start with the original 1280 x 720 validation or test image.
2. Add 280 black rows above and below to form a 1280 x 1280 square.
3. Resize the square to 640 x 640 and save it as PNG.
4. Transform the ground-truth polygon coordinates into the same padded coordinate system.

These are derived evaluation images, not additional training observations. The original dataset was retained. The derived YAML's `train` entry points at the validation directory only to provide a complete configuration; no training was performed on it.

The final `DFRobot_Ready/best.onnx` was evaluated using ONNX Runtime CPU FP32, `batch=1`, `rect=False`, and input size 640. The fixed batch size is required by the export. A prediction-loop failure caused by passing all paths as one in-memory batch was corrected by using a directory source that respects batch one; evaluation then completed successfully.

### Confidence selection and final counting

Confidence **0.56** was selected from the validation set by maximizing mean per-class segmentation F1 over candidate confidence values from 0.10 through 0.80, rounded to two decimals. Test labels were not used to choose this threshold.

At that threshold, a separate test counting pass used NMS IoU 0.7 and full-size reconstructed masks. A prediction counted as a true positive only when it had the correct class and mask IoU of at least 0.50 with an unmatched ground-truth instance. Matching was greedy in descending prediction confidence. Wrong identities or unmatched predictions counted as false positives; unmatched ground-truth instances counted as false negatives.

### Overall metrics

| Evaluation | Images | Mask mAP@50 | Mask mAP@50-95 |
|---|---:|---:|---:|
| Final ONNX, validation, black square padding | 209 | 0.9802 | 0.8567 |
| Final ONNX, test, black square padding | 160 | **0.9049** | **0.8150** |
| Earlier PyTorch test evaluation | 160 | 0.8995 | 0.7293 |

The PyTorch evaluation used the standard validator settings in `train_export.py`, including its rectangular-padding defaults, whereas the final ONNX evaluation used explicitly square, black-padded inputs. Preprocessing and mask-evaluation geometry differ. The change in these scores should not be interpreted as an accuracy improvement caused by ONNX export; the controlled identical-input comparison is the evidence for numerical equivalence.

### Final per-class results

AP values summarize predictions across confidence thresholds. Precision, recall, and TP/FP/FN below refer specifically to the separate counting pass at confidence 0.56 and mask IoU >= 0.50.

| Part | Test instances | Mask AP50 | Mask AP50-95 | Precision | Recall | TP | FP | FN |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Pieza_01 | 10 | 0.9950 | 0.9271 | 0.9091 | 1.0000 | 10 | 1 | 0 |
| Pieza_02 | 50 | 0.9875 | 0.9326 | 0.9074 | 0.9800 | 49 | 5 | 1 |
| Pieza_03 | 17 | 0.9100 | 0.7312 | 1.0000 | 0.8235 | 14 | 0 | 3 |
| Pieza_04 | 27 | 0.9760 | 0.8474 | 0.9630 | 0.9630 | 26 | 1 | 1 |
| Pieza_05 | 12 | 0.7757 | 0.6709 | 0.4615 | 1.0000 | 12 | 14 | 0 |
| Pieza_06 | 13 | 0.9950 | 0.9548 | 1.0000 | 0.8462 | 11 | 0 | 2 |
| Pieza_07 | 14 | 0.6237 | 0.5667 | 0.5000 | **0.1429** | 2 | 2 | 12 |
| Pieza_08 | 47 | 0.9765 | 0.8894 | 0.9787 | 0.9787 | 46 | 1 | 1 |

Precision is `TP / (TP + FP)`; recall is `TP / (TP + FN)`. IoU is mask intersection area divided by mask union area. AP50 uses an IoU threshold of 0.50; AP50-95 averages across thresholds from 0.50 to 0.95. mAP averages AP over the classes and is not the percentage of physical parts correctly classified in operation.

No false-positive detections were observed in the **seven** empty-belt test images at the selected threshold. That is a small background sample, not proof of zero operational false positives.

[test_predictions.jpg](YOLO11n_seg_pipeline/DFRobot_Ready/test_predictions.jpg) shows deterministic early/late test examples for every class, with expected identity, predicted identity, confidence, and mask overlap. Examples were not selected for having high scores. The sheet includes Pieza_07 being classified as Pieza_05 despite good outline overlap.

The machine-readable results are in [accuracy_report.json](YOLO11n_seg_pipeline/DFRobot_Ready/accuracy_report.json), with a shorter explanation in [MODEL_REPORT.md](YOLO11n_seg_pipeline/DFRobot_Ready/MODEL_REPORT.md).

## 14. DFRobot handoff and subsequent camera observations

At the original handoff, the completed scope ended at dataset preparation, training, testing, and ONNX export. DFRobot packaging, quantization, and camera import were left to the user.

The recorded packaging selections were:

| Field | Selection |
|---|---|
| Data source | YOLO |
| Dataset folder | `YOLO11n_seg_pipeline/dataset/` |
| ONNX file | `YOLO11n_seg_pipeline/DFRobot_Ready/best.onnx` |
| Task | Segmentation |
| Model | YOLO11n |
| Input size | 640 |

The dataset folder, rather than only `data.yaml`, supplies images and labels for the official tool. The trained PyTorch checkpoint is retained for development but is not the ONNX input requested by the package generator.

[PACKAGING_INSTRUCTIONS.txt](YOLO11n_seg_pipeline/DFRobot_Ready/PACKAGING_INSTRUCTIONS.txt) records the absolute local paths and original installation steps: preserve the generated ZIP name and contents, copy it to `Huskylens\storage\installation_package`, and use Model Installation > Local Installation. Refer to the [official deployment guide](https://wiki.dfrobot.com/sen0638/docs/22604) for the tool and device procedure.

After the handoff, **the user reported testing the model on the camera**:

- Pieza_07 sometimes appeared as Pieza_05, while the reverse confusion was less evident.
- Both Pieza_05 and Pieza_07 struggled with a particular face upward.
- Pieza_06 often appeared as Pieza_02, while the reverse confusion was less evident.

These are user-reported operational observations, separate from the saved automated FP32 benchmark. No formal on-camera confusion matrix, latency measurement, or matched-image FP32-versus-quantized comparison was produced by the pipeline. Statements in the original export reports that camera inference was not performed refer to that automated pipeline scope; they do not negate the later user tests.

The model graph does not directly return an orientation angle. Extracting a rough axis from an accessible mask, mapping position into physical belt coordinates, and handling angle ambiguity would require additional processing and confirmation of the camera/API outputs available to the application.

## 15. Reproduction, inspection, and rerun boundaries

### Working directory

Commands below assume Windows PowerShell in the pipeline folder of this repository:

```powershell
Set-Location .\vision_training\YOLO11n_seg_pipeline
```

Use a compatible environment with the recorded dependencies. Merely installing a generic PyTorch package does not guarantee the CUDA-enabled build recorded above.

### Inspect the preserved artifacts

```powershell
Get-Content dataset\audit.json
Get-Content runs\yolo11n_seg_640\args.yaml
Get-Content DFRobot_Ready\export_verification.json
Get-Content DFRobot_Ready\MODEL_REPORT.md
Get-FileHash DFRobot_Ready\best.onnx -Algorithm SHA256
```

These commands inspect existing results and do not retrain the model.

### Preparation stages and their dependencies

| Script | Main required input | Main output or role |
|---|---|---|
| `prepare_candidates.py --fps 2` | Original video | Frames, proposals, candidate tracks, initial review galleries |
| `track_embeddings.py` | Candidate images/JSON and pretrained YOLO weights | Rotation descriptor cache |
| `propose_classes.py` | Descriptor cache, user reference JSON, recording-phase definitions | Proposed classes and labeled galleries |
| Visual review and saved JSON edits | User references, galleries, scene inspection | Accepted/excluded track decisions |
| `refine_masks.py` | Reviewed tracks, candidate frames, SAM weights | Pieza_01 mask proposals and galleries |
| `repair_all_outlines.py` | Archived initial dataset/manifest and SAM weights | Green-part replacement polygons for review |
| Visual review and saved JSON edits | Mask galleries and negative images | Accepted refinements and background decisions |
| `build_dataset.py` | Candidates plus all reviewed decision files | Final dataset, manifest, YAML, and audit |
| `train_export.py` | Reviewed final dataset and pretrained YOLO weights | Training run, test evaluation, ONNX, export verification |
| `evaluate_delivery.py` | Final ONNX and validation/test images/labels | Black-padding evaluation, operating-point counts, reports, previews |

**This was not a fully automatic, one-command annotation process.** Review decisions were saved through assistant visual inspection and JSON updates. Running a proposal script alone does not recreate those acceptance decisions or the user's class confirmations.

The scripts also reflect the final repaired state, not every earlier code revision. In particular, `repair_all_outlines.py` reads `dataset_initial_color_masks/`, which was created by the earlier builder before the full-outline requirement was added. The current builder already expects `review/all_parts_refined.json`. Therefore, starting in an empty folder and running all current scripts linearly will not reproduce the initial archive automatically. Preserve the archive and review artifacts, or adapt the preparation code for a new dataset.

The recording phase times, user seed track IDs, source video name, and existing mask decisions are specific to this recording. They must not be reused blindly to label another video.

### Build parameters used for the final split

```powershell
python build_dataset.py --block-seconds 20 --guard 2 --dry-run
```

The dry run previews selection without replacing `dataset/`. The actual historical build used the same explicit arguments without `--dry-run` after the preceding dataset had been preserved. The script's default guard is **1.5 seconds**, so reproducing this run requires explicitly passing **`--guard 2`**. The builder refuses to overwrite an existing dataset.

### Training and evaluation entry points

```powershell
# Historical final training command; existing weights make a fresh run refuse.
python -u train_export.py --epochs 120 --batch 8

# For an interrupted run that still has a resumable optimizer checkpoint:
python -u train_export.py --resume

# Re-evaluate and re-export the saved best checkpoint without retraining:
python -u train_export.py --export-only

# Evaluate the exported model with black square padding and regenerate reports:
python -u evaluate_delivery.py
```

The training script's default batch size is **2**; the completed run explicitly used **8**. The script hardcodes the `yolo11n_seg_640` run and checkpoint locations. To conduct a new experiment, use a separate preserved workspace/output configuration and update those references consistently rather than overwriting this delivery.

`--resume` is for a genuine interrupted checkpoint with optimizer state. It should not be presented as a way to resume the finalized, stripped `best.pt` or `last.pt` after this completed run.

`--export-only` still performs test evaluation, exports again, rewrites delivery files, and updates the dataset YAML path. `evaluate_delivery.py` rewrites evaluation reports/previews and reuses already generated square images when their paired labels exist. Preserve existing results before rerunning; if source labels or images change, its derived evaluation cache also needs deliberate regeneration in the new experiment.

No training, dataset rebuild, export, or camera installation was performed merely to create this README.

## 16. Limitations and proposed improvements

The saved run has several limitations:

- All data comes from one recording and the same physical specimens.
- Training instances are imbalanced, particularly Pieza_02 versus Pieza_06, and are correlated within tracked sequences.
- Both-face coverage was not explicitly balanced or measured in a separate face-stratified test.
- Part pairings were tied to particular recording sections, so context may be easier to learn than a general identity distinction.
- The difficult parts occupy approximately 40-60 pixels along their typical longer dimension after resizing to 640; fine distinguishing details may be poorly represented.
- Masks were generated with annotation models and reviewed; they are not independently hand-traced expert ground truth for every pixel.
- The threshold selected on validation generalized poorly to Pieza_07 in the test set. The JSON key `suggested_confidence_from_validation` is historical naming, not proof that 0.56 is a suitable operational threshold for every class.
- A high-confidence wrong identity is not corrected simply by raising the confidence threshold.

Following the camera tests, the user clarified that **identity and position are the priorities, rough orientation is sufficient, distinguishing marks cannot be added, lighting is already at the available maximum, and camera placement may be changed**.

The following were discussed as future work, but **were not implemented in the documented run**:

1. Compare overhead and modestly tilted camera views to determine whether molded features distinguish both mirrored pairs under both faces. The suggested 15- and 25-degree offsets from vertical were experimental starting points, not proven optimal settings.
2. Increase useful part resolution while keeping the required recognition area and complete largest part in view. A camera move also requires reassessment of physical coordinate calibration.
3. Collect balanced independent passages of Pieza_02, Pieza_06, Pieza_05, and Pieza_07, with both faces, varied rotations/positions, and randomized companions. Use a new recording for final testing.
4. Rebalance class/face sampling and compare reduced Mosaic/downscaling while retaining valid rotations and keeping reflection augmentation disabled.
5. Train a YOLO11n detection baseline on boxes derived from the same reviewed polygons, and compare it fairly with segmentation using identity errors, position quality, inference performance, and the eventual orientation method. No detection model comparison was completed here.
6. Compare matched failure images before and after DFRobot conversion to determine whether quantization or preprocessing adds errors. The existing Pieza_05/Pieza_07 issue already appeared in FP32 inference.

The work has not established that the mirrored views are inherently indistinguishable. If two allowed views genuinely contain the same visible information, however, additional examples or a different output head cannot recover a hidden physical identity. With marks unavailable, a useful change in viewpoint or another observable physical cue would be necessary for those cases.

## 17. Evidence and references

The authoritative records for this experiment are the saved scripts, manifests, reviews, logs, and reports:

- [Short pipeline README](YOLO11n_seg_pipeline/README.md).
- [User-confirmed class references](YOLO11n_seg_pipeline/review/verified_class_seeds.json).
- [Track review decisions](YOLO11n_seg_pipeline/review/reviewed_tracks.json).
- [Full-outline repair decisions](YOLO11n_seg_pipeline/review/all_parts_refined.json).
- [Final dataset audit](YOLO11n_seg_pipeline/dataset/audit.json) and [source manifest](YOLO11n_seg_pipeline/dataset/source_manifest.json).
- [Resolved training arguments](YOLO11n_seg_pipeline/runs/yolo11n_seg_640/args.yaml) and [epoch results](YOLO11n_seg_pipeline/runs/yolo11n_seg_640/results.csv).
- [Export verification](YOLO11n_seg_pipeline/DFRobot_Ready/export_verification.json).
- [Final ONNX accuracy results](YOLO11n_seg_pipeline/DFRobot_Ready/accuracy_report.json), [delivery report](YOLO11n_seg_pipeline/DFRobot_Ready/MODEL_REPORT.md), and [test preview](YOLO11n_seg_pipeline/DFRobot_Ready/test_predictions.jpg).

External documentation used for the training/deployment approach:

- [DFRobot HUSKYLENS 2 custom model deployment](https://wiki.dfrobot.com/sen0638/docs/22604).
- [Ultralytics YOLO11](https://docs.ultralytics.com/models/yolo11/).
- [Ultralytics instance segmentation](https://docs.ultralytics.com/tasks/segment/).
- [Ultralytics segmentation dataset format](https://docs.ultralytics.com/datasets/segment/).
- [Ultralytics ONNX/export workflow](https://docs.ultralytics.com/modes/export/).
- [Ultralytics SAM 2 annotation model](https://docs.ultralytics.com/models/sam-2/).

External documentation can change. The recorded versions, argument snapshots, output shapes, and measured results above describe this particular saved experiment.

## 18. Figures and result tables

A separate [visual results gallery](Training_Results/README.md) provides figures, captions, and tables derived from the saved run. [Training_Results.zip](Training_Results.zip) packages the figures, Markdown tables, provenance, and generation script for use in reports or presentations. New plots are available as 220-dpi PNG and editable SVG; the original eight-class prediction sheet is retained as JPEG.

| Figure / table | What it shows |
|---|---|
| [Training history](Training_Results/01_training_history.png) | Training/validation losses, validation mAP curves, and selected epoch 81 |
| [Dataset balance](Training_Results/02_dataset_balance.png) | Per-class instances and image split sizes |
| [Validation versus test](Training_Results/03_validation_vs_test.png) | AP50 and AP50-95 differences for every class using the final ONNX |
| [Precision and recall](Training_Results/04_precision_recall.png) | Per-class behavior at confidence 0.56 and mask IoU >= 0.50 |
| [Mask confusion matrix](Training_Results/05_mask_confusion_matrix.png) | Wrong identities and unmatched instances at that operating point |
| [Mirrored-pair examples](Training_Results/06_mirrored_pair_examples.png) | Original image crops and predictions, including both one-way identity errors |
| [All eight classes](Training_Results/07_all_class_predictions.jpg) | The preserved deterministic test preview from the original delivery |
| [Per-class results table](Training_Results/08_per_class_results_table.png) | Sample counts, AP, precision/recall, and TP/FP/FN in a reusable image |

The visual summary did not retrain or change the model. A CPU inference pass on the existing black-padded test images reproduced the original per-class TP/FP/FN counts exactly. A subsequent matching pass attributed remaining overlapping masks to wrong identities: at confidence 0.56, **12 of 14 Pieza_07 instances matched Pieza_05 predictions**, and **2 of 13 Pieza_06 instances matched Pieza_02 predictions**. These are FP32 ONNX test observations, not quantified on-camera error rates.

The mask confusion matrix deliberately preserves the original correct-class matches first, then pairs remaining masks by IoU; its method is documented in the gallery and [diagnostic data](Training_Results/mask_confusion_data.json). It should not be confused with the native Ultralytics box-based confusion matrix, which uses different matching settings.

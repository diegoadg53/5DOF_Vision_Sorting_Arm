# Eight-part YOLO11n segmentation pipeline

Target: HuskyLens 2 firmware 1.4.1, YOLO11n-seg, 640 x 640.

Class IDs 0 through 7 mean Pieza_01 through Pieza_08. Both faces retain the
same class. Horizontal and vertical flips and mirrored copy-paste are disabled.

## Source and annotation provenance

Source: `../OriginalVideo/19700101_080621_recovered.mp4`.
The original video and both original reviewer applications are preserved.

The user identified the A-P and Q-AB review examples and confirmed these
recording pairs: Pieza_02; Pieza_03 + Pieza_05; Pieza_04 + Pieza_08;
Pieza_07 + Pieza_08; Pieza_06 + Pieza_08; Pieza_01 + Pieza_06.
Section boundaries were refined from visible object transitions.

Rotation-only YOLO feature descriptors proposed class propagation; the labeled
track galleries were then visually reviewed and errors corrected. Mixed tracks,
short tracks and incomplete outlines are excluded. These review decisions are
saved, separately from the initial automatic proposals, under `review/`.

Green parts use crop-prompted SAM2.1-t external silhouettes. Color contours
serve only as prompt seeds: the user found that color-only masks omitted shiny
or dark regions of Pieza_05 and Pieza_08, so that initial dataset and training
attempt were archived and the full dataset was repaired before retraining.
The repair review excluded 70 object masks containing hands, rails, reflections
or incomplete boundaries. Internal holes are filled in this polygon
representation. Pieza_01 also uses a crop-prompted SAM2.1-t mask;
the user confirmed that its housing and attached mechanism form one instance.
All Pieza_01 mask galleries were inspected, and handling artifacts, rail pixels,
truncated masks and tracking swaps were rejected.

Every retained full frame has a separate polygon for every retained object.
Frames with an unresolved visible object are excluded rather than partially
labeled. Empty-belt examples were visually reviewed; a negative candidate with
a partially visible Pieza_01 was removed.

## Dataset

`dataset/data.yaml` is the training and packaging configuration.
`dataset/source_manifest.json` preserves video frame IDs, timestamps, object
tracks, polygons, methods and splits. `dataset/audit.json` records counts and
structural checks.

The repaired dataset has 855 train, 209 validation and 160 test images. All eight classes
occur in each split. Splits use 20-second temporal blocks, a two-second guard
on each block boundary, and exclude tracks crossing block boundaries.
Source image hashes and track/group assignments are checked for split overlap.

The split is a held-out evaluation of one recording. The same physical specimens
and camera conditions occur across splits, so the result does not establish
generalization to new specimens, new lighting or quantized camera inference.

## Training and export

Run `python train_export.py --epochs 120 --batch 8` in this folder.
Training uses the local RTX 3050 4 GB GPU and pretrained `yolo11n-seg.pt`.
Early stopping uses patience 25. Four data-loader workers and RAM image caching
use the available 32 GB system RAM. A fixed seed and training arguments are saved.
Use `--resume` to continue an interrupted run from its last checkpoint.

The script evaluates `best.pt` on the held-out test set and exports static,
batch-one FP32 ONNX with opset 12 and no embedded NMS. It validates the ONNX
graph, expected segmentation tensor shapes and raw PyTorch/ONNX numerical
agreement on representative test images.

Final model files and packaging instructions are written to `DFRobot_Ready/`.
The DFRobot tool should receive `DFRobot_Ready/best.onnx` and the `dataset/`
folder. Check task Segmentation, architecture YOLO11n, and input size 640.
Packaging, UINT8 conversion and physical camera verification remain subsequent
steps to perform with the official tool and HuskyLens 2.

## Completed run and measured limits

The repaired-data run completed 106 epochs through early stopping and selected
the checkpoint from epoch 81. `DFRobot_Ready/best.onnx` is static batch-one
YOLO11n-seg at 640, opset 12, FP32, without embedded NMS. Its graph and output
shapes pass validation, and raw outputs agree with a fresh PyTorch checkpoint.

`evaluate_delivery.py` evaluates the final ONNX using square black padding.
The 160-image held-out test gives mask mAP@50 0.9049 and mAP@50-95 0.8150.
These averages hide weak separation of Pieza_05 and Pieza_07: at the
validation-selected confidence 0.56, Pieza_07 recall is only 2/14, with test
examples classified as Pieza_05 despite good mask overlap. This is a model
for packaging and camera trials; reliable recognition of all eight identities
has not been established. See `DFRobot_Ready/MODEL_REPORT.md` and
`DFRobot_Ready/test_predictions.jpg` for the class-specific evidence.

Further improvement requires independent recordings with varied rotations,
both faces, and randomized part pairings. Mirrored identities must have a
visible distinction under the allowed presentation: visually identical views
cannot be reliably assigned different identities from shape alone.

References:
- https://wiki.dfrobot.com/sen0638/docs/22604
- https://docs.ultralytics.com/models/yolo11/
- https://docs.ultralytics.com/modes/export/

## Reproducibility

Preparation scripts are supplied for inspection and future data collection.
Do not rerun them over the reviewed artifacts without preserving the existing
review decisions. `build_dataset.py` refuses to overwrite an existing dataset.
The training script refuses to replace existing weights without `--resume`.
RAM caching can introduce run-to-run variation despite the recorded seed.

# YOLO11n-seg delivery report

The eight-class segmentation model is trained and exported for the DFRobot packaging stage.

## Files to use

- ONNX: `best.onnx` in this folder.
- Dataset: `../dataset/` (contains `data.yaml`, `images/`, and `labels/`).
- Packaging settings: YOLO data source, Segmentation task, YOLO11n model, 640 input size.
- Class IDs 0 through 7 correspond to Pieza_01 through Pieza_08.

## Final evaluation

The final ONNX was evaluated on 160 test images from held-out temporal groups. Inputs use the square black padding and resizing implemented in the local DFRobot converter.

Mask mAP@50: 0.9049. Mask mAP@50-95: 0.8150.
Validation-selected reference confidence: 0.56, selected on validation frames using mean segmentation F1.

The following precision/recall counts use that confidence and require the correct class and mask IoU of at least 0.50.

| Part | Test instances | Mask AP50 | Mask AP50-95 | Precision | Recall | TP | FP | FN |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Pieza_01 | 10 | 0.995 | 0.927 | 0.909 | 1.000 | 10 | 1 | 0 |
| Pieza_02 | 50 | 0.988 | 0.933 | 0.907 | 0.980 | 49 | 5 | 1 |
| Pieza_03 | 17 | 0.910 | 0.731 | 1.000 | 0.824 | 14 | 0 | 3 |
| Pieza_04 | 27 | 0.976 | 0.847 | 0.963 | 0.963 | 26 | 1 | 1 |
| Pieza_05 | 12 | 0.776 | 0.671 | 0.462 | 1.000 | 12 | 14 | 0 |
| Pieza_06 | 13 | 0.995 | 0.955 | 1.000 | 0.846 | 11 | 0 | 2 |
| Pieza_07 | 14 | 0.624 | 0.567 | 0.500 | 0.143 | 2 | 2 | 12 |
| Pieza_08 | 47 | 0.977 | 0.889 | 0.979 | 0.979 | 46 | 1 | 1 |

Empty-belt test images with a false positive: 0 of 7.

See `test_predictions.jpg` for deterministic early/late test examples of all eight classes. The image captions show expected and predicted classes and mask overlap.

## Identity recognition limit

The export is suitable for packaging and camera trials, but reliable recognition of all eight identities is not established. Pieza_05/Pieza_07 is the main weak pair: the deterministic preview shows Pieza_07 being classified as Pieza_05 despite good silhouette overlap. At the validation-selected reference confidence, Pieza_07 recall is 14.3%. An overall mAP value must not be read as an eight-class recognition success rate.

Further data should include independent recordings with randomized part pairings, all rotations, and both faces. For true mirrored parts, verify that each identity has a camera-visible distinction with either face up; if two views look identical, training cannot reliably separate them from those images alone. Distinguishing marks visible on both faces or a controlled face-up presentation can resolve that ambiguity if no existing visual feature does.

## Scope and remaining verification

All data comes from one recording with the same physical specimens. These results do not establish performance on new specimens or changed conditions. Masks were generated with SAM2.1-t, reviewed, and incomplete outlines excluded; internal holes are filled in the external-polygon representation.

The ONNX graph, tensor shapes, and numerical agreement with PyTorch are checked in `export_verification.json`. DFRobot packaging, UINT8 quantization, camera import, camera latency, and camera accuracy have not been tested here. On the camera, test all eight parts with either face up, particularly the mirrored Pieza_02/Pieza_06 and Pieza_05/Pieza_07 pairs, as well as simultaneous parts.

Packaging reference: https://wiki.dfrobot.com/sen0638/docs/22604

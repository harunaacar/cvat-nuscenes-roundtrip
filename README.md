# CVAT – nuScenes Round-Trip Converter

This project provides a conversion workflow between the **nuScenes autonomous driving dataset format** and **CVAT 3D annotation format**.

The goal is to import nuScenes LiDAR and camera data into CVAT, edit 3D annotations, export them from CVAT, convert the modified annotations back into nuScenes format, and re-import the result into CVAT to verify that the changes were preserved.

## Project Flow

```text
nuScenes
   ↓
nuscenes_to_cvat.py
   ↓
CVAT 3D Task
   ↓
Edit / Delete / Modify 3D Cuboids
   ↓
Export as Datumaro 1.0
   ↓
cvat_to_nuscenes_roundtrip.py
   ↓
Updated nuScenes Dataset
   ↓
nuscenes_to_cvat.py
   ↓
CVAT
   ↓
Verify Changes
```

## Repository Structure

```text
cvat-nuscenes-roundtrip/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── scripts/
│   ├── nuscenes_to_cvat.py
│   └── cvat_to_nuscenes_roundtrip.py
│
├── examples/
│   ├── scene-0061_mapping.json
│   └── scene-0061_labels.txt
│
└── docs/
    └── roundtrip-flow.md
```

Large nuScenes folders such as `samples/`, `sweeps/`, `maps/`, and the full `v1.0-mini` dataset should not be committed to the repository.

## Requirements

Python 3.12 was used during development.

```bash
pip install -r requirements.txt
```

Main dependencies:

```text
nuscenes-devkit
numpy
pyquaternion
Pillow
```

## nuScenes to CVAT

Run:

```bash
py -3.12 nuscenes_to_cvat.py --dataroot "C:\path\to\v1.0-mini" --scene scene-0061 --max-frames 5
```

Typical output:

```text
output/
├── scene-0061_cvat_media.zip
├── scene-0061_datumaro_annotations.zip
├── scene-0061_mapping.json
└── scene-0061_labels.txt
```

## Converter Output Files

### `scene-0061_cvat_media.zip`

Contains the sensor data uploaded to the CVAT 3D task.

```text
pointcloud/
├── 0000000000.pcd
├── 0000000001.pcd
└── ...

related_images/
├── 0000000000_pcd/
│   ├── CAM_FRONT.jpg
│   ├── CAM_FRONT_LEFT.jpg
│   ├── CAM_FRONT_RIGHT.jpg
│   ├── CAM_BACK.jpg
│   ├── CAM_BACK_LEFT.jpg
│   └── CAM_BACK_RIGHT.jpg
└── ...
```

The LiDAR point cloud is the main 3D frame. Camera images are attached as related/contextual images.

### `scene-0061_datumaro_annotations.zip`

Contains the nuScenes 3D annotations converted into Datumaro 1.0 format.

```text
annotations/
└── default.json
```

Important annotation fields include:

```text
cuboid_3d
position
rotation
scale
label_id
track_id
```

This file is imported after the 3D task has been created.

### `scene-0061_mapping.json`

This is one of the most important files in the round-trip workflow.

CVAT does not preserve all nuScenes-specific identifiers and sensor metadata. The mapping file keeps the relationship between CVAT and the original nuScenes dataset.

It stores relationships such as:

```text
CVAT frame
↔ nuScenes sample_token

CVAT track_id
↔ nuScenes instance_token

LiDAR frame
↔ LIDAR_TOP sample_data_token

Camera frame
↔ camera sample_data_token

Sensor
↔ calibrated_sensor

Sensor frame
↔ ego_pose
```

It also preserves calibration information such as:

```text
sensor translation
sensor rotation
camera intrinsic matrix
ego pose translation
ego pose rotation
```

Purpose:

```text
CVAT export
        +
mapping.json
        +
original nuScenes dataset
        ↓
reconstruct valid nuScenes annotations
```

Without this mapping, a CVAT `track_id` cannot reliably be mapped back to the original nuScenes `instance_token`.

### `scene-0061_labels.txt`

Contains the nuScenes class names used by the selected scene.

Example:

```text
human.pedestrian.adult
movable_object.barrier
movable_object.trafficcone
vehicle.car
vehicle.truck
vehicle.bus.rigid
```

These labels must exist in the CVAT task before the Datumaro annotation ZIP is imported.

## Importing into CVAT

Create a new task with:

```text
Dimension: 3D
```

Upload:

```text
scene-0061_cvat_media.zip
```

Create the labels listed in:

```text
scene-0061_labels.txt
```

Then:

```text
Actions
→ Upload annotations
→ Datumaro 1.0
```

Upload:

```text
scene-0061_datumaro_annotations.zip
```

## CVAT Export File

After editing annotations in CVAT, export them as:

```text
Datumaro 1.0
```

Example:

```text
annotation_after_track_delete.zip
```

This ZIP represents the current annotation state in CVAT and can include:

```text
deleted tracks
modified cuboid positions
modified cuboid dimensions
modified rotations
new annotations
track changes
```

This file is used together with:

```text
annotation_after_track_delete.zip
+
scene-0061_mapping.json
+
original nuScenes dataset
```

as input for the round-trip converter.

## CVAT to nuScenes

Run:

```bash
py -3.12 cvat_to_nuscenes_roundtrip.py ^
  --cvat-export "C:\path\to\annotation_after_track_delete.zip" ^
  --mapping "C:\path\to\scene-0061_mapping.json" ^
  --dataroot "C:\path\to\v1.0-mini" ^
  --out "C:\path\to\v1.0-mini-roundtrip"
```

The converter reconstructs the nuScenes annotation structure using the edited CVAT data.

## Round-Trip nuScenes Output

The converter creates a new nuScenes dataset, for example:

```text
v1.0-mini-roundtrip/
```

The original sensor data remains unchanged.

Files such as:

```text
samples/
sweeps/
maps/
sample.json
sample_data.json
calibrated_sensor.json
ego_pose.json
```

are preserved from the original nuScenes dataset.

The main files rebuilt from CVAT annotations are:

```text
sample_annotation.json
instance.json
```

### `sample_annotation.json`

Stores the updated 3D object annotations.

Important fields include:

```text
sample_token
instance_token
translation
size
rotation
prev
next
num_lidar_pts
num_radar_pts
```

CVAT cuboids are transformed from the local `LIDAR_TOP` coordinate system back into the nuScenes global coordinate system before being written here.

### `instance.json`

Stores object identity across multiple frames.

Important fields include:

```text
token
category_token
nbr_annotations
first_annotation_token
last_annotation_token
```

The converter rebuilds this file according to the tracks that remain after editing in CVAT.

If a track is deleted in CVAT, the reconstructed nuScenes dataset should no longer contain that deleted track.

### `roundtrip_report.json`

Summarizes the conversion process.

It can contain information such as:

```text
number of converted frames
number of rebuilt annotations
number of instances
source CVAT export
source mapping file
output dataset location
integrity check result
```

This file is useful for debugging and verification.

## Coordinate Transformation

CVAT cuboids are represented in the local `LIDAR_TOP` coordinate system.

nuScenes stores `sample_annotation` boxes in the global coordinate system.

During CVAT → nuScenes conversion:

```text
LiDAR coordinate
      ↓
calibrated_sensor
      ↓
Ego vehicle coordinate
      ↓
ego_pose
      ↓
Global nuScenes coordinate
```

The reverse transformation is used during nuScenes → CVAT conversion.

## Instance Tracking

nuScenes uses `instance_token` to identify the same object across frames.

CVAT uses `track_id`.

The converter maintains:

```text
instance_token ↔ track_id
```

It also rebuilds:

```text
nbr_annotations
first_annotation_token
last_annotation_token
prev
next
```

## Round-Trip Verification

Recommended validation flow:

```text
Original nuScenes
        ↓
Convert to CVAT
        ↓
Delete or modify a track
        ↓
Export from CVAT
        ↓
Convert back to nuScenes
        ↓
Convert generated nuScenes back to CVAT
        ↓
Reload into CVAT
        ↓
Export again
        ↓
Compare the two releases
```

Recommended release artifacts:

```text
Release 1:
annotation_after_track_delete.zip

Release 2:
annotation_after_roundtrip_reload.zip
```

Example verification:

```text
Delete a vehicle track in CVAT
        ↓
Export: annotation_after_track_delete.zip
        ↓
Convert back to nuScenes
        ↓
Convert nuScenes to CVAT again
        ↓
Reload task
        ↓
Export: annotation_after_roundtrip_reload.zip
        ↓
Verify the deleted track is still absent
```

## GitHub Releases

Suggested releases:

```text
v0.1-track-delete
└── annotation_after_track_delete.zip

v0.2-roundtrip-reload
└── annotation_after_roundtrip_reload.zip
```

Suggested description for `v0.1-track-delete`:

```text
CVAT Datumaro export after deleting a tracked object.
Used as the input of the CVAT → nuScenes round-trip conversion.
```

Suggested description for `v0.2-roundtrip-reload`:

```text
CVAT Datumaro export after converting the modified annotations back to nuScenes
and reloading the generated dataset into CVAT.

This release verifies that the deleted track remains deleted after the complete
round-trip conversion.
```

## Notes

The current implementation uses:

```text
LiDAR → CVAT 3D point cloud
Cameras → related/contextual images
nuScenes metadata → mapping.json
3D annotations → Datumaro cuboid_3d
```

Radar data is currently preserved as metadata rather than displayed as an independent CVAT sensor stream.

## Future Work

- Native nuScenes importer/exporter for CVAT
- Automatic projection of 3D LiDAR cuboids onto camera images
- Radar point cloud integration
- Multi-sensor calibration visualization
- Automated 3D annotation using models such as CenterPoint
- LiDAR-camera fusion models such as BEVFusion
- Additional round-trip consistency checks

## Dataset

Tested with:

```text
nuScenes v1.0-mini
scene-0061
```

Official references:

- nuScenes: https://www.nuscenes.org/
- CVAT: https://github.com/cvat-ai/cvat
- Datumaro: https://github.com/cvat-ai/datumaro

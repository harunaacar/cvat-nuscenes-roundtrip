import argparse
import json
import shutil
import zipfile
from pathlib import Path

import numpy as np
from PIL import Image
from nuscenes.nuscenes import NuScenes


CAMERA_CHANNELS = [
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_FRONT_LEFT",
]

RADAR_CHANNELS = [
    "RADAR_FRONT",
    "RADAR_FRONT_LEFT",
    "RADAR_FRONT_RIGHT",
    "RADAR_BACK_LEFT",
    "RADAR_BACK_RIGHT",
]


def zip_folder(folder: Path, zip_path: Path):
    if zip_path.exists():
        zip_path.unlink()

    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for file in folder.rglob("*"):
            if file.is_file():
                zf.write(file, file.relative_to(folder))


def collect_scene_samples(nusc: NuScenes, scene_name: str):
    scene = next((s for s in nusc.scene if s["name"] == scene_name), None)

    if scene is None:
        raise ValueError(f"Scene bulunamadı: {scene_name}")

    samples = []
    token = scene["first_sample_token"]

    while token:
        sample = nusc.get("sample", token)
        samples.append(sample)
        token = sample["next"]

    return scene, samples


def write_pcd_from_nuscenes_bin(src_bin: Path, dst_pcd: Path):
    points = np.fromfile(src_bin, dtype=np.float32)

    if points.size % 5 != 0:
        raise ValueError(f"Beklenmeyen nuScenes LiDAR formatı: {src_bin}")

    points = points.reshape((-1, 5))
    xyzi = points[:, :4].astype(np.float32, copy=False)

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {len(xyzi)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(xyzi)}\n"
        "DATA binary\n"
    ).encode("ascii")

    dst_pcd.parent.mkdir(parents=True, exist_ok=True)

    with open(dst_pcd, "wb") as f:
        f.write(header)
        f.write(xyzi.tobytes())


def get_sensor_metadata(nusc: NuScenes, sample_data_token: str):
    sd = nusc.get("sample_data", sample_data_token)
    calibrated = nusc.get("calibrated_sensor", sd["calibrated_sensor_token"])
    ego_pose = nusc.get("ego_pose", sd["ego_pose_token"])
    sensor = nusc.get("sensor", calibrated["sensor_token"])

    return {
        "sample_data_token": sample_data_token,
        "filename": sd["filename"],
        "timestamp": int(sd["timestamp"]),
        "channel": sensor["channel"],
        "modality": sensor["modality"],
        "calibrated_sensor_token": sd["calibrated_sensor_token"],
        "ego_pose_token": sd["ego_pose_token"],
        "calibrated_sensor": {
            "translation": [float(v) for v in calibrated["translation"]],
            "rotation": [float(v) for v in calibrated["rotation"]],
            "camera_intrinsic": calibrated.get("camera_intrinsic", []),
        },
        "ego_pose": {
            "translation": [float(v) for v in ego_pose["translation"]],
            "rotation": [float(v) for v in ego_pose["rotation"]],
        },
    }


def get_scene_categories(nusc: NuScenes, samples):
    categories = set()

    for sample in samples:
        for ann_token in sample["anns"]:
            ann = nusc.get("sample_annotation", ann_token)
            categories.add(ann["category_name"])

    categories = sorted(categories)
    label_to_id = {name: i for i, name in enumerate(categories)}

    return categories, label_to_id


def make_cuboid_annotation(
    nusc,
    ann_token,
    box,
    label_to_id,
    instance_to_track,
    ann_id,
):
    ann = nusc.get("sample_annotation", ann_token)
    instance_token = ann["instance_token"]

    if instance_token not in instance_to_track:
        instance_to_track[instance_token] = len(instance_to_track) + 1

    yaw, pitch, roll = box.orientation.yaw_pitch_roll

    x, y, z = [float(v) for v in box.center]
    width, length, height = [float(v) for v in box.wlh]

    return {
        "id": int(ann_id),
        "type": "cuboid_3d",
        "attributes": {
            "occluded": False,
            "track_id": int(instance_to_track[instance_token]),
            "keyframe": True,
            "outside": False,
            "instance_token": instance_token,
            "sample_annotation_token": ann_token,
            "visibility_token": ann.get("visibility_token", ""),
            "num_lidar_pts": int(ann.get("num_lidar_pts", 0)),
            "num_radar_pts": int(ann.get("num_radar_pts", 0)),
        },
        "group": 0,
        "label_id": int(label_to_id[ann["category_name"]]),
        "position": [x, y, z],
        "rotation": [float(roll), float(pitch), float(yaw)],
        "scale": [length, width, height],
    }


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--dataroot",
        default=r"C:\Users\Acarh\Desktop\v1.0-mini",
    )
    parser.add_argument(
        "--version",
        default="v1.0-mini",
    )
    parser.add_argument(
        "--scene",
        default="scene-0061",
    )
    parser.add_argument(
        "--out",
        default=None,
        help="Output klasörü. Verilmezse scriptin yanına cvat_nuscenes_output_v6 oluşturulur.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
    )

    args = parser.parse_args()

    dataroot = Path(args.dataroot).resolve()

    if args.out:
        out_root = Path(args.out).resolve()
    else:
        out_root = Path(__file__).resolve().parent / "cvat_nuscenes_output_v6"

    out_root.mkdir(parents=True, exist_ok=True)

    if not dataroot.exists():
        raise FileNotFoundError(f"nuScenes klasörü bulunamadı: {dataroot}")

    nusc = NuScenes(
        version=args.version,
        dataroot=str(dataroot),
        verbose=False,
    )

    scene, samples = collect_scene_samples(nusc, args.scene)

    if args.max_frames > 0:
        samples = samples[:args.max_frames]

    categories, label_to_id = get_scene_categories(nusc, samples)

    media_root = out_root / f"{args.scene}_cvat_media"
    datumaro_root = out_root / f"{args.scene}_datumaro"

    if media_root.exists():
        shutil.rmtree(media_root)

    if datumaro_root.exists():
        shutil.rmtree(datumaro_root)

    pointcloud_dir = media_root / "pointcloud"
    related_images_root = media_root / "related_images"
    annotations_dir = datumaro_root / "annotations"

    pointcloud_dir.mkdir(parents=True, exist_ok=True)
    related_images_root.mkdir(parents=True, exist_ok=True)
    annotations_dir.mkdir(parents=True, exist_ok=True)

    items = []
    frame_mapping = []
    instance_to_track = {}
    annotation_counter = 0

    print(f"Scene: {args.scene}")
    print(f"Frame sayısı: {len(samples)}")

    for frame_idx, sample in enumerate(samples):
        item_id = f"{frame_idx:010d}"

        lidar_token = sample["data"]["LIDAR_TOP"]
        lidar_sd = nusc.get("sample_data", lidar_token)

        src_lidar = dataroot / lidar_sd["filename"]
        pcd_name = f"{item_id}.pcd"
        dst_lidar = pointcloud_dir / pcd_name

        write_pcd_from_nuscenes_bin(src_lidar, dst_lidar)

        related_dir_name = f"{item_id}_pcd"
        related_dir = related_images_root / related_dir_name
        related_dir.mkdir(parents=True, exist_ok=True)

        related_images_json = []
        cameras = {}

        for channel in CAMERA_CHANNELS:
            cam_token = sample["data"].get(channel)

            if not cam_token:
                continue

            cam_sd = nusc.get("sample_data", cam_token)
            src_image = dataroot / cam_sd["filename"]

            if not src_image.exists():
                print(f"UYARI: kamera dosyası yok: {src_image}")
                continue

            ext = src_image.suffix.lower() or ".jpg"
            image_name = f"{channel}{ext}"
            dst_image = related_dir / image_name

            shutil.copy2(src_image, dst_image)

            with Image.open(src_image) as im:
                width, height = im.size

            related_images_json.append({
                "path": f"related_images/{related_dir_name}/{image_name}",
                "size": [int(width), int(height)],
            })

            cameras[channel] = get_sensor_metadata(nusc, cam_token)

        radars = {}

        for channel in RADAR_CHANNELS:
            radar_token = sample["data"].get(channel)

            if radar_token:
                radars[channel] = get_sensor_metadata(nusc, radar_token)

        _, boxes, _ = nusc.get_sample_data(
            lidar_token,
            selected_anntokens=sample["anns"],
        )

        box_by_token = {box.token: box for box in boxes}

        frame_annotations = []

        for ann_token in sample["anns"]:
            box = box_by_token.get(ann_token)

            if box is None:
                continue

            frame_annotations.append(
                make_cuboid_annotation(
                    nusc=nusc,
                    ann_token=ann_token,
                    box=box,
                    label_to_id=label_to_id,
                    instance_to_track=instance_to_track,
                    ann_id=annotation_counter,
                )
            )

            annotation_counter += 1

        item = {
            "id": item_id,
            "annotations": frame_annotations,
            "attr": {
                "frame": frame_idx
            },
            "point_cloud": {
                "path": f"pointcloud/{pcd_name}"
            },
        }

        if related_images_json:
            item["related_images"] = related_images_json

        items.append(item)

        frame_mapping.append({
            "frame": frame_idx,
            "item_id": item_id,
            "sample_token": sample["token"],
            "timestamp": int(sample["timestamp"]),
            "lidar": get_sensor_metadata(nusc, lidar_token),
            "cameras": cameras,
            "radars": radars,
            "annotation_tokens": list(sample["anns"]),
        })

        print(
            f"[{frame_idx + 1}/{len(samples)}] "
            f"{item_id} | cuboid={len(frame_annotations)} | camera={len(related_images_json)}"
        )

    datumaro_json = {
        "info": {},
        "categories": {
            "label": {
                "labels": [
                    {
                        "name": name,
                        "parent": "",
                        "attributes": [
                            "occluded",
                            "track_id",
                            "keyframe",
                            "outside",
                            "instance_token",
                            "sample_annotation_token",
                            "visibility_token",
                            "num_lidar_pts",
                            "num_radar_pts",
                        ],
                    }
                    for name in categories
                ],
                "attributes": ["occluded"],
            },
            "points": {
                "items": []
            },
        },
        "items": items,
    }

    default_json = annotations_dir / "default.json"

    with open(default_json, "w", encoding="utf-8") as f:
        json.dump(
            datumaro_json,
            f,
            indent=2,
            ensure_ascii=False,
        )

    out_root.mkdir(parents=True, exist_ok=True)

    mapping_path = out_root / f"{args.scene}_mapping.json"

    with open(mapping_path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "scene_name": args.scene,
                "scene_token": scene["token"],
                "version": args.version,
                "frame_count": len(samples),
                "categories": categories,
                "instance_token_to_track_id": instance_to_track,
                "track_id_to_instance_token": {
                    str(v): k for k, v in instance_to_track.items()
                },
                "frames": frame_mapping,
            },
            f,
            indent=2,
            ensure_ascii=False,
        )

    labels_path = out_root / f"{args.scene}_labels.txt"
    labels_path.write_text("\n".join(categories), encoding="utf-8")

    media_zip = out_root / f"{args.scene}_cvat_media.zip"
    datumaro_zip = out_root / f"{args.scene}_datumaro_annotations.zip"

    zip_folder(media_root, media_zip)
    zip_folder(datumaro_root, datumaro_zip)

    if not media_zip.exists():
        raise RuntimeError(f"Media ZIP oluşturulamadı: {media_zip}")

    if not datumaro_zip.exists():
        raise RuntimeError(f"Datumaro ZIP oluşturulamadı: {datumaro_zip}")

    print()
    print("==============================================")
    print("TAMAMLANDI")
    print("==============================================")
    print(f"Media ZIP    : {media_zip}")
    print(f"Datumaro ZIP : {datumaro_zip}")
    print(f"Mapping JSON : {mapping_path}")
    print(f"Labels TXT   : {labels_path}")
    print(f"Track sayısı : {len(instance_to_track)}")
    print(f"Cuboid sayısı: {annotation_counter}")
    print()
    print("ZIP KONTROL")
    print(f"Media ZIP exists    : {media_zip.exists()} | {media_zip.stat().st_size / (1024*1024):.2f} MB")
    print(f"Datumaro ZIP exists : {datumaro_zip.exists()} | {datumaro_zip.stat().st_size / 1024:.2f} KB")
    print(f"Output folder       : {out_root}")


if __name__ == "__main__":
    main()

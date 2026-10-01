import argparse
import json
import shutil
import uuid
import zipfile
from pathlib import Path

import numpy as np
from pyquaternion import Quaternion


def load_json(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path: Path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def extract_datumaro_json(export_zip: Path, temp_dir: Path):
    if temp_dir.exists():
        shutil.rmtree(temp_dir)
    temp_dir.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(export_zip, "r") as zf:
        zf.extractall(temp_dir)

    candidates = list(temp_dir.rglob("default.json"))
    if not candidates:
        raise FileNotFoundError("CVAT export ZIP içinde default.json bulunamadı.")

    return candidates[0]


def euler_xyz_to_quaternion(rx, ry, rz):
    qx = Quaternion(axis=[1, 0, 0], angle=float(rx))
    qy = Quaternion(axis=[0, 1, 0], angle=float(ry))
    qz = Quaternion(axis=[0, 0, 1], angle=float(rz))
    return qz * qy * qx


def lidar_box_to_global(position, rotation, lidar_meta):
    """
    CVAT cuboid LIDAR_TOP local -> nuScenes global.
    """
    p_lidar = np.array(position, dtype=np.float64)

    cs = lidar_meta["calibrated_sensor"]
    ego = lidar_meta["ego_pose"]

    q_ego_from_lidar = Quaternion(cs["rotation"])
    t_ego_from_lidar = np.array(cs["translation"], dtype=np.float64)

    q_global_from_ego = Quaternion(ego["rotation"])
    t_global_from_ego = np.array(ego["translation"], dtype=np.float64)

    p_ego = q_ego_from_lidar.rotate(p_lidar) + t_ego_from_lidar
    p_global = q_global_from_ego.rotate(p_ego) + t_global_from_ego

    rx, ry, rz = rotation
    q_lidar_box = euler_xyz_to_quaternion(rx, ry, rz)
    q_global_box = q_global_from_ego * q_ego_from_lidar * q_lidar_box

    return (
        [float(v) for v in p_global],
        [
            float(q_global_box.w),
            float(q_global_box.x),
            float(q_global_box.y),
            float(q_global_box.z),
        ],
    )


def main():
    parser = argparse.ArgumentParser(
        description="CVAT Datumaro 3D export + mapping.json -> valid nuScenes round-trip dataset"
    )

    parser.add_argument("--cvat-export", required=True)
    parser.add_argument("--mapping", required=True)
    parser.add_argument(
        "--dataroot",
        default=r"C:\Users\Acarh\Desktop\v1.0-mini",
    )
    parser.add_argument("--version", default="v1.0-mini")
    parser.add_argument(
        "--out",
        default=r"C:\Users\Acarh\Desktop\v1.0-mini-roundtrip",
    )

    args = parser.parse_args()

    cvat_export = Path(args.cvat_export).resolve()
    mapping_path = Path(args.mapping).resolve()
    dataroot = Path(args.dataroot).resolve()
    out_root = Path(args.out).resolve()

    for p in [cvat_export, mapping_path, dataroot]:
        if not p.exists():
            raise FileNotFoundError(p)

    src_meta = dataroot / args.version
    if not src_meta.exists():
        raise FileNotFoundError(f"nuScenes metadata klasörü bulunamadı: {src_meta}")

    temp_dir = Path(__file__).resolve().parent / "_cvat_export_tmp"

    print("1) CVAT export okunuyor...")
    datumaro_path = extract_datumaro_json(cvat_export, temp_dir)
    datumaro = load_json(datumaro_path)

    print("2) Mapping okunuyor...")
    mapping = load_json(mapping_path)

    frame_by_item = {
        str(frame["item_id"]): frame
        for frame in mapping["frames"]
    }

    track_to_instance = {
        int(track_id): instance_token
        for track_id, instance_token
        in mapping.get("track_id_to_instance_token", {}).items()
    }

    print("3) Orijinal nuScenes tabloları okunuyor...")
    sample_annotations = load_json(src_meta / "sample_annotation.json")
    instances = load_json(src_meta / "instance.json")
    categories = load_json(src_meta / "category.json")
    samples = load_json(src_meta / "sample.json")

    category_name_to_token = {
        row["name"]: row["token"]
        for row in categories
    }

    original_instance_by_token = {
        inst["token"]: inst
        for inst in instances
    }

    original_ann_by_sample_instance = {
        (ann["sample_token"], ann["instance_token"]): ann
        for ann in sample_annotations
    }

    sample_timestamp = {
        s["token"]: int(s["timestamp"])
        for s in samples
    }

    label_rows = datumaro["categories"]["label"]["labels"]
    label_id_to_name = {
        i: row["name"]
        for i, row in enumerate(label_rows)
    }

    mapped_sample_tokens = {
        frame["sample_token"]
        for frame in mapping["frames"]
    }

    # Bu sample'lar CVAT exportundan yeniden kurulacak.
    untouched_annotations = [
        dict(ann)
        for ann in sample_annotations
        if ann["sample_token"] not in mapped_sample_tokens
    ]

    rebuilt_annotations = []
    new_track_to_instance = dict(track_to_instance)

    # Yeni instance'lar için category token'ı burada sakla.
    new_instance_category = {}

    print("4) CVAT cuboid'leri nuScenes global coordinate'e çevriliyor...")

    for item in datumaro.get("items", []):
        item_id = str(item["id"])

        if item_id not in frame_by_item:
            print(f"UYARI: mapping bulunamadı, frame atlandı: {item_id}")
            continue

        frame_meta = frame_by_item[item_id]
        sample_token = frame_meta["sample_token"]
        lidar_meta = frame_meta["lidar"]

        for ann in item.get("annotations", []):
            if ann.get("type") != "cuboid_3d":
                continue

            attrs = ann.get("attributes", {})
            track_id = attrs.get("track_id")

            if track_id is None:
                track_id = 1_000_000 + int(ann.get("id", 0))

            track_id = int(track_id)

            label_name = label_id_to_name[int(ann["label_id"])]
            if label_name not in category_name_to_token:
                raise ValueError(f"nuScenes category bulunamadı: {label_name}")

            category_token = category_name_to_token[label_name]

            if track_id not in new_track_to_instance:
                new_track_to_instance[track_id] = uuid.uuid4().hex

            instance_token = new_track_to_instance[track_id]
            new_instance_category[instance_token] = category_token

            translation, quaternion = lidar_box_to_global(
                ann["position"],
                ann["rotation"],
                lidar_meta,
            )

            # CVAT/Datumaro scale = [length, width, height]
            length, width, height = [float(v) for v in ann["scale"]]

            # nuScenes size = [width, length, height]
            nuscenes_size = [width, length, height]

            original = original_ann_by_sample_instance.get(
                (sample_token, instance_token)
            )

            if original:
                ann_token = original["token"]
                attribute_tokens = original.get("attribute_tokens", [])
                visibility_token = original.get("visibility_token", "")
                num_lidar_pts = int(original.get("num_lidar_pts", 0))
                num_radar_pts = int(original.get("num_radar_pts", 0))
            else:
                ann_token = uuid.uuid4().hex
                attribute_tokens = []
                visibility_token = str(attrs.get("visibility_token", ""))
                num_lidar_pts = int(attrs.get("num_lidar_pts", 0))
                num_radar_pts = int(attrs.get("num_radar_pts", 0))

            rebuilt_annotations.append({
                "token": ann_token,
                "sample_token": sample_token,
                "instance_token": instance_token,
                "visibility_token": visibility_token,
                "attribute_tokens": attribute_tokens,
                "translation": translation,
                "size": nuscenes_size,
                "rotation": quaternion,
                "prev": "",
                "next": "",
                "num_lidar_pts": num_lidar_pts,
                "num_radar_pts": num_radar_pts,
            })

    # Bütün dataset annotation'ları.
    output_annotations = untouched_annotations + rebuilt_annotations

    print("5) TÜM instance zincirleri yeniden kuruluyor...")

    # Kritik düzeltme:
    # instance.json yalnız CVAT'ta değişen scene için değil,
    # output_annotations'ta kalan bütün instance'lar için yeniden oluşturulur.
    anns_by_instance = {}
    for ann in output_annotations:
        anns_by_instance.setdefault(ann["instance_token"], []).append(ann)

    output_instances = []

    for instance_token, anns in anns_by_instance.items():
        anns.sort(
            key=lambda a: sample_timestamp.get(a["sample_token"], 0)
        )

        # prev / next'i bütün dataset boyunca yeniden oluştur.
        for i, ann in enumerate(anns):
            ann["prev"] = anns[i - 1]["token"] if i > 0 else ""
            ann["next"] = anns[i + 1]["token"] if i + 1 < len(anns) else ""

        original_inst = original_instance_by_token.get(instance_token)

        if original_inst:
            category_token = original_inst["category_token"]
        elif instance_token in new_instance_category:
            category_token = new_instance_category[instance_token]
        else:
            raise ValueError(
                f"Instance category bulunamadı: {instance_token}"
            )

        output_instances.append({
            "token": instance_token,
            "category_token": category_token,
            "nbr_annotations": len(anns),
            "first_annotation_token": anns[0]["token"],
            "last_annotation_token": anns[-1]["token"],
        })

    print("6) Dataset bütünlük kontrolü...")

    output_instance_tokens = {
        inst["token"]
        for inst in output_instances
    }

    missing_instances = sorted({
        ann["instance_token"]
        for ann in output_annotations
        if ann["instance_token"] not in output_instance_tokens
    })

    if missing_instances:
        raise RuntimeError(
            "sample_annotation -> instance referansları eksik: "
            + ", ".join(missing_instances[:10])
        )

    # Token uniqueness kontrolü.
    ann_tokens = [a["token"] for a in output_annotations]
    if len(ann_tokens) != len(set(ann_tokens)):
        raise RuntimeError("Duplicate sample_annotation token bulundu.")

    instance_tokens = [i["token"] for i in output_instances]
    if len(instance_tokens) != len(set(instance_tokens)):
        raise RuntimeError("Duplicate instance token bulundu.")

    print("7) Yeni nuScenes dataset yazılıyor...")

    if out_root.exists():
        shutil.rmtree(out_root)

    shutil.copytree(dataroot, out_root)

    out_meta = out_root / args.version

    save_json(
        out_meta / "sample_annotation.json",
        output_annotations,
    )

    save_json(
        out_meta / "instance.json",
        output_instances,
    )

    report = {
        "source_cvat_export": str(cvat_export),
        "source_mapping": str(mapping_path),
        "mapped_frames": len(mapping["frames"]),
        "rebuilt_annotations": len(rebuilt_annotations),
        "total_annotations": len(output_annotations),
        "total_instances": len(output_instances),
        "output_dataset": str(out_root),
        "integrity_check": "passed",
    }

    save_json(
        out_root / "roundtrip_report.json",
        report,
    )

    if temp_dir.exists():
        shutil.rmtree(temp_dir)

    print()
    print("============================================")
    print("ROUND-TRIP TAMAMLANDI - INTEGRITY CHECK OK")
    print("============================================")
    print(f"Output          : {out_root}")
    print(f"CVAT annotations: {len(rebuilt_annotations)}")
    print(f"Total ann       : {len(output_annotations)}")
    print(f"Total instance  : {len(output_instances)}")
    print(f"Report          : {out_root / 'roundtrip_report.json'}")


if __name__ == "__main__":
    main()

# pick_and_place_text/yolo.py
import time
import numpy as np
import cv2
import os
import json
import rclpy
from ultralytics import YOLO

class YoloModel:
    def __init__(self):
        resource_path = "/home/rokey/quoridor_ws/src/quoridor_main/resource"
        model_path = os.path.join(resource_path, "quoridor_final.pt")
        json_path = os.path.join(resource_path, "class_name_tool.json")

        self.model = YOLO(model_path)

        with open(json_path, "r") as f:
            class_dict = json.load(f)
            self.id2name = {int(k): v for k, v in class_dict.items()}

    def get_fused_segment_centers(
        self,
        img_node,
        duration=1.0,
        conf_th=0.5,
        dist_th=30
    ):
        frames = self._get_frames(img_node, duration)
        results = self.model(frames, verbose=False)

        raw = []

        for res in results:
            if res.masks is None:
                continue

            for i, box in enumerate(res.boxes):
                score = float(box.conf[0])
                if score < conf_th:
                    continue

                cls = self.id2name[int(box.cls[0])]
                mask = res.masks.xy[i].astype(np.int32)

                # 🔹 centroid
                M = cv2.moments(mask)
                if M["m00"] == 0:
                    continue

                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]

                # 🔹 mask bounding box → 방향 판별용
                x, y, w, h = cv2.boundingRect(mask)

                raw.append({
                    "class": cls,
                    "center": np.array([cx, cy]),
                    "score": score,
                    "w": w,
                    "h": h,
                })

        # 🔹 temporal fusion
        fused = []
        used = [False] * len(raw)

        for i, det in enumerate(raw):
            if used[i]:
                continue

            group = [det]
            used[i] = True

            for j, other in enumerate(raw):
                if used[j]:
                    continue
                if other["class"] != det["class"]:
                    continue

                if np.linalg.norm(det["center"] - other["center"]) < dist_th:
                    group.append(other)
                    used[j] = True

            centers = np.array([g["center"] for g in group])
            scores = np.array([g["score"] for g in group])
            widths  = np.array([g["w"] for g in group])
            heights = np.array([g["h"] for g in group])

            # 🔥 wall 방향 판별
            orientation = None
            if det["class"] == "wall":
                if widths.mean() > heights.mean():
                    orientation = "horizontal"
                else:
                    orientation = "vertical"

            fused.append({
                "class": det["class"],
                "center": centers.mean(axis=0),
                "score": scores.mean(),
                "count": len(group),
                "orientation": orientation,
            })

        return fused

    def _get_frames(self, img_node, duration):
        end = time.time() + duration
        frames = []
        while time.time() < end:
            rclpy.spin_once(img_node)
            frame = img_node.get_color_frame()
            if frame is not None:
                frames.append(frame)
            time.sleep(0.01)
        return frames

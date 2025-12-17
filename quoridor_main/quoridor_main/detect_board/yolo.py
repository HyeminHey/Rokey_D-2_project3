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
        resource_path = "/home/hyemin/quoridor_ws/src/quoridor_main/resource"
        model_path = os.path.join(resource_path, "quoridor_final.pt")
        json_path = os.path.join(resource_path, "class_name_tool.json")

        if not os.path.exists(model_path):
            raise FileNotFoundError(f"YOLO model not found: {model_path}")

        if not os.path.exists(json_path):
            raise FileNotFoundError(f"Class json not found: {json_path}")

        self.model = YOLO(model_path)

        with open(json_path, "r") as f:
            class_dict = json.load(f)
            self.id2name = {int(k): v for k, v in class_dict.items()}

    def get_fused_segment_centers(
        self,
        img_node,
        duration=1.0,
        conf_th=0.5,
        dist_th=15,
        angle_th=10.0,
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
                # if cls != "wall":
                #     continue
                mask = res.masks.xy[i].astype(np.int32)


                # === minAreaRect ===
                (cx, cy), (w, h), _ = cv2.minAreaRect(mask)
                center = np.array([cx, cy])

                # === PCA 기반 방향 ===
                angle = self._pca_angle(mask)


                raw.append({
                    "class": cls,
                    "center": center,
                    "score": score,
                    "w": w,
                    "h": h,
                    "angle": angle,
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
            angles  = np.array([g["angle"] for g in group])

            mean_angle  = self._circular_mean_deg(angles)
            folded_angle = self._fold_angle_180(mean_angle)

            if folded_angle < angle_th:
                orientation = "horizontal"
                grasp_angle = 0.0
            elif abs(folded_angle - 90.0) < angle_th:
                orientation = "vertical"
                grasp_angle = 90.0
            else:
                orientation = "misaligned"
                grasp_angle = mean_angle


            fused.append({
                "class": det["class"],
                "center": centers.mean(axis=0),
                "score": scores.mean(),
                "count": len(group),
                "orientation": orientation,
                "angle": grasp_angle
            })

        return fused

    def _pca_angle(self, mask):
        pts = mask.reshape(-1, 2)
        pts = pts - pts.mean(axis=0)

        _, _, vt = np.linalg.svd(pts)
        direction = vt[0]
        return np.degrees(np.arctan2(direction[1], direction[0]))
    
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

    def _normalize_angle(self, rect):
        """
        OpenCV minAreaRect angle normalize
        return: -90 ~ +90 (wall direction)
        """
        (_, _), (w, h), angle = rect
        if w < h:
            angle += 90.0
        return angle

    def _circular_mean_deg(self, angles):
        """
        평균 각도 (degree)
        """
        rad = np.deg2rad(angles)
        mean_rad = np.arctan2(np.mean(np.sin(rad)), np.mean(np.cos(rad)))
        return np.rad2deg(mean_rad)
    
    def _fold_angle_180(self, angle):
        """
        angle (deg) → [0, 90] 범위로 folding
        0 ≡ 180, 90 ≡ -90
        """
        a = abs(angle) % 180.0
        if a > 90.0:
            a = 180.0 - a
        return a

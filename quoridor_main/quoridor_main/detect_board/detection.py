import numpy as np
import rclpy
from rclpy.node import Node

from scipy.spatial.transform import Rotation
from quoridor_main.detect_board.realsense import ImgNode
from quoridor_main.detect_board.yolo import YoloModel
from qulido_robot_msgs.srv import GetBoardState
from qulido_robot_msgs.msg import Int32Row
import DR_init
import sys, os

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("board_detect", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import get_current_posx
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

BOARD_X_MIN = 280
BOARD_X_MAX = 674
BOARD_Y_MIN = -185
BOARD_Y_MAX = 210 # for detect

# ===============================
# Depth Median (🔥 중요)
# ===============================
def get_depth_median(depth, cx, cy, k=5):
    h, w = depth.shape
    xs = slice(max(cx - k, 0), min(cx + k, w))
    ys = slice(max(cy - k, 0), min(cy + k, h))
    region = depth[ys, xs]
    valid = region[region > 0]
    return np.median(valid) if len(valid) else 0

def map_pawn_to_board(x, y):
    # 1️⃣ 보드 범위 체크 (가장 중요)
    if not (BOARD_X_MIN <= x <= BOARD_X_MAX and
            BOARD_Y_MIN <= y <= BOARD_Y_MAX):
        return None

    cell_w = (BOARD_X_MAX - BOARD_X_MIN) / 7
    cell_h = (BOARD_Y_MAX - BOARD_Y_MIN) / 7

    row = int((x - BOARD_X_MIN) / cell_w)
    col = int((y - BOARD_Y_MIN) / cell_h)

    # 2️⃣ 인덱스 범위 체크
    if row < 0 or row > 6 or col < 0 or col > 6:
        return None

    return row, col



def map_wall_to_board(x, y):
    pawn_cell_w = (BOARD_X_MAX - BOARD_X_MIN) / 7
    pawn_cell_h = (BOARD_Y_MAX - BOARD_Y_MIN) / 7

    wall_x_min = BOARD_X_MIN + pawn_cell_w / 2
    wall_x_max = BOARD_X_MAX - pawn_cell_w / 2
    wall_y_min = BOARD_Y_MIN + pawn_cell_h / 2
    wall_y_max = BOARD_Y_MAX - pawn_cell_h / 2

    # 1️⃣ wall 영역 체크
    if not (wall_x_min <= x <= wall_x_max and
            wall_y_min <= y <= wall_y_max):
        return None

    wall_cell_w = (wall_x_max - wall_x_min) / 6
    wall_cell_h = (wall_y_max - wall_y_min) / 6

    row = int((x - wall_x_min) / wall_cell_w)
    col = int((y - wall_y_min) / wall_cell_h)

    # 2️⃣ 인덱스 체크
    if row < 0 or row > 5 or col < 0 or col > 5:
        return None

    return row, col


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.img_node = ImgNode()
        self.model = YoloModel()

        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )

        self.now_state = None


        # 🔥 결과 저장용
        self.vision_srv = self.create_service(
            GetBoardState,
            "/vision/get_board_state",
            self.handle_get_board_state
        )

        self.get_logger().info("✅ Vision service [/vision/get_board_state] ready")

    def handle_get_board_state(self, request, response):
        self.get_logger().info("📸 Vision request received")

        try:
            self.get_robot_pos_safe()
        except RuntimeError as e:
            self.get_logger().error(f"❌ Robot pose not ready: {e}")
            response.board_state = []
            return response
        
        self.now_state = request.now_state
        self.get_logger().info(f"now_state = {self.now_state}")

        # 이전 결과 초기화
        self.red_pawns = []
        self.blue_pawns = []
        self.horizontal_walls = []
        self.vertical_walls = []
        self.misaligned_walls = []

        # 인식 수행
        self.process_scene()


        # 실제코드
        if self.now_state == "HUMAN_TURN":
            board_array = self.build_board_state_array(
                self.red_pawns,
                self.blue_pawns,
                self.horizontal_walls,
                self.vertical_walls
            )
            # 🔥 Int32Row[] 로 변환
            response.board_state = []
            for item in board_array:
                row = Int32Row()
                row.data = item   # [type, r, c]
                response.board_state.append(row)

            self.get_logger().info(
                f"📤 Vision response: {[r.data for r in response.board_state]}"
            )

        elif self.now_state == "CLEAN_UP":
            # cleanup용 board array 생성
            clean_board_array = self.build_clean_board_state_array(
                self.red_pawns,
                self.blue_pawns,
                self.horizontal_walls,
                self.vertical_walls,
                self.misaligned_walls,
            )
            # 🔥 Int32Row[] 로 변환
            response.board_state = []
            for item in clean_board_array:
                row = Int32Row()
                row.data = item   # [type, r, c]
                response.board_state.append(row)

            self.get_logger().info(
                f"📤 Vision response: {[r.data for r in response.board_state]}"
            )
        return response


    def process_scene(self):
        detections = self.model.get_fused_segment_centers(self.img_node)

        depth = self._wait_for_valid_data(
            self.img_node.get_depth_frame, "depth frame"
        )

        for det in detections:
            # 🔒 temporal 신뢰도 필터
            if det["count"] < 3:
                continue

            cx, cy = map(int, det["center"])
            z = get_depth_median(depth, cx, cy)
            if z <= 0:
                continue

            cam_xyz = self._pixel_to_camera_coords(cx, cy, z)
            base_xyz = self._camera_to_base(cam_xyz)

            cls = det["class"]
            
            PAWN_Y_OFFSET = -15.0  # mm (실험으로 조정)

            if cls == "red_pawn":
                x, y, z = base_xyz
                self.red_pawns.append((x, y + PAWN_Y_OFFSET, z))

            elif cls == "blue_pawn":
                x, y, z = base_xyz
                self.blue_pawns.append((x, y + PAWN_Y_OFFSET, z))

            elif cls == "wall":
                yaw = self.image_angle_to_base_yaw(det["angle"])

                if det["orientation"] == "horizontal":
                    self.horizontal_walls.append(base_xyz)
                elif det["orientation"] == "vertical":
                    self.vertical_walls.append(base_xyz)
                elif det["orientation"] == "misaligned":
                    x, y, z = base_xyz
                    self.misaligned_walls.append((x, y, z, yaw))

        self.get_logger().info(f"Red pawns: {self.red_pawns}")
        self.get_logger().info(f"Blue pawns: {self.blue_pawns}")
        self.get_logger().info(f"H Walls: {self.horizontal_walls}")
        self.get_logger().info(f"V Walls: {self.vertical_walls}")
        self.get_logger().info(f"M Walls: {self.misaligned_walls}")


    def get_robot_pos_safe(self, retry=10):
        for i in range(retry):
            try:
                pos = get_current_posx()
            except IndexError:
                self.get_logger().warn(
                    f"⚠ get_current_posx() failed internally (retry {i+1}/{retry})"
                )
                rclpy.spin_once(dsr_node, timeout_sec=0.1)
                continue

            if pos and len(pos) > 0 and len(pos[0]) == 6:
                return pos[0]

            self.get_logger().warn(
                f"⚠ get_current_posx returned invalid data (retry {i+1}/{retry})"
            )
            rclpy.spin_once(dsr_node, timeout_sec=0.1)

        raise RuntimeError("❌ Failed to get robot pose after retries")


    def _camera_to_base(self, camera_coords):
        # detection.py 기준 경로
        # current_dir = os.path.dirname(os.path.abspath(__file__))

        # # quoridor_main/detect_board → quoridor_main → resource
        # resource_path = os.path.abspath(
        #     os.path.join(current_dir, "..", "..", "resource")
        # )
        resource_path = "/home/hyemin/quoridor_ws/src/quoridor_main/resource"
        gripper2cam = np.load(
            os.path.join(resource_path, "T_gripper2camera.npy")
        )

        robot_pos = self.get_robot_pos_safe()
        x, y, z, rx, ry, rz = robot_pos

        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        cam = np.append(np.array(camera_coords), 1)
        base = base2gripper @ gripper2cam @ cam

        return tuple(base[:3])
    
    def _wait_for_valid_data(self, getter, description):
        """getter 함수가 유효한 데이터를 반환할 때까지 spin 하며 재시도합니다."""
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data
    
    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def _pixel_to_camera_coords(self, x, y, z):
        fx, fy = self.intrinsics["fx"], self.intrinsics["fy"]
        ppx, ppy = self.intrinsics["ppx"], self.intrinsics["ppy"]

        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )
    
    def image_angle_to_base_yaw(self, angle_deg):
        theta = np.deg2rad(angle_deg)
        R_cam = Rotation.from_euler("Z", theta).as_matrix()

        x, y, z, rx, ry, rz = self.get_robot_pos_safe()
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        resource_path = "/home/hyemin/quoridor_ws/src/quoridor_main/resource"
        gripper2cam = np.load(os.path.join(resource_path, "T_gripper2camera.npy"))

        R_base = base2gripper[:3, :3] @ gripper2cam[:3, :3] @ R_cam
        return Rotation.from_matrix(R_base).as_euler("ZYX", degrees=True)[0]

    
    def build_board_state_array(
        self,
        red_pawns,
        blue_pawns,
        horizontal_walls,
        vertical_walls,
    ):
        board_state = []

        # 🔴 Red Pawn
        for x, y, _ in red_pawns:
            rc = map_pawn_to_board(x, y)
            if rc is None:
                continue
            r, c = rc
            board_state.append([1, r, c])

        # 🔵 Blue Pawn
        for x, y, _ in blue_pawns:
            rc = map_pawn_to_board(x, y)
            if rc is None:
                continue
            r, c = rc
            board_state.append([-1, r, c])

        # ➖ Horizontal Wall
        for x, y, _ in horizontal_walls:
            rc = map_wall_to_board(x, y)
            if rc is None:
                continue
            r, c = rc
            board_state.append([-2, r, c])

        # ➕ Vertical Wall
        for x, y, _ in vertical_walls:
            rc = map_wall_to_board(x, y)
            if rc is None:
                continue
            r, c = rc
            board_state.append([2, r, c])

        return board_state


    # msg type이 int32인게 문제가 될까.. mm단위라 괜찮지 않을까..
    def build_clean_board_state_array(
        self,
        red_pawns,
        blue_pawns,
        horizontal_walls,
        vertical_walls,
        misaligned_walls
    ):
        clean_board_state = []

        # 🔴 Red Pawn
        for x, y, _ in red_pawns:
            if not (BOARD_X_MIN <= x <= BOARD_X_MAX and BOARD_Y_MIN <= y <= BOARD_Y_MAX):
                continue
            clean_board_state.append([1, int(x), int(y)])

        # 🔵 Blue Pawn
        for x, y, _ in blue_pawns:
            if not (BOARD_X_MIN <= x <= BOARD_X_MAX and BOARD_Y_MIN <= y <= BOARD_Y_MAX):
                continue
            clean_board_state.append([-1, int(x), int(y)])

        # ➖ Horizontal Wall
        for x, y, _ in horizontal_walls:
            if not (BOARD_X_MIN <= x <= BOARD_X_MAX and BOARD_Y_MIN <= y <= BOARD_Y_MAX):
                continue
            clean_board_state.append([-2, int(x), int(y)])

        # ➕ Vertical Wall
        for x, y, _ in vertical_walls:
            if not (BOARD_X_MIN <= x <= BOARD_X_MAX and BOARD_Y_MIN <= y <= BOARD_Y_MAX):
                continue
            clean_board_state.append([2, int(x), int(y)])
        
        # Misaligned Wall
        for x, y, _, angle in misaligned_walls:
            if not (BOARD_X_MIN <= x <= BOARD_X_MAX and BOARD_Y_MIN <= y <= BOARD_Y_MAX):
                continue
            clean_board_state.append([3, int(x), int(y), int(angle)])

        return clean_board_state



def main(args=None):
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

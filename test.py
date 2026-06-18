import cv2
import numpy as np
import socket
import json
import time
import math
import threading
import subprocess

LOCAL_IP = "0.0.0.0"
LOCAL_PORT = 9005

TARGET_IP = "192.168.31.202"
TARGET_PORT = 9005

CAMERA_ID = 0
CAMERA_W = 1920
CAMERA_H = 1200
CAMERA_FPS = 30

# 只降低显示窗口，不降低摄像头采集分辨率。
# AprilTag 检测仍然使用 1920x1200 原图，保证远处 tag 还能看见。
DISPLAY_W = 640
DISPLAY_H = 400
DISPLAY_INTERVAL = 1.0 / 15.0

CV_THREADS = 4

# OpenCV 的 V4L2 后端在这块板子上读 MJPG 只有十几帧；
# GStreamer jpegdec 实测能到 30fps，所以默认走 GStreamer。
USE_GSTREAMER_CAMERA = True

# 摄像头画质参数都放这里，方便现场自己调。
# auto_exposure: 1=手动曝光，3=自动曝光。自动曝光可能导致帧率或亮度波动。
# 现场建议先用手动曝光：tag 黑白边界清楚，比画面看起来很亮更重要。
# 是否在程序启动时用 v4l2-ctl 写入下面这些摄像头参数。
CAMERA_APPLY_CONTROLS = True
# 曝光模式：1=手动曝光，3=自动曝光。建议比赛/定位时用 1，亮度更稳定。
CAMERA_AUTO_EXPOSURE = 1
# 手动曝光时间。数值越大画面越亮、运动拖影越明显；太大可能影响帧率。
CAMERA_EXPOSURE = 40
# 传感器增益。数值越大画面越亮，但噪声也越多，tag 边缘会变脏。
CAMERA_GAIN = 30
# 亮度偏移。一般保持 0，过高会冲淡黑色 tag，过低会压暗细节。
CAMERA_BRIGHTNESS = 0
# 对比度。适当提高有利于 tag 黑白分明，太高会丢灰阶细节。
CAMERA_CONTRAST = 10
# 饱和度。对 AprilTag 检测影响不大，保持摄像头默认值即可。
CAMERA_SATURATION = 56
# 自动白平衡：0=关闭，1=开启。关闭后颜色不容易忽冷忽热。
CAMERA_WHITE_BALANCE_AUTO = 0
# 手动白平衡色温。画面偏蓝就调高，偏黄就调低。
CAMERA_WHITE_BALANCE_TEMPERATURE = 4600

# 运行时按 w/s 调曝光时允许的最小值，防止一键调得太暗。
EXPOSURE_MIN = 40
# 运行时按 w/s 调曝光时允许的最大值，防止过曝或拖影太重。
EXPOSURE_MAX = 320
# 每按一次 w 或 s，曝光变化的步长。
EXPOSURE_STEP = 10
# 运行时按 a/d 调增益时允许的最小值。
GAIN_MIN = 0
# 运行时按 a/d 调增益时允许的最大值，过高会明显增加噪声。
GAIN_MAX = 300
# 每按一次 a 或 d，增益变化的步长。
GAIN_STEP = 20

# 先用更小 ROI 测试速度。如果容易丢车，再改成 160 或 200。
# ROI 越大越稳但越慢；ROI 越小越快但车辆移动大时更容易丢。
ROI_MARGIN = 120
LOST_FULL_SCAN_AFTER = 5

SEND_INTERVAL = 0.04
PRINT_INTERVAL = 1.0

# 四个基准 tag 的真实物理坐标，单位 cm。
# 标定完成后用它们求透视变换矩阵，把图像坐标转成场地坐标。
PHYSICAL_POINTS = {
    10: [0, 0],
    13: [399, 0],
    12: [399, 449],
    11: [0, 449],
}
FIELD_W, FIELD_H = 399, 449


def set_camera_control(name, value):
    """调用 v4l2-ctl 修改摄像头参数。

    用 subprocess 而不是 OpenCV cap.set，是因为 GStreamer 打开摄像头后，
    OpenCV 的属性设置不一定能稳定传到 UVC 驱动。
    """
    try:
        subprocess.run(
            [
                "v4l2-ctl",
                "-d",
                f"/dev/video{CAMERA_ID}",
                f"--set-ctrl={name}={int(value)}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return True
    except Exception:
        return False


def apply_camera_controls(state):
    """把当前曝光/增益/白平衡参数一次性写入摄像头。"""
    if not CAMERA_APPLY_CONTROLS:
        return

    set_camera_control("auto_exposure", state["auto_exposure"])
    set_camera_control("exposure_time_absolute", state["exposure"])
    set_camera_control("gain", state["gain"])
    set_camera_control("brightness", state["brightness"])
    set_camera_control("contrast", state["contrast"])
    set_camera_control("saturation", state["saturation"])
    set_camera_control("white_balance_automatic", state["white_balance_auto"])
    if state["white_balance_auto"] == 0:
        set_camera_control("white_balance_temperature", state["white_balance_temperature"])


def clamp_value(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def handle_camera_tuning_key(key, state):
    """处理运行时调参按键。

    w/s 调曝光，d/a 调增益，e 切手动/自动曝光，r 恢复默认值。
    """
    changed = False

    if key == ord("w"):
        state["exposure"] = clamp_value(state["exposure"] + EXPOSURE_STEP, EXPOSURE_MIN, EXPOSURE_MAX)
        changed = True
    elif key == ord("s"):
        state["exposure"] = clamp_value(state["exposure"] - EXPOSURE_STEP, EXPOSURE_MIN, EXPOSURE_MAX)
        changed = True
    elif key == ord("d"):
        state["gain"] = clamp_value(state["gain"] + GAIN_STEP, GAIN_MIN, GAIN_MAX)
        changed = True
    elif key == ord("a"):
        state["gain"] = clamp_value(state["gain"] - GAIN_STEP, GAIN_MIN, GAIN_MAX)
        changed = True
    elif key == ord("e"):
        state["auto_exposure"] = 3 if state["auto_exposure"] == 1 else 1
        changed = True
    elif key == ord("r"):
        state["exposure"] = CAMERA_EXPOSURE
        state["gain"] = CAMERA_GAIN
        state["auto_exposure"] = CAMERA_AUTO_EXPOSURE
        changed = True

    if changed:
        apply_camera_controls(state)
        mode = "AUTO" if state["auto_exposure"] == 3 else "MANUAL"
        print(
            f"camera tune: exposure={state['exposure']} "
            f"gain={state['gain']} exposure_mode={mode}",
            flush=True
        )

    return changed


def print_camera_tuning_help():
    """打印运行时按键说明。窗口里也会显示英文简写版。"""
    print("Camera tuning keys:", flush=True)
    print("  w: 曝光增加", flush=True)
    print("  s: 曝光减少", flush=True)
    print("  d: 增益增加", flush=True)
    print("  a: 增益减少", flush=True)
    print("  e: 手动/自动曝光切换", flush=True)
    print("  r: 恢复代码里的默认曝光和增益", flush=True)
    print("  q: 退出", flush=True)


class LatestCamera:
    """只保留摄像头最新帧的采集线程。

    检测线程永远读取最新帧，旧帧直接丢掉，避免算法偶尔变慢时延迟越堆越高。
    """
    def __init__(self, camera_id, width, height, fps):
        self.cap = self._open_camera(camera_id, width, height, fps)

        self.lock = threading.Lock()
        self.frame = None
        self.frame_time = 0.0
        self.frame_id = 0
        self.running = True

        self.capture_fps = 0.0
        self.capture_count = 0
        self.capture_t0 = time.perf_counter()

        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _open_camera(self, camera_id, width, height, fps):
        if USE_GSTREAMER_CAMERA:
            # appsink drop=true max-buffers=1: 下游处理慢时丢旧帧，只保留最新帧。
            # sync=false: 不按播放时钟等待，尽快把帧交给程序。
            pipeline = (
                f"v4l2src device=/dev/video{camera_id} io-mode=2 ! "
                f"image/jpeg,width={width},height={height},framerate={fps}/1 ! "
                "jpegdec ! "
                "videoconvert ! video/x-raw,format=BGR ! "
                "appsink drop=true max-buffers=1 sync=false"
            )
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                print("Camera backend: GStreamer jpegdec", flush=True)
                return cap
            print("GStreamer camera open failed, fallback to OpenCV V4L2.", flush=True)

        cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        print("Camera backend: OpenCV V4L2", flush=True)
        return cap

    def _reader(self):
        """后台持续读摄像头，并统计真实采集 FPS。"""
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.005)
                continue

            now = time.perf_counter()
            with self.lock:
                self.frame = frame
                self.frame_time = now
                self.frame_id += 1

                self.capture_count += 1
                elapsed = now - self.capture_t0
                if elapsed >= 1.0:
                    self.capture_fps = self.capture_count / elapsed
                    self.capture_count = 0
                    self.capture_t0 = now

    def read_latest(self):
        with self.lock:
            if self.frame is None:
                return False, None, 0.0, 0

            # 少复制一次 1920x1200 大图，降低内存带宽压力。
            return True, self.frame, self.frame_time, self.frame_id

    def release(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()


def clamp_roi(x1, y1, x2, y2, width, height):
    """把 ROI 限制在图像范围内，避免切片越界。"""
    x1 = max(0, min(int(x1), width - 1))
    y1 = max(0, min(int(y1), height - 1))
    x2 = max(1, min(int(x2), width))
    y2 = max(1, min(int(y2), height))

    if x2 <= x1 or y2 <= y1:
        return None

    return x1, y1, x2, y2


def make_roi_from_marker(marker_corner, frame_shape, margin):
    """根据上一次检测到的车 tag 生成下一帧搜索 ROI。"""
    height, width = frame_shape[:2]
    c = marker_corner[0]

    x1 = np.min(c[:, 0]) - margin
    y1 = np.min(c[:, 1]) - margin
    x2 = np.max(c[:, 0]) + margin
    y2 = np.max(c[:, 1]) + margin

    return clamp_roi(x1, y1, x2, y2, width, height)


def detect_markers_roi(frame, aruco_dict, params, roi=None):
    # 有 ROI 时先裁剪，再转灰度，避免每次都处理整张 1920x1200。
    if roi is not None:
        x1, y1, x2, y2 = roi
        work_img = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2GRAY)
        offset = np.array([[[x1, y1]]], dtype=np.float32)
    else:
        work_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        offset = np.array([[[0, 0]]], dtype=np.float32)

    corners, ids, rejected = cv2.aruco.detectMarkers(
        work_img,
        aruco_dict,
        parameters=params
    )

    if corners is not None:
        corners = [c + offset for c in corners]

    return corners, ids, rejected


def get_bev_point(u, v, matrix):
    """把图像像素点通过透视矩阵映射到场地物理坐标。"""
    point = np.array([u, v, 1.0], dtype=np.float32).reshape(3, 1)
    transformed = np.dot(matrix, point)
    w = transformed[2, 0]

    if w == 0:
        return None

    return float(transformed[0, 0] / w), float(transformed[1, 0] / w)


def draw_minimap(bg_img, car_x, car_y):
    """在缩小后的显示图上画场地小地图。

    注意：不要在 1920x1200 原图上画小地图，会浪费不少时间。
    """
    img_h, img_w = bg_img.shape[:2]
    map_w = min(180, max(120, img_w // 4))
    map_h = int(map_w * FIELD_H / FIELD_W)
    margin_x = img_w - map_w - 12
    margin_y = 12

    cv2.rectangle(bg_img, (margin_x, margin_y), (margin_x + map_w, margin_y + map_h), (0, 0, 0), -1)
    cv2.rectangle(bg_img, (margin_x, margin_y), (margin_x + map_w, margin_y + map_h), (0, 255, 0), 1)

    for i in range(1, 4):
        x_line = margin_x + int((i * 100) / FIELD_W * map_w)
        cv2.line(bg_img, (x_line, margin_y), (x_line, margin_y + map_h), (80, 80, 80), 1)

    for i in range(1, 5):
        y_line = margin_y + int((i * 100) / FIELD_H * map_h)
        cv2.line(bg_img, (margin_x, y_line), (margin_x + map_w, y_line), (80, 80, 80), 1)

    if car_x is not None and car_y is not None:
        px = max(0, min(car_x, FIELD_W))
        py = max(0, min(car_y, FIELD_H))

        map_px = margin_x + int((px / FIELD_W) * map_w)
        map_py = margin_y + int((py / FIELD_H) * map_h)

        cv2.circle(bg_img, (map_px, map_py), 5, (0, 0, 255), -1)
        cv2.putText(bg_img, "CAR", (map_px + 6, map_py + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)


def draw_tag_markers(display_img, tag_corners, tag_ids, scale_x, scale_y, color, prefix):
    """在显示图上标出已经识别到的 tag。

    tag_corners 使用原始 1920x1200 坐标，绘制时按显示比例缩小。
    """
    if tag_ids is None or tag_corners is None:
        return

    for corner, tag_id in zip(tag_corners, tag_ids):
        pts = corner[0].copy()
        pts[:, 0] *= scale_x
        pts[:, 1] *= scale_y
        pts = pts.astype(np.int32)

        cx = int(np.mean(pts[:, 0]))
        cy = int(np.mean(pts[:, 1]))
        label = f"{prefix}{int(tag_id[0])}"

        cv2.polylines(display_img, [pts], True, color, 2)
        cv2.circle(display_img, (cx, cy), 4, color, -1)
        cv2.putText(display_img, label, (cx + 6, cy - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)


def draw_base_tag_status(display_img, base_cache):
    """显示 4 个参考 tag 哪些已经看到、哪些还缺。"""
    expected_ids = sorted(PHYSICAL_POINTS.keys())
    found_ids = sorted(base_cache.keys())
    missing_ids = [tag_id for tag_id in expected_ids if tag_id not in base_cache]

    text = f"Base found:{found_ids}"
    if missing_ids:
        text += f" missing:{missing_ids}"

    cv2.rectangle(display_img, (10, DISPLAY_H - 36), (460, DISPLAY_H - 8), (0, 0, 0), -1)
    cv2.putText(display_img, text, (18, DISPLAY_H - 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1)


def make_detector_params(min_perimeter, quad_decimate=None, use_aruco3=False):
    """创建 AprilTag/ArUco 检测参数。

    FULL 参数偏稳，ROI 参数偏快；不同 OpenCV 版本支持的字段不完全一样，
    所以用 hasattr 做兼容。
    """
    params = cv2.aruco.DetectorParameters_create()
    params.minMarkerPerimeterRate = min_perimeter
    params.polygonalApproxAccuracyRate = 0.06

    if hasattr(params, "aprilTagQuadDecimate") and quad_decimate is not None:
        params.aprilTagQuadDecimate = quad_decimate

    if hasattr(params, "useAruco3Detection"):
        params.useAruco3Detection = use_aruco3

    return params


cv2.setUseOptimized(True)
cv2.setNumThreads(CV_THREADS)

# 启动时先把摄像头调到一组可用的手动参数。
# 后面也可以用键盘实时调，不需要重启程序。
camera_control_state = {
    "auto_exposure": CAMERA_AUTO_EXPOSURE,
    "exposure": CAMERA_EXPOSURE,
    "gain": CAMERA_GAIN,
    "brightness": CAMERA_BRIGHTNESS,
    "contrast": CAMERA_CONTRAST,
    "saturation": CAMERA_SATURATION,
    "white_balance_auto": CAMERA_WHITE_BALANCE_AUTO,
    "white_balance_temperature": CAMERA_WHITE_BALANCE_TEMPERATURE,
}
apply_camera_controls(camera_control_state)

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind((LOCAL_IP, LOCAL_PORT))
print(f"UDP ready -> target {TARGET_IP}:{TARGET_PORT}")

# FULL 要照顾远处 tag，ROI 要更快。
# 标定和重新找车时用 FULL；锁定车辆后用 ROI。
params_full = make_detector_params(min_perimeter=0.01, quad_decimate=None, use_aruco3=False)
params_roi = make_detector_params(min_perimeter=0.03, quad_decimate=1.5, use_aruco3=True)
params_base = make_detector_params(min_perimeter=0.01, quad_decimate=None, use_aruco3=False)

dict_car = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_16h5)
dict_base = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)

camera = LatestCamera(CAMERA_ID, CAMERA_W, CAMERA_H, CAMERA_FPS)

print("--- Low latency Auto BEV Tracker start ---")
print(f"Camera: {CAMERA_W}x{CAMERA_H} @ {CAMERA_FPS}")
print(f"Display: {DISPLAY_W}x{DISPLAY_H} @ {1.0 / DISPLAY_INTERVAL:.0f}fps")
print(f"OpenCV threads: {CV_THREADS}")
print(f"ROI margin: {ROI_MARGIN}")
print(
    f"Camera controls: exposure={camera_control_state['exposure']} "
    f"gain={camera_control_state['gain']} auto_exposure={camera_control_state['auto_exposure']}"
)
print_camera_tuning_help()
if hasattr(params_roi, "aprilTagQuadDecimate"):
    print(f"ROI aprilTagQuadDecimate: {params_roi.aprilTagQuadDecimate}")
if hasattr(params_roi, "useAruco3Detection"):
    print(f"ROI useAruco3Detection: {params_roi.useAruco3Detection}")

M_matrix = None
is_calibrated = False
base_cache = {}

car_roi = None
lost_count = 0

fps = 0.0
fps_count = 0
fps_t0 = time.perf_counter()

last_send_time = 0.0
last_print_time = 0.0
last_display_time = 0.0
last_processed_frame_id = -1

last_car_x_cm = None
last_car_y_cm = None
last_scan_mode = "FULL"
last_base_corners = None
last_base_ids = None
last_car_corners = None
last_car_ids = None

try:
    while True:
        # 摄像头线程已经在后台持续读帧，这里只拿“最新的一帧”。
        ok, frame, frame_time, frame_id = camera.read_latest()

        if not ok:
            time.sleep(0.005)
            continue

        if frame_id == last_processed_frame_id:
            time.sleep(0.001)
            continue

        last_processed_frame_id = frame_id
        now = time.perf_counter()
        delay_ms = (now - frame_time) * 1000.0

        if not is_calibrated:
            # 未标定时必须全图找 4 个基准 AprilTag。
            corners_base, ids_base, _ = detect_markers_roi(
                frame,
                dict_base,
                params_base,
                roi=None
            )
            last_base_corners = corners_base
            last_base_ids = ids_base

            if ids_base is not None:
                for i in range(len(ids_base)):
                    tid = ids_base[i][0]
                    if tid in PHYSICAL_POINTS:
                        c = corners_base[i][0]
                        cx, cy = np.mean(c[:, 0]), np.mean(c[:, 1])
                        base_cache[tid] = [cx, cy]

            if len(base_cache) == 4:
                # 按固定顺序组织 4 个点，求图像坐标到场地坐标的透视变换。
                src_pts = np.array([
                    base_cache[11],
                    base_cache[12],
                    base_cache[13],
                    base_cache[10],
                ], dtype=np.float32)

                dst_pts = np.array([
                    PHYSICAL_POINTS[11],
                    PHYSICAL_POINTS[12],
                    PHYSICAL_POINTS[13],
                    PHYSICAL_POINTS[10],
                ], dtype=np.float32)

                M_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
                is_calibrated = True
                print("Calibration done.")

        else:
            car_x_cm, car_y_cm = None, None

            # 正常情况下只在上一次车辆附近找；连续丢失后才回到全图搜索。
            need_full_scan = (
                car_roi is None
                or lost_count >= LOST_FULL_SCAN_AFTER
            )

            if need_full_scan:
                search_roi = None
                scan_mode = "FULL"
                detect_params = params_full
            else:
                search_roi = car_roi
                scan_mode = "ROI"
                detect_params = params_roi

            corners_car, ids_car, _ = detect_markers_roi(
                frame,
                dict_car,
                detect_params,
                roi=search_roi
            )
            last_car_corners = corners_car
            last_car_ids = ids_car

            found_car = False

            if ids_car is not None:
                for i in range(len(ids_car)):
                    if ids_car[i][0] == 1:
                        found_car = True

                        c = corners_car[i][0]
                        cx, cy = np.mean(c[:, 0]), np.mean(c[:, 1])

                        car_roi = make_roi_from_marker(corners_car[i], frame.shape, ROI_MARGIN)
                        lost_count = 0

                        phys_coord = get_bev_point(cx, cy, M_matrix)

                        if phys_coord:
                            # 车辆中心点坐标，单位从 cm 转成 m 后发给目标端。
                            car_x_cm, car_y_cm = phys_coord
                            last_car_x_cm = car_x_cm
                            last_car_y_cm = car_y_cm

                            x_m = round(car_x_cm / 100.0, 3)
                            z_m = round(car_y_cm / 100.0, 3)

                            f_px = (c[3] + c[2]) / 2
                            b_px = (c[0] + c[1]) / 2

                            # 用 tag 前后边中点估计车辆朝向。
                            f_ph = get_bev_point(f_px[0], f_px[1], M_matrix)
                            b_ph = get_bev_point(b_px[0], b_px[1], M_matrix)

                            real_yaw = 0.0
                            if f_ph and b_ph:
                                real_yaw = round(
                                    math.degrees(
                                        math.atan2(
                                            f_ph[0] - b_ph[0],
                                            f_ph[1] - b_ph[1]
                                        )
                                    ),
                                    2
                                )

                            if now - last_send_time >= SEND_INTERVAL:
                                # 发送频率由 SEND_INTERVAL 控制，避免 UDP 过于频繁。
                                payload = {
                                    "type": "robot_position",
                                    "pos": [x_m, 0.00, z_m],
                                    "euler": [0.0, real_yaw, 0.0],
                                }

                                try:
                                    json_str = json.dumps(payload)
                                    udp_socket.sendto(
                                        json_str.encode("utf-8"),
                                        (TARGET_IP, TARGET_PORT)
                                    )
                                    last_send_time = now

                                    if now - last_print_time >= PRINT_INTERVAL:
                                        print(
                                            f"send: X:{x_m}m Z:{z_m}m "
                                            f"Yaw:{real_yaw} FPS:{fps:.1f} "
                                            f"CAM:{camera.capture_fps:.1f} "
                                            f"Delay:{delay_ms:.0f}ms Mode:{scan_mode} "
                                            f"ROI:{car_roi}"
                                        )
                                        last_print_time = now

                                except Exception as e:
                                    print(f"send failed: {e}")

                        break

            if not found_car:
                lost_count += 1
                if lost_count >= LOST_FULL_SCAN_AFTER:
                    car_roi = None

            last_scan_mode = scan_mode

        # 这里统计的是算法处理 FPS，不是摄像头采集 FPS。
        fps_count += 1
        elapsed = now - fps_t0

        if elapsed >= 1.0:
            fps = fps_count / elapsed
            fps_count = 0
            fps_t0 = now

        if now - last_display_time >= DISPLAY_INTERVAL:
            # 显示可以降分辨率和降刷新率，检测仍然使用原始高分辨率帧。
            display_frame = cv2.resize(
                frame,
                (DISPLAY_W, DISPLAY_H),
                interpolation=cv2.INTER_AREA
            )

            scale_x = DISPLAY_W / float(CAMERA_W)
            scale_y = DISPLAY_H / float(CAMERA_H)

            cv2.putText(display_frame, "SYSTEM READY" if is_calibrated else f"Calibrating: Found {len(base_cache)}/4",
                        (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if is_calibrated else (0, 165, 255), 2)

            draw_tag_markers(
                display_frame,
                last_base_corners,
                last_base_ids,
                scale_x,
                scale_y,
                (0, 200, 255),
                "B"
            )
            draw_tag_markers(
                display_frame,
                last_car_corners,
                last_car_ids,
                scale_x,
                scale_y,
                (0, 0, 255),
                "C"
            )
            draw_base_tag_status(display_frame, base_cache)

            if is_calibrated and car_roi is not None:
                x1, y1, x2, y2 = car_roi
                cv2.rectangle(
                    display_frame,
                    (int(x1 * scale_x), int(y1 * scale_y)),
                    (int(x2 * scale_x), int(y2 * scale_y)),
                    (255, 180, 0),
                    1
                )

            cv2.rectangle(display_frame, (10, 45), (230, 85), (0, 0, 0), -1)

            cv2.putText(display_frame, f"FPS: {fps:.1f} CAM:{camera.capture_fps:.1f}", (18, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 255), 2)

            if last_car_x_cm is not None and last_car_y_cm is not None:
                cv2.putText(display_frame, f"X:{int(last_car_x_cm)}cm Y:{int(last_car_y_cm)}cm",
                            (300, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                            (0, 255, 255), 2)

            draw_minimap(display_frame, last_car_x_cm, last_car_y_cm)

            cv2.imshow("Auto BEV Tracker & Radar", display_frame)
            last_display_time = now

        key = cv2.waitKey(1) & 0xFF
        # 运行时调曝光/增益，方便现场找 tag 最清楚的画面。
        handle_camera_tuning_key(key, camera_control_state)

        if key == ord("q"):
            break

except KeyboardInterrupt:
    pass

finally:
    camera.release()
    udp_socket.close()
    cv2.destroyAllWindows()

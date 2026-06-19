import cv2
import threading
import time
from config import CAMERA_ID, CAMERA_W, CAMERA_H, CAMERA_FPS, USE_GSTREAMER_CAMERA

class CameraCapture:
    """摄像头采集线程，只保留最新帧"""
    
    def __init__(self):
        self.cap = self._open_camera()
        self.lock = threading.Lock()
        self.frame = None
        self.frame_time = 0.0
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _open_camera(self):
        if USE_GSTREAMER_CAMERA:
            pipeline = (
                f"v4l2src device=/dev/video{CAMERA_ID} io-mode=2 ! "
                f"image/jpeg,width={CAMERA_W},height={CAMERA_H},framerate={CAMERA_FPS}/1 ! "
                "jpegdec ! "
                "videoconvert ! video/x-raw,format=BGR ! "
                "appsink drop=true max-buffers=1 sync=false"
            )
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                print("Camera: GStreamer backend")
                return cap
        
        # 降级到V4L2
        cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_H)
        cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        print("Camera: V4L2 backend (fallback)")
        return cap
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
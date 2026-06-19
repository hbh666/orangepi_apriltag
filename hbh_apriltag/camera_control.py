import subprocess
from config import *

def set_camera_control(name, value):
    """调用v4l2-ctl设置摄像头参数"""
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
    
def apply_camera_controls():
    """应用所有摄像头参数"""
    if not CAMERA_APPLY_CONTROLS:
        return
    
    controls = {
        "auto_exposure": CAMERA_AUTO_EXPOSURE,
        "exposure_time_absolute": CAMERA_EXPOSURE,
        "gain": CAMERA_GAIN,
        "brightness": CAMERA_BRIGHTNESS,
        "contrast": CAMERA_CONTRAST,
        "saturation": CAMERA_SATURATION,
        "white_balance_automatic": CAMERA_WHITE_BALANCE_AUTO,
    }
    
    for name, value in controls.items():
        set_camera_control(name, value)
    
    if CAMERA_WHITE_BALANCE_AUTO == 0:
        set_camera_control("white_balance_temperature", CAMERA_WHITE_BALANCE_TEMPERATURE)
#=======摄像头配置=========
CAMERA_ID = 0
CAMERA_W = 1920
CAMERA_H = 1200
CAMERA_FPS = 30
USE_GSTREAMER_CAMERA = True

#=======网络配置===========
LOCAL_IP = "0.0.0.0"#监听网络上所有设备的9005端口
LOCAL_PORT = 9005
TARGET_IP = "192.168.10.202"
TARGET_PORT = 9005

#=======场地配置(cm)=======
#四个基准tag的真实物理坐标
PYSICAL_POINTS = {
    10: [0,0],    #左下
    13: [399,0],  #右下
    12: [399,499],#右上
    11: [0,499],  #左上
}
FILED_W , FILED_H = 399,499

#=======检测参数============
ROI_MARGIN = 120        #roi搜索边距
LOST_FULL_SCAN_AFTER = 5   #连续丢5帧后全图搜索
SEND_INTERVAL = 0.0 #发送时间间隔(s)


#========滤波参数===========
POSE_FILTER_ENABLE = True  
POSE_FILTER_ALPHA_POS = 0.35    #位置滤波系数
POSE_FILTER_ALPHA_YAW = 0.35    #角度滤波系数
POSE_FILTER_RESET_MISS = 5      #丢失5帧后重置滤波

#========摄像头控制参数======
CAMERA_APPLY_CONTROLS = True
CAMERA_AUTO_EXPOSURE = 1                    # 曝光模式：1=手动曝光，3=自动曝光。1亮度更稳定。
CAMERA_EXPOSURE = 40                        # 手动曝光时间。数值越大画面越亮、运动拖影越明显；太大可能影响帧率。
CAMERA_GAIN = 40                            # 传感器增益。数值越大画面越亮，但噪声也越多，tag 边缘会变脏。
CAMERA_BRIGHTNESS = 0                       # 亮度偏移。一般保持 0，过高会冲淡黑色 tag，过低会压暗细节。
CAMERA_CONTRAST = 10                        # 对比度。适当提高有利于 tag 黑白分明，太高会丢灰阶细节。
CAMERA_SATURATION = 56                      # 饱和度。对 AprilTag 检测影响不大，保持摄像头默认值即可。
CAMERA_WHITE_BALANCE_AUTO = 0               # 自动白平衡：0=关闭，1=开启。关闭后颜色不容易忽冷忽热。
CAMERA_WHITE_BALANCE_TEMPERATURE = 4600

# ========== OpenCV优化 ==========
CV_THREADS = 4
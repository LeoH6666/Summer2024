# Single Color RGB565 Blob Tracking Example
#
# This example shows off single color RGB565 tracking using the OpenMV Cam.

import sensor, image, time, math
import pyb,time
from pyb import UART,LED,Pin

threshold_index = 0 # 0 for red, 1 for green, 2 for blue

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds = [(16, 86, 32, 127, -19, 86), # generic_red_thresholds
              (30, 100, -64, -8, -32, 32), # generic_green_thresholds
              (0, 30, 0, 64, -128, 0),# generic_blue_thresholds
              (10, 88, 14, 127, -45, 79),#redball
              (29, 87, -36, 1, 38, 58),#球网整体
              (57, 69, -45, -14, 28, 46),#球框
              (28, 86, -40, 11, 25, 57)] #测试样例

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()
LEDB = LED(3)                  # 蓝色LED

uart = UART(3, 115200)         # 初始化串口3，波特率115200 --》母板上 UART接口

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.

while True:
    clock.tick()  # 保持固定帧率，以控制循环速度
    img = sensor.snapshot()  # 获取摄像头的当前图像帧

    red_left = False
    red_right = False
    red_straight = False
    green_straight = False
    green_retreat = False
    start_circle = False
    near_goal = False

    for blob_green in img.find_blobs([thresholds[4]], pixels_threshold=50, area_threshold=50, merge=True):
        if blob_green.code()==1:
            #距离球门过近
            if blob_green.area()>60000:
                uart.write("%d" % 5)
                print("RD: 5")
                near_goal = True
            else:
                continue

    # 在图像中查找红色色块
    for blob_red in img.find_blobs([thresholds[3]], pixels_threshold=30, area_threshold=30, merge=True):
        if blob_red.code() == 1:
            print("red_area:",blob_red.area())
            #判断是否距离红球过远
            if blob_red.area()<10000:
                # 获取色块的中心坐标
                cx = blob_red.cx()
                cy = blob_red.cy()
                #print(clock.fps())
                #print("cx:", cx)
                #print("cy：",cy)
                # 根据色块的位置发送不同的数字给UART串口
                if cx < img.width() // 3:  # 色块在左侧
                    uart.write("%d" % 3)  # 发送数字3
                    #print("RD: 3")
                    red_left = True
                elif cx > 2 * img.width() // 3:  # 色块在右侧
                    uart.write("%d" % 2)  # 发送数字2
                    #print("RD: 2")
                    red_right = True
                elif cx>img.width() //3 and cx<2 * img.width()//3:    # 色块在中间位置
                    uart.write("%d" % 1)
                    #print("RD: 1")
                    red_straight = True    # 发送数字1

                if uart.any():  # 如果接收到任何消息
                    receive = uart.read().decode().strip()  # 将接收到的消息解码并去除首尾空格
                    #print(receive)
                    # 如果色块的纵横比大于0.5，认为它是非圆形的
                if blob_red.elongation() > 0.5:
                    img.draw_edges(blob_red.min_corners(), color=(255, 0, 0))  # 在色块的最小角点画边界
                    img.draw_line(blob_red.major_axis_line(), color=(0, 255, 0))  # 画出色块的主轴线
                    img.draw_line(blob_red.minor_axis_line(), color=(0, 0, 255))  # 画出色块的次轴线
            else:
                start_circle = True
                red_straight = True
                for blob_green in img.find_blobs([thresholds[4]], pixels_threshold=50, area_threshold=50, merge=True):
                    if blob_green.code()==1:
                        print("green_area:",blob_green.area())
                        green_cx = blob_green.cx()
                        print(clock.fps())
                        print("green_cx:", green_cx)
                        # 如果色块的纵横比大于0.5，认为它是非圆形的
                        if blob_green.elongation() > 0.5:
                            img.draw_edges(blob_green.min_corners(), color=(255, 0, 0))  # 在色块的最小角点画边界
                            img.draw_line(blob_green.major_axis_line(), color=(0, 255, 0))  # 画出色块的主轴线
                            img.draw_line(blob_green.minor_axis_line(), color=(0, 0, 255))  # 画出色块的次轴线

                        if green_cx>img.width() //4 and green_cx<3 * img.width()//4:

                            #距离球门过近
                            if blob_green.area()>70000:
                                uart.write("%d" % 5)
                                print("RD: 5")
                                green_retreat = True
                            else:
                                uart.write("%d" % 1)
                                print("RD: 1")
                                green_straight = True    # 发送数字1
                                # 如果色块的纵横比大于0.5，认为它是非圆形的


        # 画出色块的矩形框
        img.draw_rectangle(blob_red.rect())
        # 在色块的中心点画一个十字
        img.draw_cross(blob_red.cx(), blob_red.cy())

        # 在色块的中心点附近画一个关键点，标记出色块的旋转角度
        img.draw_keypoints([(blob_red.cx(), blob_red.cy(), int(math.degrees(blob_red.rotation())))], size=20)

    # 如果没有检测到红色色块，发送默认数字给UART串口
    if not red_left and not red_right and not red_straight and not near_goal:
        uart.write("%d" % 0)  # 发送数字0表示没有检测到红色色块
        print("RD: 0")
    if not green_straight and not green_retreat and start_circle:
        uart.write("%d" % 4)  # 发送数字4, 表示需要转圈了
        print("RD: 4")
    # 打印当前帧率
    #print(clock.fps())
    #print("width:",img.width()//3)







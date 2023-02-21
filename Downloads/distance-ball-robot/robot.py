# -*- coding: utf-8 -*-

import cv2
import numpy as np
import mss
import serial
import time
import math
from matplotlib import pyplot as plt
import copy

#カメラキャリブレーション-------------------------------------
fx=687.69502062
fy=689.65444106
cx=316.32462213
cy=260.7029882
mtx = np.array([[fx, 0, cx],[0, fy, cy],[0, 0, 1]])

k1=-4.40275331e-01
k2=2.66910647e-01
p1=3.94470747e-06
p2=-2.39415957e-03
k3=-2.57563018e-01
dist = np.array([[k1,k2,p1,p2,k3]])

#設定項目----------------------------------------------
capture_loc = (150, 155, 950, 550) #left, top, width, height
QR_SIZE = 100

#初期のグローバル変数定義----------------------------------
qcd = cv2.QRCodeDetector()
machine = [0,0,0]
qr_locs = []        #(y,x,color)
qr_detects = []     #(detect_time, distance, color)
color_list = []
key = -1
start = None
start_color = None
before_points = None
last_order = None
before = None
frame_count = 0

degree = 180
delay = 350

lag = 0
pred_loc = (0,0,0)
rv = 0
v = 0
target_color = None
target_loc = None


#カメラゆがみ補正
def undistort(img):
    h,w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    return dst

#Discord画面をキャプチャ
def SCT(bbox):
    with mss.mss() as sct:
        left, top, width, height = capture_loc
        grab_area = {'left': left, 'top': top, 'width': width, 'height': height}
        img = np.array(sct.grab(grab_area))
        return img

#目的地を入力すると、移動距離と目指す角度、その角度までの最短移動量を返す
def getNextMove(dest):
    dy = dest[0] - machine[0]
    dx = dest[1] - machine[1]
    theta = math.degrees(math.atan2(dy, dx))
    if theta < 0:
        theta += 360
    distance = math.sqrt((dy**2 + dx**2))
    dt = theta - machine[2]
    if abs(dt) > 180:
        dt = np.sign(dt)*(360-abs(dt))*-1
    print('getNextMove:' ,theta, distance, dt)
    return theta, distance, dt

#内容の読めていないQRコードをフィルタリング＆カラーデータを返す
def getQRData(img):
    retval, decoded_info, points , _ = qcd.detectAndDecodeMulti(img)
    if retval:
        texts = []
        corners = []
        colors = []
        for s, p in zip(decoded_info, points):
            if s != '':
                sp = s.split(',')
                color = (int(sp[0]), int(sp[1]), int(sp[2]), int(sp[3]), int(sp[4]), int(sp[5]))
                texts.append(s)
                corners.append(p)
                colors.append(color)
        if not texts:
            retval = False
        return (retval, texts, corners, colors)
    return (retval, decoded_info, points, ())

#QRコードがある程度綺麗な四角形で判定できているか
def isCorrectQR(points):
    side_diff = abs((points[0,1]+points[3,1]) - (points[1,1]+points[2,1]))
    return (side_diff < 10)
"""
def isMove(points):
    if before_points is None:
        return False
    move_diff = abs(before_points[0,1]-points[0,1])
    print(move_diff)
    return (move_diff > 0.1)
"""

def isCenter(img, points):
    check = abs((img.shape[1]/2) - ((p[0,0] + p[1,0] + p[2,0] + p[3,0]) / 4))
    print('center rate: '+str(check))
    return (check < 30)

def write(ser, message, count):
    global last_order
    text = message + str(count)
    if text != last_order:
        ser.write(text.encode('utf-8'))
        print('send order ['+text+']')
        last_order = text
        
def timerStart():
    global start
    start = time.perf_counter()
    
def getTime():
    end = time.perf_counter()
    return end-start

def controll():
    global delay
    global degree
    global key
    if key == int(ord('e')):
        write(ser, 'e', 10)
        key = -1
    elif key == int(ord('d')):
        write(ser, 'd', 10)
        key = -1
    elif key == int(ord('a')):
        write(ser, 'a', 10)
        key = -1
    elif key == int(ord('w')):
        write(ser, 'w', 10)
        key = -1
    elif key == int(ord('s')):
        write(ser, 's', 10)
        key = -1
    elif key == int(ord('z')):
        write(ser, 'z', -1)
        key = -1
    elif key == int(ord('k')):
        write(ser, 'k', -1)
        delay = delay + 5
        print(degree, delay)
        key = -1
    elif key == int(ord('m')):
        write(ser, 'm', -1)
        delay = delay - 5
        print(degree, delay)
        key = -1
    elif key == int(ord('o')):
        write(ser, 'o', -1)
        delay = delay + 10
        print(degree, delay)
        key = -1
    elif key == int(ord('n')):
        write(ser, 'n', 10)
        delay = delay - 10
        print(degree, delay)
        key = -1
    
def isMove(img, first_ret):
    global before
    cpyimg = copy.copy(img)
    size = (160, 75)
    gray = cv2.resize(cpyimg, size)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if before is None:
        before = gray
        return first_ret
    img_diff = cv2.absdiff(gray, before)
    before = gray
    num = np.sum(img_diff)
    print(num)
    return (num > 800000) or num == 0

def delayFrame():
    global frame_count
    frame_count = frame_count + 1
    if frame_count % 2 == 1:
        return True
    else:
        return False
    
def label(frame_mask, img):
    retval, labels, stats, centroids = cv2.connectedComponentsWithStats(frame_mask)
    if retval != 0:
        for coordinate in stats[1:]:
            print(coordinate)
            if coordinate[4] > 2500:
                check = abs((img.shape[1]/2)-coordinate[1])
                return ((check < 50), int(coordinate[0]), int(coordinate[1]))
    return (False, -999,-999)
    
def findBall(img, color):
    H, S, V, HT, ST, VT = color
    cpyimg = copy.copy(img)
    hsv = cv2.cvtColor(cpyimg, cv2.COLOR_BGR2HSV)
    lower = np.array([max(0,H-HT), max(0,S-ST), max(0, V-VT)])
    upper = np.array([min(255,H+HT), min(255, S+ST), min(255, V+VT)])
    frame_mask = cv2.inRange(hsv, lower, upper)
    frame_mask = cv2.medianBlur(frame_mask, ksize=23)
    return label(frame_mask, cpyimg)

def getDest(distance):
    T = 350
    dy = distance * math.sin(math.radians(machine[2]))
    dx = distance * math.cos(math.radians(machine[2]))
    fx = machine[1] + dx
    fy = machine[0] + dy
    rate_x = abs((abs(fx)-T)/fx)
    rate_y = abs((abs(fy)-T)/fy)
    if abs(fx) <= T:
        rate_x = 0
    if abs(fy) <= T:
        rate_y = 0
    rate = max(rate_x, rate_y)
    if rate > 0:
        if abs(dx) >= 1e-10:
            fx = fx*(1-rate)
        if abs(dy) >= 1e-10:
            fy = fy*(1-rate)
    return (fy, fx)

def getDistance(dest):
    dy = dest[0] - machine[0]
    dx = dest[1] - machine[1]
    return math.sqrt((dy**2 + dx**2))

def waitTime(sec):
    aft = getTime()
    return (aft > sec)

def getQRLocation(color):
    qrs = []
    for y, x, c in qr_locs:
        if c == color:
            qrs.append((y,x))
    return np.average(qrs, axis=0)
    
    
#ここからループ処理---------------------------------------------------------

try:
    ser = serial.Serial("COM10", 57600)
    ser.write(b'e')
    while key != int(ord('q')):
        img = SCT((150, 150, 850, 550))
        retval, decoded_info, points, colors = getQRData(img)
        if not retval:
            before_points = None
        if retval and key == int(ord('1')):
            machine = [0,0,0]
            qr_detects = []
            qr_locs = []
            start = None
            start_color = None
            before = None
            timerStart()
            write(ser, 'a', -1)     #反時計回りに回転開始
            print('go')
            key = 2
        if retval and key == 2:
            if isMove(img, False):
                lag = getTime()
                print('move!')
                print(lag)
                key = 3
            before_points = points[0]
        if retval and key == 3:
            for s, p, c in zip(decoded_info, points, colors):
                if isCenter(img, p):
                    if start_color is None:
                        start_color = c
                        timerStart()
                        print(start_color)
                    distance = (QR_SIZE * fy) / abs(((p[0,1]+p[1,1])/2)-((p[2,1]+p[3,1])/2))
                    prog_time = getTime()
                    qr_detects.append((prog_time, distance, c))
                    if start_color == c and prog_time > 5:
                        print("end",c,start_color)
                        write(ser, 'e', -1) #1周し終えたら停止
                        key = 4
        if retval and key == 4:
            clock = getTime()   #一周するのにかかった時間
            rv = 360/clock
            timerStart()
            for t, s, c in qr_detects:
                rad = 360*(t/clock)
                the = math.radians(rad)
                qr_locs.append((s*math.sin(the), s*math.cos(the), c))
            y, x, c = zip(*qr_locs)
            plt.scatter(x, y)
            plt.show()
            key = 5
        if key == 5:
            if not isMove(img, True) and delayFrame():
                lag = getTime()
                print(lag)
                count = int(lag*10)
                write(ser, 'd', count+1)
                timerStart()
                key = 6
        if retval and key == 6:
            if waitTime(lag+5):
                print(clock)
                write(ser, 'e', -1)
                if isCenter(img, points[0]):
                    p = points[0]
                    key = -1
                    distance = (QR_SIZE * fy) / abs(((p[0,1]+p[1,1])/2)-((p[2,1]+p[3,1])/2))
                    print('center!')
                else:
                    print('not center!')
        if key == int(ord('2')) and retval:
            write(ser, 'w', 30)
            before = None
            timerStart()
            key = 7
        if key == 7 and retval:
            if waitTime(5) and not isMove(img, True):
                p = points[0]
                after_dis = (QR_SIZE * fy) / abs(((p[0,1]+p[1,1])/2)-((p[2,1]+p[3,1])/2))
                v = (distance - after_dis) / 3
                print(distance, after_dis, v)
                write(ser, 's', 30)
                timerStart()
                before = None
                key = 8
        if key == 8 and retval:
            if waitTime(5) and not isMove(img, True):
                p = points[0]
                dista = (QR_SIZE * fy) / abs(((p[0,1]+p[1,1])/2)-((p[2,1]+p[3,1])/2))
                print(dista)
                key = -1
        if key == int(ord('3')):
            count = int((dista / v)*10)
            print(count)
            write(ser, 'w', count)
            key = -1
        if key == int(ord('4')):
            for y,x,c in qr_locs:
                if c not in color_list:
                    color_list.append(c)
            print(color_list)
            before = None
            key = -1
        if key == int(ord('5')):
            count = int((360/rv)*10)
            write(ser, 'a', count)
            if isMove(img, False):
                timerStart()
                key = 10
        if key == 10:
            for c in color_list:
                retval, y,x = findBall(img, c)
                if retval:
                    dr = rv*getTime()
                    machine[2] = (dr + machine[2]) % 360
                    print(machine)
                    key = 11
                    target_color = c
                    print(c)
                    before = None
                    timerStart()
                    write(ser, 'e', -1)
                    break
        if key == 11:
            if not isMove(img, True):
                lag = getTime()
                print(lag)
                count = int(lag*10)
                key = 12
                timerStart()
        if key == 12:
            if waitTime(5):
                write(ser, 'd', count)
                timerStart()
                key = 13
        if key == 13:
            if waitTime(lag+5):
                dest = getDest(400)
                distance = getDistance(dest)
                count = int((distance / v)*10)
                machine[0] = dest[0]
                machine[1] = dest[1]
                write(ser, 'x', count)  #ボールを拾いに向かう
                print('machine', machine)
                timerStart()
                key = -1
        if key == int(ord('6')):
            if waitTime((count/10)+5):
                qr_loc = getQRLocation(target_color)
                md, dis, deg = getNextMove((qr_loc/3)) #QRコードと中心を結ぶ線の1:2点に行く
                machine[0] = qr_loc[0]/3
                machine[1] = qr_loc[1]/3
                machine[2] = md
                print('machine2', machine,qr_loc)
                count = int((deg/rv)*10)
                if count > 0:
                    write(ser, 'a', count)
                else:
                    write(ser,'d', abs(count))
                timerStart()
                key = 14
        if key == 14:
            if waitTime((abs(count)/10)+5):
                count = int((dis / v)*10)
                write(ser, 'w', count)
                key = 15
                timerStart()
        if key == 15:
            if waitTime((abs(count)/10)+5):
                md, dis, deg = getNextMove(qr_loc)
                machine[2] = md
                count = int(((deg/rv)*10)+10)
                if count > 0:
                    write(ser, 'a', count)
                else:
                    write(ser,'d', abs(count))
                timerStart()
                key = 16
        if key == 16:
            if waitTime(8):
                key == -1
        if key == 16 and retval:
            p = points[0]
            c = colors[0]
            if isCenter(img, p) and target_color == c:
                timerStart()
                before = None
                key = 17
        if key == 17:
                if not isMove(img, True):
                    lag = getTime()
                    print(lag)
                    count = int(lag*10)
                    key = 18
                    timerStart()
        if key == 18:
            if waitTime(5):
                write(ser, 'd', count)
                timerStart()
                key = -1
        if key == int(ord('7')):
            if waitTime((abs(count)/10)+5):
                count = int((((3*dis)/5) / v)*10)
                machine[0] = (3*(machine[0]+qr_loc[0]))/5
                machine[1] = (3*(machine[1]+qr_loc[1]))/5
                write(ser, 'w', count)
                key = 19
                timerStart()
        if key == 19:
            if waitTime((count/10)+5):
                write(ser, 'z', -1)
                key = -1
        if key == int(ord('8')):
            dis = getDistance((0,0))
            machine[0] = 0
            machine[1] = 0
            count = int((dis / v)*10)
            write(ser, 's', count)
            target_color = None
            key = -1
                        
        controll()        
        cv2.imshow('QR_Image', img)
        
        input = cv2.waitKey(1)
        if input != -1:
            key = input
finally:
    print('end')
    ser.close()
    cv2.destroyAllWindows()
        
        
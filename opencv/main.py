import numpy as np
import cv2, random, math, time

import sys
import os
import signal


def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

lidar_range = []
image = np.empty(shape=[0])
pub = None
Width = 1280
Height = 720
Offset = 600
Gap = 40
flag = 0
limit = 0
c = 0
mission = 0





# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1 + Offset), (x2, y2 + Offset), color, 2)
    return img


# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                  (lpos + 5, 25 + offset),
                  (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                  (rpos + 5, 25 + offset),
                  (0, 255, 0), 2)
    cv2.rectangle(img, (center - 5, 15 + offset),
                  (center + 5, 25 + offset),
                  (0, 255, 0), 2)
    cv2.rectangle(img, (315, 15 + offset),
                  (325, 25 + offset),
                  (0, 0, 255), 2)
    return img


# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2 - y1) / float(x2 - x1)

        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width / 2 - 90):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width / 2 + 90):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines


# get average m, b of lines
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = x_sum / (size * 2)
    y_avg = y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b


# get lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)

    if m == 0 and b == 0:
        if left:
            pos = 0
        if right:
            pos = Width
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height / 2) - b) / float(m)

        cv2.line(img, (int(x1), int(Height)), (int(x2), int(Height/2)), (0, 255, 0), 3)

    return img, int(pos)


# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap

    # gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 0
    high_threshold = 300
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)


    # HoughLinesP
    roi = edge_img[0: Offset + Gap, 0: Width]
    all_lines = cv2.HoughLinesP(roi, 1, math.pi / 180, 30, 30, 10)


    # divide left, right lines
    try:
        left_lines, right_lines = divide_left_right(all_lines)

        # get center of lines
        frame, lpos = get_line_pos(frame, left_lines, left=True)
        frame, rpos = get_line_pos(frame, right_lines, right=True)
        # draw lines
        # frame = draw_lines(frame, left_lines)
        # frame = draw_lines(frame, right_lines)
        center = (lpos+rpos)/2
        frame = cv2.line(frame, (int(center), 400), (int(center), 720), (255, 0, 255), 2)

        # draw rectangle
        # frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
        # roi2 = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
        # roi2 = draw_rectangle(roi2, lpos, rpos)

        # frame = get_line_pos(frame,(0,center),(400,10),(255,0,0),2)
        cv2.imshow('Canny Edge Detection', frame)
        return lpos,rpos
    except:
        cv2.imshow('Canny Edge Detection',frame)
        return 0,1280
    # return lpos, rpos




    
def start():
    global pub
    global image
    global Gap
    global Width, Height
    global c, mission

    pid_P = 0.7
    pid_I = 0
    pid_D = 0
    sum_angle = 0
    mission = 0
    cnt = 0
    prev_angle = 0
    # Initialize the camera device.
    cap = cv2.VideoCapture(0)

    # Make sure the cam is initialized.
    if not cap.isOpened():
        print("Unable to open camera")

    print('START!!!!')
    mission_start = 0

    while True:
        # Read frames from the video stream.
        ret, frame = cap.read()
        lpos,rpos = process_image(frame)

        if not ret:
            break
        center = (lpos + rpos) / 2
        print(center)
        angle = -(Width / 2 - center)
        sum_angle += angle
        diff_angle = angle - prev_angle
        c = pid_P * angle + pid_I * sum_angle + pid_D * (diff_angle)
        prev_angle = angle
        prev_angle = angle
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    start()
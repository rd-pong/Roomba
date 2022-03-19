import math
import random
import time

import matplotlib.image as imgplt
import numpy as np
import serial
from matplotlib import pyplot as plt  # export MPLBACKEND=Agg
from pycreate2 import Create2

center = [70, 55]
[x_pos, y_pos] = center
cur_ang = 0
start_time = time.time()
img_2 = imgplt.imread('images/map1.png')
plt.imshow(img_2, cmap='Greys_r', zorder=0)
plt.xticks([])
plt.yticks([])
plt.axis('off')
pause = True


def draw_line(start, end):
    global start_time
    '''
    start, end: int[2]
    '''
    zoom = 0.1
    print(start, end)
    x = np.array([center[0] + start[0] * zoom, center[0] + end[0] * zoom])
    y = np.array([center[0] + start[1] * zoom, center[0] + end[1] * zoom])
    plt.plot(x, y, zorder=1)
    if time.time() - start_time > 1:
        plt.savefig('images/PGM/map1_new.png')
        start_time = time.time()


def free_walk(bot, ser):
    global x_pos, y_pos, cur_ang
    bot.drive_direct(100,100)
    [last_x, last_y] = [x_pos, y_pos]
    for i in range(5):
        sensors = bot.get_sensors()
        bp = sensors.light_bumper
        bp_list = [bp.left, bp.front_left, bp.center_left, bp.center_right, bp.front_right, bp.right]
        distance = sensors.distance / 10
        angle = sensors.angle
        old_x, old_y = x_pos, y_pos
        cur_ang = (cur_ang + angle) % 360
        x_pos += math.sin(cur_ang / 180 * math.pi) * distance
        y_pos -= math.cos(cur_ang / 180 * math.pi) * distance
        # draw_line([old_x, old_y], [x_pos, y_pos])
        if bp_list.count(False) < 6:
            break
        time.sleep(random.randint(0, 5) * 0.4 / 5)
    # draw_line([last_x, last_y], [x_pos, y_pos])
    if random.randint(0, 1):
        ser.write(b'\x92\x00\x3F\xFF\xC1')  # counterclockwise
    else:
        ser.write(b'\x92\xFF\xC1\x00\x3F')  # clockwise
    time.sleep(random.randint(0, 20) * 0.03)


def turn_90(bot, bps, ser):
    global cur_ang
    lc, rc = bps[:3].count(True), bps[3:].count(True)
    if lc > rc:
        ser.write(b'\x92\xFF\xC1\x00\x3F')  # clockwise
    elif rc > lc:
        ser.write(b'\x92\x00\x3F\xFF\xC1')  # counterclockwise
    else:
        if random.randint(0, 1):
            ser.write(b'\x92\x00\x3F\xFF\xC1')  # counterclockwise
        else:
            ser.write(b'\x92\xFF\xC1\x00\x3F')  # clockwise
    for i in range(5):
        time.sleep(0.1 / 5 * 25)
        sensors = bot.get_sensors()
        cur_ang = (cur_ang + sensors.angle) % 360
        bp = sensors.light_bumper
        bp_list = [bp.left, bp.front_left, bp.center_left, bp.center_right, bp.front_right, bp.right]
        if bp_list.count(False) >= 6:
            break
    time.sleep(0.5)


def switch(_pause):
    global pause
    pause = _pause


def random_walk(bot, ser):
    while True:
        if (pause):
            time.sleep(1)
            continue
        sensors = bot.get_sensors()
        bp = sensors.light_bumper
        bp_list = [bp.left, bp.front_left, bp.center_left, bp.center_right, bp.front_right, bp.right]
        print(bp_list)
        num_falses = bp_list.count(False)
        if num_falses == 6:
            free_walk(bot, ser)
        else:
            turn_90(bot, bp_list, ser)


x_pos, y_pos = 62, 62

if __name__ == '__main__':
    ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)
    bot = Create2('/dev/ttyUSB0', 115200)
    bot.start()
    bot.safe()

    # x_pos, y_pos = 62, 62
    # cur_ang = 0 # 0-359
    random_walk(bot, ser)

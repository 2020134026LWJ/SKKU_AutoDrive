# Source Generated with Decompyle++
# File: decision_making_func_lib.cpython-310.pyc (Python 3.10)

import cv2
import numpy as np
import sys
import os
from typing import Tuple
message = "\n _   _        \n| | | |       \n| |_| | _____ \n|  _  ||_____|\n|_| |_|       \n              \n __  __         _      _  _  _  _           \n|  \\/  |  ___  | |__  (_)| |(_)| |_  _   _  \n| |\\/| | / _ \\ | '_ \\ | || || || __|| | | | \n| |  | || (_) || |_) || || || || |_ | |_| | \n|_|  |_| \\___/ |_.__/ |_||_||_| \\__| \\__, | \n                                     |___/  \n  ____  _                  \n / ___|| |  __ _  ___  ___ \n| |    | | / _` |/ __|/ __|\n| |___ | || (_| |\\__ \\__  \\\n \\____||_| \\__,_||___/|___/\n                           \n\n"
print(message)
print('H-Mobility Class 자율주행 심화과정')
print('Sungkyunkwan University Automation Lab.')
print('')
print('------------------Authors------------------')
print('Hyeong-Keun Hong <whaihong@g.skku.edu>')
print('Siwoo Lee <edenlee@g.skku.edu>')
print('Jinsun Lee <with23skku@g.skku.edu>')
print('Gyu-Hyeon Hwang <rbgus7080@g.skku.edu>')
print('Eun-Ho Kim <dmsghdmstj@g.skku.edu> ')
print('Yeonggwang Choi <dudrhkd7811@g.skku.edu>')
print('Young-Hoon Suh <dudgns0407@g.skku.edu>')
print('Se Jeong Lim <tpwjd218@naver.com>')
print('HyeokJun Choi <mick95@naver.com>')
print('Seong-Hyeon Lim <forfortuna@skku.edu>')
print('------------------------------------------')

def calculate_slope_between_points(p1, p2):
    p1_x = p1[0]
    p1_y = p1[1]
    p2_x = p2[0]
    p2_y = p2[1]
    if p1_y == p2_y:
        slope = 'inf'
        return slope
    slope = np.arctan((p2_x - p1_x) / (p1_y - p2_y)) * 180 / np.pi
    return slope


# 주조 과정
import rclpy
import time
from copy import deepcopy

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from dsr_common2.imp import DR_init
from dsr_common2.imp.DR_common2 import posx, posj
from tcp.tcp_server import TCPServer
from blacksmith_robot.stop_motion import MovejStop

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init(args=None)
node = rclpy.create_node("qc_management", namespace=ROBOT_ID)

DR_init.__dsr__node = node


from dsr_common2.imp.DSR_ROBOT2 import (
    movel,
    movel,
    movesj, 
    movesx, 
    move_periodic,
    task_compliance_ctrl, 
    set_desired_force, 
    check_force_condition, 
    release_compliance_ctrl, 
    release_force,
    set_tool, 
    set_tcp, 
    get_current_posx, 
    wait,
    set_digital_output,
    DR_BASE, 
    DR_TOOL, 
    DR_AXIS_X, 
    DR_AXIS_Y, 
    DR_AXIS_Z, 
    DR_FC_MOD_REL,
    OFF,
    ON,
)
DR_ACC_L = 70
DR_VEL_L = 70
DR_ACC_J = 70
DR_VEL_J = 70

## 홈 위치
JReady = [0, 0, 90, 0, 90, 0]
home = posj(JReady)

## 래들 감지
deg1 = [2.68, 46.84, 75.27, 90.16, -90.72, 35.87]   # 도 단위
p1_1 = posj(deg1)   # 컵 있는 위치로 이동
p1_2 = posx([623.690, -424.150, 226.040, 91.71, -89.53, -86.24])   # 가까이 가기 위해 앞으로 movel

# 용탕 주입 위치
pour_position = posx([420.990, -155.200, 198.160, 91.71, -89.53, -86.24])  

# 래들 복귀
deg2 = [16.86, 49.07, 60.98, 94.97, -104.06, 24.34]   # 도 단위
p5_1 = posj(deg2)   # 경유점1

deg3 = [-12.66, 47.76, 68.60, 83.15, -77.03, 31.00]   # 도 단위
p5_2 = posj(deg3)   # 경유점2

deg4 = [-11.6, 48.95, 73.14, 80.8, -79.49, 37.23]   # 도 단위
p5_3 = posj(deg4)   # 경유점3

set_tool("TCP208mm")
set_tcp("GripperSA_rg2_250509")


# ────── 그리퍼 동작 ──────
def release():  # 열기
    set_digital_output(1, ON)
    set_digital_output(2, ON)

def grip():     # 닫기
    set_digital_output(1, OFF)
    set_digital_output(2, OFF)

def grip_cup(): # 래들 크기만큼만 닫기
    set_digital_output(1, ON)
    set_digital_output(2, OFF)


# ────── 주조 과정 클래스 ──────
class Casting:
    def __init__(self):
        self.ms = MovejStop()

    # 용광로 위치로 이동 및 래들 수거
    def move_furnace_position(self):
        print("용광로 위치로 이동 중...")
        # 충격 감지 시, 정지
        self.ms.move_j(p1_1, vel=DR_VEL_J, acc=DR_ACC_J)
        movel(p1_2, vel=DR_VEL_L, acc=DR_ACC_L)
        
        print("래들 감지 중...")
        # 순응제어 및 (-)y방향 힘
        task_compliance_ctrl(stx=[20000, 200, 20000, 20, 20, 20])
        time.sleep(0.1)
        set_desired_force(fd=[0, -5, 0, 0, 0, 0], dir=[0, 1, 0, 0, 0, 0], time=0.0, mod=DR_FC_MOD_REL)

        # 감지 시, 해당 좌표 받기
        while True:
            if check_force_condition(axis=DR_AXIS_Z, max=2.5, ref=DR_TOOL):
                release_force()
                release_compliance_ctrl()
                wait(0.5)

                # 감지된 좌표
                self.cup_position = get_current_posx()[0]   
                print("래들 감지 완료")  
                print(self.cup_position)
                break
        
        print("래들 수거 중...")  
        # 감지된 좌표에서 뒤로 이동 후, 그리퍼 열기
        self.target_posx1 = deepcopy(self.cup_position)
        self.target_posx1[1] += 70.0
        movel(posx(self.target_posx1), vel=DR_VEL_L, acc=DR_ACC_L, ref=DR_BASE)
        release()

        # 뒤로 움직인 거리 + 래들의 크기만큼 이동 후, 그리퍼 닫기
        self.target_posx1[1] -= 126.0
        movel(posx(self.target_posx1), vel=DR_VEL_L, acc=DR_ACC_L, ref=DR_BASE)
        grip_cup()
        print("래들 수거 완료")  

    # 거푸집 위치로 용탕 이송
    def move_mould_position(self):
        # 용광로에서 나올 때, 걸리지 않게 z축 방향으로 올리기
        self.target_posx2 = deepcopy(self.target_posx1)
        self.target_posx2[2] += 31.0
        movel(posx(self.target_posx2), vel=DR_VEL_L, acc=DR_ACC_L, ref=DR_BASE)

        # 용탕이 넘치지 않게 주의
        # 1. z축 방향은 고정
        # 2. 로봇 관절 움직임 최소
        self.target_posx2[0] += 64.28
        self.target_posx2[1] += 367.26
        x1 = posx(self.target_posx2)
        self.target_posx2[0] -= 164.03
        self.target_posx2[1] += 249.57
        x2 = posx(self.target_posx2)
        self.target_posx2[0] -= 99.31
        self.target_posx2[1] -= 167.14
        x3 = posx(self.target_posx2)
        
        print("거푸집 위치로 이동 중...")
        movesx([x1, x2, x3], vel=[50, 20], acc=[60, 30])
        print("이동 완료")

    # 용탕 균일화 및 용탕 주입
    def pour_action(self):
        # 용탕 균일화 - 불순물과 온도 편차를 줄이기
        print("용탕 균일화 중...")
        move_periodic(
            amp=[15.0, 20.0, 0.0, 0.0, 0.0, 0.0],
            period=[2.4, 1.8, 0.0, 0.0, 0.0, 0.0],
            atime=0.2,
            repeat=self.repeat,
            ref=DR_BASE
        )
        print("용탕 균일화 완료")  

        # 용탕 주입 - 튀지 않게 2번의 과정으로 주입하기
        movel(pour_position, vel=DR_VEL_L, acc=DR_ACC_L)

        print("용탕 주입 중(1/2)...")   # 천천히 용탕의 흐름 만들기
        self.pour_pose = deepcopy(pour_position)
        self.pour_pose[3:] = [94.42, -157.26, -82.15]
        movel(posx(self.pour_pose), vel=18, acc=15)
        wait(0.5)

        print("용탕 주입 중(2/2)...")   # 나머지 모두 주입
        self.pour_pose[3:] = [82.71, 166.37, -93.31]
        movel(posx(self.pour_pose), vel=24, acc=20)
        
        print("용탕 주입 완료")
        movel(pour_position, vel=DR_VEL_L, acc=DR_ACC_L)

    # 래들 원위치
    def ladle_return(self):
        print("래들 원위치 중...")
        movesj([p5_1, p5_2, p5_3], vel=DR_VEL_J, acc=DR_ACC_J)

        release()   # 그리퍼 열기
        print("래들 원위치 완료") 

    def run(self, periodic_repeat=5):
        self.repeat = periodic_repeat
        
        # 초기 setting 
        release()
        time.sleep(0.1)
        grip()
        time.sleep(0.1)
        self.ms.move_j(home, vel=DR_VEL_J, acc=DR_ACC_J)

        print("주조 공정 시작")  
        self.move_furnace_position()    # 용광로 위치로 이동 및 래들 수거
        wait(1)
        self.move_mould_position()      # 거푸집 위치로 용탕 이송
        wait(1)
        self.pour_action()              # 용탕 균일화 및 용탕 주입
        wait(1)
        self.ladle_return()             # 래들 원위치
        self.ms.move_j(home, vel=DR_VEL_J, acc=DR_ACC_J)
        grip()
        print("주조 공정 완료")
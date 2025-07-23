# 단조 과정
"""
    1. Holding motion
    2. Hammering motion
    3. Grinding Vertical motion
    4. Grinding Horizontal motion
    5. Tweak motion
"""

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
    amove_periodic,
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
    get_digital_input,
    DR_BASE, 
    DR_TOOL, 
    DR_AXIS_X, 
    DR_AXIS_Y, 
    DR_AXIS_Z, 
    DR_FC_MOD_REL,
    OFF,
    ON,
)
# 위치 도달 확인용 허용 오차
POS_TOLERANCE = 10.0  # mm 단위

POS_J = '1'
POS_X = '0'

GRIP = 1
RELEASE = 2

DR_ACC_L = 70
DR_VEL_L = 70
DR_ACC_J = 70
DR_VEL_J = 70

VELOCITY, ACC = 100, 100

TARGET_FORCE = 10

CHECK_FORCE = 2.4

COMPLIANCE_AXIS = 2000
COMPLIANCE = 20

# 전역변수
wait_poseJ = [1.78,38.87,88.84,-179.96,127.93,-93.51]
moru_J=[4.25,34.93,57.56,0.22,87.55,4.31]

holding1J=[33.59,53.62,28.86,-36.83,114.41,20.42]
holding2J=[29.45,51.93,25.10,-25.24,112.71,20.01]
holding3J=[26.12,50.26,23.14,-16.19,112.55,18.50]
holding4J=[22.24,49.32,22.25,-8.86,111.96,18.50]

## 0. 집으로 이동
JReady = [0, 0, 90, 0, 90, 0]
home = posj(JReady)

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

# ────── 수평 연마 클래스 ──────
class GrindingTask:
    def __init__(self):
        self.J_READY = [0, 0, 90, 0, 90, 0]
        self.wait_poseJ = [1.78, 38.87, 88.84, -179.96, 127.93, -93.51]
        self.PRE_ANVIL_CONTACT = posx([820.91, -297.59, 503.38, 123.22, -90.2, 28.28])
        self.AFTER_ANVIL_CONTACT = posx([926.79, -56.24, 306.90, 159.42, -115.64, 62.35])
        self.ANVIL_CONTACT = posx([671.92, -358.47, 433.59, 95.53, -92.94, 0.53])
        self.mj=MovejStop()

        self.Z_FORCE = -60.0
        self.AMP_Y = 25.0
        self.PERIOD_Y = 0.7
        self.ACC_TIME = 0.3
        self.SLEEP_MARGIN = 1.0
        self.update_repeat(15)

    def update_repeat(self, repeat_cnt):
        self.repeat_cnt = repeat_cnt
        self.repeat_time = self.PERIOD_Y * self.repeat_cnt

    def set_tool_and_tcp(self):
        set_tool("Tool Weight_3_24")
        set_tcp("TCP208mm")

    def move_to_initial_pose(self):
        print("[INFO] 초기 자세로 이동")
        self.mj.move_j(self.J_READY, vel=60, acc=ACC)

    def move_to_initial_grip_pose(self):
        print("[INFO] 초기 수령 자세로 이동")
        self.mj.move_j(self.wait_poseJ, vel=60, acc=ACC)

    def wait_tool(self):
        release()
        time.sleep(0.5)
        print('터치 감지 대기중...')
        while True:
            if (
                check_force_condition(axis=DR_AXIS_Z, max=20, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_Y, max=20, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_X, max=20, ref=DR_TOOL)
            ):
                print('터치 감지 되었습니다.')
                time.sleep(3)
                grip()
                print('잡았습니다.')
                time.sleep(0.5)
                break

    def release_tool(self):
        time.sleep(0.5)
        print('터치 감지 대기중...')
        while True:
            if (
                check_force_condition(axis=DR_AXIS_Z, max=20, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_Y, max=20, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_X, max=20, ref=DR_TOOL)
            ):
                print('터치 감지 되었습니다.')
                time.sleep(3)
                release()
                print('release 성공.')
                time.sleep(0.5)
                break

    def move_to_anvil_contact(self):
        print("[INFO] 모루 접촉 지점으로 이동")
        movel(self.AFTER_ANVIL_CONTACT, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(self.PRE_ANVIL_CONTACT, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(self.ANVIL_CONTACT, vel=VELOCITY, acc=ACC, ref=DR_BASE)

    def apply_compliance_and_force(self):
        print("[INFO] 순응 제어 및 Z축 힘 제어 적용")
        task_compliance_ctrl([2000, 2000, 200, 300, 300, 300])
        set_desired_force(
            [0, 0, self.Z_FORCE, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            DR_FC_MOD_REL
        )

    def perform_periodic_motion(self):
        time.sleep(2.0)
        print("[INFO] amove_periodic 시작 (Y축 진동)")
        amove_periodic(
            amp=[0.0, self.AMP_Y, 0.0, 0.0, 0.0, 0.0],
            period=[0.0, self.PERIOD_Y, 0.0, 0.0, 0.0, 0.0],
            atime=self.ACC_TIME,
            repeat=self.repeat_cnt,
            ref=DR_BASE
        )
        print("[DEBUG] 진동 시간 대기")
        time.sleep(self.repeat_time + self.SLEEP_MARGIN)

    def shutdown_and_return(self):
        print("[INFO] 진동 종료 → 정지 위치 복귀")
        release_compliance_ctrl()
        movel(self.ANVIL_CONTACT, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(self.PRE_ANVIL_CONTACT, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(self.AFTER_ANVIL_CONTACT, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        self.mj.move_j(self.J_READY, vel=60, acc=ACC)

    def grinding_run(self,num=15):
        self.update_repeat(num)
        self.set_tool_and_tcp()
        self.move_to_initial_pose()
        self.move_to_initial_grip_pose()
        self.wait_tool()
        self.move_to_initial_pose()
        self.move_to_anvil_contact()
        self.apply_compliance_and_force()
        self.perform_periodic_motion()
        self.shutdown_and_return()
        self.move_to_initial_grip_pose()
        self.release_tool()


# ────── 비틀기 클래스 ──────
class SpitRotationTask:### 고유 값
    def __init__(self):
        self.J_READY = [0, 0, 90, 0, 90, 0]
        self.J_A = [-0.04, 4.78, 60.43, 0.03, 114.86, -0.02]  # A값에서 시작
        self.J_HIGH = 87.0
        self.J_LOW = -176.0
        self.repeat = 3
        self.mj=MovejStop()

    def set_tool_tcp(self):
        set_tool("Tool Weight_3_24")
        set_tcp("TCP208mm")

    def wait_tool(self):
        release()
        time.sleep(0.5)
        print('터치 감지 대기중...')
        while True:
            if (
                check_force_condition(axis=DR_AXIS_Z, max=15, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_Y, max=15, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_X, max=15, ref=DR_TOOL)
            ):
                print('터치 감지 되었습니다.')
                time.sleep(3)
                grip()
                print('잡았습니다.')
                time.sleep(0.5)
                break

    def release_tool(self):
        time.sleep(0.5)
        print('터치 감지 대기중...')
        while True:
            if (
                check_force_condition(axis=DR_AXIS_Z, max=15, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_Y, max=15, ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_X, max=15, ref=DR_TOOL)
            ):
                print('터치 감지 되었습니다.')
                time.sleep(3)
                release()
                print('release 성공.')
                time.sleep(0.5)
                break

    def move_to_initial_pose(self): ###고유 함수
        print("[INFO] 비틀기 작업 시작")
        print("[INFO] 초기 자세로 이동")
        self.mj.move_j(self.J_READY, vel=60, acc=ACC)
        print("비틀기를 할 물체를 주세요.")
        self.wait_tool()


    def rotate_twist(self):#### 고유 함수
        for i in range(self.repeat):
            print(f"[INFO] 반복 {i+1}/{self.repeat}")

            print(f"[INFO] J6이 {self.J_HIGH}로 회전")
            j_high = self.J_A.copy()
            j_high[5] = self.J_HIGH
            self.mj.move_j(j_high, vel=VELOCITY, acc=ACC)
            time.sleep(0.5)
            print("[INFO] grip")
            grip()
            time.sleep(1.0)

            print("[INFO] A지점으로 이동")
            self.mj.move_j(self.J_A, vel=VELOCITY, acc=ACC)

            print(f"[INFO] J6을 {self.J_LOW}로 회전")
            j_low = j_high.copy()
            j_low[5] = self.J_LOW
            self.mj.move_j(j_low, vel=VELOCITY, acc=ACC)
            time.sleep(0.5)
            print("[INFO] release")
            release()
            time.sleep(1.0)
            print("[INFO] A지점으로 이동")
            self.mj.move_j(self.J_A, vel=VELOCITY, acc=ACC)
            print(f"[INFO] J6이 {self.J_HIGH}로 회전")
            j_high = self.J_A.copy()
            j_high[5] = self.J_HIGH
            self.mj.move_j(j_high, vel=VELOCITY, acc=ACC)
            time.sleep(0.5)
            print("[INFO] grip")
            grip()
            time.sleep(1.0)
        print("[INFO] 비틀기 반복 완료")
        print("완료 되었습니다. 회수해주세요.")
        self.release_tool()

    def spin_run(self, repeat=3):###고유 실행 함수
        self.repeat=repeat
        self.set_tool_tcp()
        self.move_to_initial_pose()
        self.rotate_twist()


# ────── 재련 클래스 ──────
class Forge():
    def __init__(self):
        self.mj=MovejStop()
    def excep_grab(self):
            while True:
                self.wait_tool()
                time.sleep(0.5)
                a = get_digital_input(1)
                b = get_digital_input(2)
                c = get_digital_input(3)
                # print(f'a:{a}')
                # print(f'b:{b}')
                # print(f'c:{c}')
                if a == 1:
                    print('도구가 정상적으로 장착되지 않았습니다.')
                    # client_socket.sendall(b"Tool grasp failed")
                    print('다시 도구 파지를 시도합니다....')
                    time.sleep(0.2) 
                    self.release50()
                else:
                    print('도구 정상 파지 완료!')
                    # client_socket.sendall(b"Tool grasp sucseesd")
                    break
    def grip15(self):   
        set_digital_output(1,0)
        wait(0.2)
        set_digital_output(2,0)
    def release50(self):
        set_digital_output(1,1)
        wait(0.2)
        set_digital_output(2,0)

    def release_tool(self):
        time.sleep(0.5)
        print('터치 감지 대기중...')
        while(True):
            if(check_force_condition(axis=DR_AXIS_Z,max=20,ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_Y,max=20,ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_X,max=20,ref=DR_TOOL)
                ):
                print('터치 감지 되었습니다.')
                delay_time = 3
                print(f'{delay_time}초 후 놓겠습니다..')
                time.sleep(delay_time)
                
                self.release50()
                print('release 성공.')
                time.sleep(0.5)
                break


    def wait_tool(self):
        self.release50()
        time.sleep(0.5)
        print('터치 감지 대기중...')
        while(True):
            if(check_force_condition(axis=DR_AXIS_Z,max=20,ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_Y,max=20,ref=DR_TOOL) or
                check_force_condition(axis=DR_AXIS_X,max=20,ref=DR_TOOL)
                ):
                print('터치 감지 되었습니다.')
                delay_time = 3
                print(f'{delay_time}초 후 잡겠습니다.')
                time.sleep(delay_time)
                
                self.grip15()
                print('잡았습니다.')
                time.sleep(0.5)
                break

    def change_tool(self):
        print("self.mj.move_j to waitting point")
        self.mj.move_j(wait_poseJ, vel=VELOCITY, acc=ACC)
        time.sleep(0.5)
        self.release_tool()
        time.sleep(0.5)
        self.excep_grab()


    def time_print(self,num):
        for k in range(1,4):
            print(f'{num}번째 자세 : {k}/3 .......')
            time.sleep(1)


    def holding(self):
        print('모루로 이동하겠습니다.')
        self.mj.move_j(holding1J, vel=VELOCITY, acc=ACC)
        print('3초 주기로 위치 변경하겠습니다.')
        self.time_print(1)
        self.mj.move_j(holding2J, vel=VELOCITY, acc=ACC)
        self.time_print(2)
        self.mj.move_j(holding3J, vel=VELOCITY, acc=ACC)
        self.time_print(3)
        self.mj.move_j(holding4J, vel=VELOCITY, acc=ACC)
        self.time_print(4)
        print('망치질 완료...')
        now = get_current_posx()[0]
        now[2]+=20
        movel(now, vel=VELOCITY, acc=ACC)

    def force_ctrl(self):
        task_compliance_ctrl(stx=[2000, 2000, 100, 20, 20, 20])
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
        while(True):
            if(check_force_condition(axis=DR_AXIS_Z,max=10,ref=DR_TOOL)):
                print('밀착 성공')
                now = get_current_posx()[0]
                now[2]+=5
                time.sleep(0.1)
                release_force()
                time.sleep(0.1)
                release_compliance_ctrl()
                print('힘 해제')
                time.sleep(0.5)
                movel(now,vel=VELOCITY, acc=ACC)
                print('이동 완료')
                time.sleep(0.5)
                print('사포질 시작')
                task_compliance_ctrl(stx=[100, 100, 100, 20, 20, 20])
                time.sleep(0.1) 
                set_desired_force(fd=[0, 0, -3, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
                
                repeat = 10
                peri = 0.5
                time.sleep(0.1)
                amove_periodic(amp=[0.0, 20.00, 0.00, 0.00, 0.00, 0.00], period=[0.00, 1.00, 0.00, 0.00, 0.00, 0.00], repeat=10 ,ref=0)

                
                time.sleep(repeat*peri+5)

                release_force()
                release_compliance_ctrl()
                print('끝')
                break

    def hammering(self,force):
        time.sleep(0.5)
        task_compliance_ctrl(stx=[20000, 20000, 100, 20, 20, 20])
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])

        while(True):
            if(check_force_condition(axis=DR_AXIS_Z,max=10,ref=DR_TOOL)):
                print('밀착 성공')
                now = get_current_posx()[0]
                pos=deepcopy(now)
                pos[2]+=15
                time.sleep(0.1)
                release_force()
                time.sleep(0.1)
                release_compliance_ctrl()
                print('힘 해제')
                time.sleep(0.5)
                movel(pos,vel=VELOCITY, acc=ACC)
                print('이동 완료')
                break

        time.sleep(0.1)
        task_compliance_ctrl(stx=[2000, 2000, 100, 20, 20, 20])
        time.sleep(0.1)
        re_force = 10-force # 1~9
        peri = 0.5 + re_force*0.1
        # peri = 1
        amove_periodic(amp = [0,0,17,0,0,0],period=[0,0,peri,0,0,0], atime=0.2, repeat=10, ref=DR_TOOL)
        time.sleep(10)
        release_compliance_ctrl()
        # release_force()
        print('끝')
        # break

    def play_sapo(self):  # 모루 이동 -> 사포작업 -> z축 살짝 위로 이동
        time.sleep(0.5)
        self.mj.move_j(moru_J, vel=VELOCITY, acc=ACC)
        print('모루 도착')
        time.sleep(0.5)
        self.force_ctrl()

        time.sleep(0.5)
        now1 = get_current_posx()[0]
        # pos=copy(now)
        now1[2]+=50
        time.sleep(0.5)
        movel(now1, vel=VELOCITY, acc=ACC)

    def play_hold(self):  # 모루 이동 -> 잡기 작업 -> 2cm z축 위로
        time.sleep(0.5)
        self.holding()
        time.sleep(0.5)

    def play_hammer(self,num=6):
        time.sleep(0.5)
        self.mj.move_j(moru_J, vel=VELOCITY, acc=ACC)
        self.hammering(int(num))
        time.sleep(0.5)
        now = get_current_posx()[0]
        now[2] += 50
        time.sleep(0.2)
        movel(now, vel=VELOCITY, acc=ACC)
        time.sleep(0.2)

    def ready_tool(self):
        print("self.mj.move_j to home")
        self.mj.move_j(JReady, vel=VELOCITY, acc=ACC)

        print("self.mj.move_j to waitting point")
        self.mj.move_j(wait_poseJ, vel=VELOCITY, acc=ACC)

        self.excep_grab()

    def finsh_tool(self):
        self.mj.move_j(wait_poseJ, vel=VELOCITY, acc=ACC)
        time.sleep(0.5)
        self.release_tool()
        time.sleep(0.5)
        self.mj.move_j(JReady, vel=VELOCITY, acc=ACC)
        print('끝')

    def hammering_start(self,num=8):
        self.ready_tool()
        print('Hammering start!')#기계가 망치질
        self.play_hammer(num)
        self.finsh_tool()

    def Holding_start(self):
        self.ready_tool()
        print('Holding start!') #기계가 잡아줌
        self.play_hold()
        self.finsh_tool()
    
    def vertical_grinding_start(self):
        self.ready_tool()
        print('Vertical Grinding start!')    #갈기
        self.play_sapo()
        self.finsh_tool()

    def all_start(self):
        self.ready_tool()
        print('Hammering start!')#기계가 망치질
        self.play_hammer()
        print('Change tool!!')      
        self.change_tool()
        print('Holding start!') #기계가 잡아줌
        self.play_hold()
        print('Change tool!!')      
        self.change_tool()
        print('Vertical Grinding start!')    #갈기
        self.play_sapo()
        self.finsh_tool()
# 무게 측정
"""
    1. 무게(g)를 여러 번 측정해 평균 구함
    2. 사전에 정의된 무게와 비교해, 폐기하거나 보관
"""

import rclpy
import json
import csv
import math
import time
from datetime import datetime
import numpy as np
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
    reset_workpiece_weight,
    task_compliance_ctrl, 
    set_desired_force, 
    check_force_condition, 
    release_compliance_ctrl, 
    release_force,
    set_tool, 
    set_tcp, 
    get_current_posx, 
    wait,
    get_workpiece_weight,
    get_tool_force,
    set_digital_output,
    get_digital_input,
    ikin,
    DR_BASE, 
    DR_AXIS_Y, 
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

set_tool("TCP208mm")
set_tcp("GripperSA_rg2_250509")

##

# ────── 그리퍼 동작 ──────
def release():
    set_digital_output(2, ON)
    set_digital_output(1, OFF)

def grip():
    set_digital_output(1, OFF)
    set_digital_output(2, OFF)


class MoveControl:
    def __init__(self, adapt_coords=True):
        """
        MoveControl 클래스를 초기화하며 로봇 설정과 좌표 적응 모드를 설정합니다.
        :param adapt_coords: 좌표 적응 사용 여부 (True: 적응, False: 원본 유지)
        """
        # 좌표 적응 설정
        self.adapt_coords = adapt_coords
        self.mj=MovejStop()
        self.griper = Griper()  # 그리퍼 초기화

    def apply_compliance(self, stx=[COMPLIANCE_AXIS, COMPLIANCE_AXIS, COMPLIANCE_AXIS, COMPLIANCE, COMPLIANCE, COMPLIANCE]):
        """주어진 강성으로 컴플라이언스 제어를 적용합니다."""
        return task_compliance_ctrl(stx=stx)

    def set_force(self, fd, dir, mod=DR_FC_MOD_REL):
        """원하는 힘을 설정합니다."""
        return set_desired_force(fd=fd, dir=dir, mod=mod)

    def check_force(self, axis, max_force, ref=DR_BASE):
        """지정된 축에서 힘 조건을 확인합니다."""
        return check_force_condition(axis=axis, max=max_force, ref=ref)

    def release_compliance(self):
        """컴플라이언스 제어를 해제합니다."""
        return release_compliance_ctrl()

    def release_force(self):
        """힘 제어를 해제합니다."""
        return release_force()

    def is_position_reached(self, target_pos):
        """현재 위치가 목표 위치의 허용 오차 내에 있는지 확인합니다."""
        current_pos = self.get_current_pos()
        for i in range(3):  # x, y, z 좌표 확인
            if abs(current_pos[i] - target_pos[i]) > self.pos_tolerance:
                return False
        return True

    def apply_force_control(self, axis, direction, pos):
        """
        픽 앤 플레이스를 위한 힘 제어를 적용합니다.
        :param axis: 힘 적용 축 (DR_AXIS_X, Y, Z)
        :param direction: 힘 방향 (+1 또는 -1)
        :param pos: 목표 위치
        :return: (성공 여부, 위치) - 성공 플래그와 적응/원본 위치
        """
        print(f"go to auto grip pos: {pos}")
        time.sleep(1.5)
        movel(pos, vel=DR_VEL_L, acc=DR_ACC_L,ref=DR_BASE)
        time.sleep(0.5)
        print("compliance on")
        self.apply_compliance()
        time.sleep(0.5)
        fd = [0]*6; fd[axis] = TARGET_FORCE * direction
        dir = [0]*6; dir[axis] = 1
        time.sleep(0.5)
        print("force on")
        self.set_force(fd=fd, dir=dir)
        time.sleep(0.5)
        while True:
            if(self.check_force(axis, CHECK_FORCE)):
                print("find pos reease force and compliance")
                release_force()
                time.sleep(0.5)
                release_compliance_ctrl()
                time.sleep(0.5)
                wait(0.5)
                pos_x = get_current_posx()[0]
                print(f"now pos = {pos_x}")
                pos_change = deepcopy(pos_x)
                pos_change[axis] += 100.0*(direction*-1)
                print("move back")
                time.sleep(0.5)
                movel(pos_x, vel=DR_VEL_L, acc=DR_ACC_L,ref=DR_BASE)
                time.sleep(0.5)
                break
        return pos_x

    def auto_grip_cup(self,pos_j, start_pos, axis = DR_AXIS_Y, direction = -1, grip_type="cup"):
        """
        힘 제어를 포함한 픽 앤 플레이스 작업을 수행합니다.
        :param pos_j: 초기관절위치 (posj)
        :param start_pos: 시작 위치 (posx)
        :param axis: 힘 적용 축 (DR_AXIS_X, Y, Z)
        :param direction: 힘 방향 (+1 또는 -1)
        :param grip_type: 그리퍼 타입 ('standard' 또는 'cup')
        :return: 성공 시 True, 실패 시 False
        """
        # 초기 관절 위치로 이동
        print("pos cup station")
        self.mj.move_j(pos_j,acc=DR_ACC_J,vel=DR_VEL_J)

        # 힘 제어 적용 target_pos는 원래 자리에 되돌려 놓을걸 가정하고 미리 받아서 나중에 활용 할까 생각중인데 안써도 그대로 둘듯
        print("start auto grip pos process")
        target_pos = self.apply_force_control(axis, direction, start_pos)

        # 그리퍼 해제
        print("release grip")
        
        time.sleep(1.5)
        self.griper.release_cup()
        time.sleep(0.5)

        print("move grip pos")
        move_pos = deepcopy(target_pos)
        if(grip_type=="cup"):
            move_pos[axis] += direction*70.0
        else:
            move_pos[axis] += direction*10.0
        time.sleep(1.5)
        movel(move_pos,acc=DR_ACC_L,vel=DR_VEL_L,ref=DR_BASE)

        print("grip")
        # 그리퍼 동작
        time.sleep(1.5)
        self.griper.grip_cup()
        time.sleep(3)
        move_pos[2]+=50
        movel(move_pos,acc=DR_ACC_L,vel=DR_VEL_L,ref=DR_BASE)
        time.sleep(3)

class POS:
    def __init__(self, pos_dict=None, file_path="src/DoosanBootcamInt1/dsr_rokey/rokey/rokey/basic/data/pos/spots.json"):
        self.spots = {}  # {"name": {POS_X: [x,y,z,a,b,c], POS_J: [j1,j2,j3,j4,j5,j6]}}
        self.file_path = file_path

        # pos_dict가 제공되면 처리 및 저장
        if pos_dict is not None:
            self.add_spots_from_dict(pos_dict)
            self.save_spots()
        # pos_dict가 없으면 파일에서 로드 시도
        else:
            print("pos를 읽어오는 중입니다.")
            self.load_spots()

    def set_j(self, pos_x, solspace=4):
        """pos_x를 입력받아 pos_j(관절 좌표)를 계산"""
        try:
            j = ikin(pos_x, solspace)
            return j.tolist() if isinstance(j, np.ndarray) else j
        except Exception as e:
            print(f"ikin error: {e}")
            return None

    def add_spot(self, name, pos_x, solspace=2):
        """이름과 pos_x를 받아 pos_j를 계산하고 저장"""
        if not isinstance(pos_x, list) or len(pos_x) != 6:
            print(f"Error: pos_x {pos_x} must be a list of 6 elements [x,y,z,a,b,c]")
            return False
        
        pos_j = self.set_j(pos_x, solspace)
        if pos_j is None:
            print(f"Failed to calculate pos_j for {name}")
            return False

        self.spots[name] = {POS_X: pos_x, POS_J: pos_j}
        print(f"Added spot {name}: pos_x={pos_x}, pos_j={pos_j}")
        return True

    def add_spots_from_list(self, pos_list, name_prefix="pos"):
        """리스트 형태 [[x,y,z,a,b,c], ...]로 여러 스팟 추가"""
        for i, pos_x in enumerate(pos_list):
            name = f"{name_prefix}_{i+1}_spot"
            self.add_spot(name, pos_x)

    def add_spots_from_dict(self, pos_dict):
        """딕셔너리 형태 {name: [x,y,z,a,b,c], ...}로 여러 스팟 추가"""
        for name, pos_x in pos_dict.items():
            self.add_spot(name, pos_x)

    def get_pos_x(self, name):
        """이름으로 pos_x 조회"""
        spot = self.spots.get(name)
        if spot is None:
            print(f"Spot {name} not found")
            return None
        return spot[POS_X]

    def get_pos_j(self, name):
        """이름으로 pos_j 조회"""
        spot = self.spots.get(name)
        if spot is None:
            print(f"Spot {name} not found")
            return None
        return spot[POS_J]

    def get_spot(self, name):
        """이름으로 pos_x와 pos_j 조회"""
        spot = self.spots.get(name)
        if spot is None:
            print(f"Spot {name} not found")
            return None
        return spot[POS_X], spot[POS_J]

    def get_all_spots(self):
        """모든 스팟 정보 리턴"""
        return self.spots

    def save_spots(self):
        """spots 데이터를 JSON 파일로 저장"""
        try:
            with open(self.file_path, "w") as f:
                json.dump(self.spots, f, indent=4)
            print(f"Spots saved to {self.file_path}")
        except Exception as e:
            print(f"Error saving spots to {self.file_path}: {e}")

    def load_spots(self):
        """JSON 파일에서 spots 데이터 로드"""
        try:
            with open(self.file_path, "r") as f:
                self.spots = json.load(f)
            print(f"Spots loaded from {self.file_path}")
            print(f"spot={self.spots}")
        except FileNotFoundError:
            print(f"Error: File {self.file_path} not found")
            self.spots = {}
        except Exception as e:
            print(f"Error loading spots from {self.file_path}: {e}")
            self.spots = {}


class WeightMeasurement:
    def __init__(self, sequence_number):
        """무게 측정 클래스 초기화 with sequence number"""
        self.sequence_number = sequence_number
        self.csv_file = self._get_unique_csv_filename()
        self.mj=MovejStop()

    def _get_unique_csv_filename(self):
        """고유한 CSV 파일 이름 생성 (weight_data_YY_MM_DD_X.csv)"""
        base_path = "src/DoosanBootcamInt1/dsr_rokey/rokey/rokey/basic/data/weight"
        os.makedirs(base_path, exist_ok=True)  # 경로가 없으면 생성
        now = datetime.now()
        date_str = now.strftime("%y_%m_%d")
        base_name = f"weight_data_{date_str}"
        counter = 0
        csv_file = os.path.join(base_path, f"{base_name}_{counter}.csv")
        
        while os.path.exists(csv_file):
            counter += 1
            csv_file = os.path.join(base_path, f"{base_name}_{counter}.csv")
        
        return csv_file

    def measure_workpiece(self, samples=100):
        """get_workpiece_weight로 무게(g)를 여러 번 측정해 평균 반환"""
        time.sleep(2)
        weights = []
        for _ in range(samples):
            weight = get_workpiece_weight()
            if weight != -1:  # 성공 시
                w=deepcopy(weight)
                print(w*1000+30)
                weights.append(weight * 1000+30)
                
            wait(0.01)  # 측정 간 0.01초 대기
        return sum(weights) / len(weights) if weights else -1

    def save_to_csv(self, weight, method, target=212):
        """무게 데이터를 CSV로 저장 with sequence number"""
        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H:%M:%S")
        error = abs(weight - target) if weight != -1 else -1
        with open(self.csv_file, mode="a", newline="") as file:
            writer = csv.writer(file)
            # Write header if file is new
            if os.path.getsize(self.csv_file) == 0:
                writer.writerow(["Date", "Time", "Sequence", "Weight", "Method", "Quality", "Error"])
            writer.writerow([date_str, time_str, self.sequence_number, weight, method, error])

    # 아래 방식은 정확도를 올릴 수 있지만 외력 영향에 수정이 어려워 외력 영향에도 약간의 보정이 있는 기본 제공 을 사용하기로 결정
    def measure_tool_force_z(self, samples=5, ref=0):
        """get_tool_force로 Z축 힘(N)만 사용해 무게(g) 평균 반환"""
        weights = []
        for _ in range(samples):
            forces = get_tool_force(ref=ref)
            if forces != -1:  # 성공 시
                fz = forces[2]  # Z축 힘 (N)
                weight = fz / 9.81 * 1000  # N → g 변환
                weights.append(weight)
            wait(0.1)  # 측정 간 0.1초 대기
        return sum(weights) / len(weights) if weights else -1

    def measure_tool_force_6axis(self, samples=5, ref=0):
        """get_tool_force로 6축 데이터를 사용해 Z축 방향 힘으로 무게(g) 평균 반환"""
        weights = []
        for _ in range(samples):
            forces = get_tool_force(ref=ref)
            if forces != -1:  # 성공 시
                fx, fy, fz, _, _, _ = forces  # Fx, Fy, Fz (N), 토크는 무시
                # Z축 방향으로 합성된 힘 계산 (단순 벡터 합으로 근사)
                f_total = math.sqrt(fx**2 + fy**2 + fz**2)
                # Z축 비율로 무게 추정
                weight = (fz / f_total * f_total) / 9.81 * 1000 if f_total != 0 else fz / 9.81 * 1000
                weights.append(weight)
            wait(0.1)  # 측정 간 0.1초 대기
        return sum(weights) / len(weights) if weights else -1

class Griper:
    def __init__(self):
        pass

    def grip(self):
        self.release()
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)

    def release(self):
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        
        
    def grip_cup(self):
        self.release_cup()
        set_digital_output(1, OFF)
        set_digital_output(2, ON)

    def release_cup(self):
        set_digital_output(1, ON)
        set_digital_output(2, ON)

    # 수정해야함 제대로 이해 안되서 로직이 안됨. 빈상태로 잡았을 때를 읽어와서 에러 상태로 했으면 좋겠긴함.
    def read_input(self, index):
        return get_digital_input(index)

spots = {
    "starting_point_for_placing_defective_items": [620.880, -396.210, 75.380, 92.62, -90.27, -89.62],
    "placing_defective_items": [291.230, -534.300, 220.520, 92.47, -89.87, -91.89],
    "weight_measurement_position": [620.050, -316.670, 223.090, 91.71, -89.53, 86.24],
    "place_position_if_weight_ok": [280.960, -121.090, 133.720, 92.37, -89.73, -88.39],
    "cup_grab_position":  [623.060, -363.060, 506.030, 88.75, -91.66, -91.9],
    "first_cup_position": [625.560, -461.170, 484.090, 90.76, -91.97, -91.37],
    "second_cup_position": [614.320, -582.500, 480.630, 93.29, -71.74, -89.96]
}

class QCManager:
    def __init__(self):
        self.ps = POS()
        self.gr = Griper()
        self.wm = WeightMeasurement(sequence_number=1)
        self.mc = MoveControl()
        self.mj=MovejStop()
        self.status = "idle"
        self.current_qc_step = 0
        self.paused = False
        self.operation_in_progress = False

    def movement_item_measure_loc(self, posj, posx):
        print("start cup auto grip...")
        self.mc.auto_grip_cup(posj, posx)
        print("move up pos grip to pos start")
        time.sleep(0.5)
        pos = self.ps.get_pos_x("cup_grab_position")
        pos[2]+=50
        time.sleep(0.5)
        movel(pos, vel=DR_VEL_L, acc=DR_ACC_L)
        time.sleep(0.5)
        print("move measure position")
        self.mj.move_j(self.ps.get_pos_j("weight_measurement_position"), vel=DR_VEL_J, acc=DR_ACC_J)
        time.sleep(0.5)

    def movement_item_loc(self, thrown):
        if thrown==False:
            print("move start thrown pos")
            self.mj.move_j(self.ps.get_pos_j("starting_point_for_placing_defective_items"), vel=DR_VEL_J, acc=DR_ACC_J)
            time.sleep(0.5)
            print("move thrown pos")
            self.mj.move_j(self.ps.get_pos_j("placing_defective_items"), vel=DR_VEL_J, acc=DR_ACC_J)
            time.sleep(1.5)
            self.gr.release_cup()
            time.sleep(0.5)
            print("move start thrown pos")
            self.mj.move_j(self.ps.get_pos_j("starting_point_for_placing_defective_items"), vel=DR_VEL_J, acc=DR_ACC_J)
            time.sleep(0.5)
        else:
            print("move store pos")
            self.mj.move_j(self.ps.get_pos_j("place_position_if_weight_ok"), vel=DR_VEL_J, acc=DR_ACC_J)
            time.sleep(0.5)
            pos=deepcopy(self.ps.get_pos_x("place_position_if_weight_ok"))
            pos[2]-=20
            time.sleep(0.5)
            movel(pos,vel=DR_VEL_L,acc=DR_ACC_L)
            time.sleep(1.5)
            self.gr.release_cup()
            time.sleep(0.5)
            self.mj.move_j(self.ps.get_pos_j("place_position_if_weight_ok"), vel=DR_VEL_J, acc=DR_ACC_J)
            time.sleep(0.5)
            

    def weight_classification(self):
        time.sleep(5)
        # reset_workpiece_weight()
        weight1 = self.wm.measure_workpiece()
        time.sleep(1.5)
        if weight1 != -1:
            # 무게 보정
            print(f"Workpiece Weight: {weight1:.2f}g")
            time.sleep(1.5)
            self.wm.save_to_csv(weight1, "workpiece")
            time.sleep(1.5)
        else:
            print("Workpiece Weight: 측정 실패")
        return 201 <= weight1 <= 221

    def weiight_classif_auto_movement(self):
        print("set_init")
        self.gr.release_cup()
        time.sleep(1.5)
        self.mj.move_j([0,0,90,0,90,0],vel=DR_VEL_J,acc=DR_ACC_J)
        time.sleep(1.5)
        self.gr.grip()
        time.sleep(1.5)
        flag = [False, False]
        self.init_pos_measure()
        time.sleep(1.5)
        self.movement_item_measure_loc(self.ps.get_pos_j("cup_grab_position"), self.ps.get_pos_x("first_cup_position"))
        flag[0] = self.weight_classification()
        self.movement_item_loc(flag[0])
        time.sleep(1.5)
        self.gr.grip()
        time.sleep(3)
        self.movement_item_measure_loc(self.ps.get_pos_j("cup_grab_position"), self.ps.get_pos_x("second_cup_position"))
        time.sleep(1.5)
        flag[1] = self.weight_classification()
        time.sleep(1.5)
        self.movement_item_loc(flag[1])
        self.operation_in_progress = False
        self.status = "idle"
        return "QC completed"
    
    def init_pos_measure(self):
        self.gr.release_cup()
        time.sleep(3)
        self.mj.move_j(self.ps.get_pos_j("weight_measurement_position"),vel=DR_VEL_J,acc=DR_ACC_J)
        time.sleep(3)
        reset_workpiece_weight()
        time.sleep(4)
        self.gr.grip()
        time.sleep(3)

    def monitor_robot(self):
        """로봇팔 상태 모니터링 (예: 외력 감지)"""
        while True:
            try:
                force = get_tool_force(DR_BASE)  # 외력 측정
                if any(abs(f) > CHECK_FORCE for f in force[:3]) and not self.paused:
                    self.handle_command("PAUSE")
                    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 외력 감지: 로봇 일시 중지")
                print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 모니터링: 상태={self.status}, QC 단계={self.current_qc_step}")
                time.sleep(1)
            except Exception as e:
                print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 모니터링 에러: {e}")
                time.sleep(1)
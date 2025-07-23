# 비동기 movej로 이동 중에 충격 감지 시, 정지 처리
import rclpy
import time
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from dsr_common2.imp import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init(args=None)
node = rclpy.create_node("qc_management", namespace=ROBOT_ID)
DR_init.__dsr__node = node


from dsr_common2.imp.DSR_ROBOT2 import (
    amovej, 
    task_compliance_ctrl, 
    release_compliance_ctrl, 
    set_tool, 
    set_tcp, 
    get_tool_force,
    check_motion,
)
set_tool("TCP208mm")
set_tcp("GripperSA_rg2_250509")

FORCE_THRESHOLD = 20.0 

class MovejStop:
    def __init__(self):
        pass

    def move_j(self,pos,vel=10,acc=10):
        while True:
            print("go_amove")
            # 목표 지점까지 비동기 movej로 이동
            amovej(pos, vel=vel, acc=acc)
            time.sleep(1.0)

            while True:
                # Tool에 작용하는 외력을 구함
                fx, fy, fz, tx, ty, tz = get_tool_force() 
                total_force = (fx**2 + fy**2 + fz**2)**0.5

                if total_force > FORCE_THRESHOLD:
                    print(f"외력 감지됨: {total_force:.2f} N → 모션 정지")
                    #MoveStop

                    # 순응 제어 활성화 (XYZ 방향만 허용)
                    task_compliance_ctrl([1, 1, 1, 0, 0, 0])
                    time.sleep(0.5)
                    release_compliance_ctrl()
                    time.sleep(0.5)
                    break
                # 모션이 완료 된 경우
                if check_motion()==0:   
                    break

            # 모션이 완료된 경우
            if check_motion()==0:   
                print("최종 목표 위치 도달")
                break
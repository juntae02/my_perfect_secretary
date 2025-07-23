# tcp 톻신
"""
    휴대폰을 통해 수신한 message 실행
    [main]
    ├── do_all                 # 전체 자동 실행 (casting → grinding → spinning → forging → QC)
    ├── fd                     # 주조 모드 진입
    │   ├── casting:X          # X회 반복 (예: casting:3)
    │   └── exit               # 메인 상태로 복귀
    ├── fg                     # 단조 모드 진입
    │   ├── all                # 단조 전체 실행 (forging -> grinding -> spinning)
    │   ├── hd                 # 잡기 (Holding)
    │   ├── hm[:X]             # 해머질 (Hammering), 선택적으로 반복 횟수 입력
    │   ├── vg                 # 수직 그라인딩 (Vertical Grinding)
    │   ├── hg[:X]             # 수평 그라인딩 (Horizontal Grinding), 반복 횟수 입력 가능
    │   ├── tw[:X]             # 비틀기 (Tweaking), 반복 횟수 입력 가능
    │   └── exit               # 메인 상태로 복귀
    ├── qc                     # 품질검사 모드 진입
    │   ├── auto               # 자동 분류 시작 (무게 측정 포함)
    │   ├── wc                 # 무게 측정만 수행
    │   └── exit               # 메인 상태로 복귀
    └── exit                   # 클라이언트 연결 종료  
"""

import socket
import threading
import subprocess
from datetime import datetime
from io import StringIO

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from blacksmith_robot.casting import Casting
from blacksmith_robot.forging import GrindingTask, SpitRotationTask, Forge
from blacksmith_robot.qc_manager import QCManager
from blacksmith_robot.stop_motion import MovejStop

# stdout을 TCP 클라이언트로 보내는 클래스
class ClientPrintRedirector:
    def __init__(self):
        self.clients = []  # 연결된 클라이언트 소켓 리스트
        self.buffer = StringIO()  # 출력 버퍼

    def add_client(self, client_socket):
        self.clients.append(client_socket)
        self.write(f"ClientPrintRedirector: Added client {client_socket.getpeername()}")

    def remove_client(self, client_socket):
        if client_socket in self.clients:
            self.clients.remove(client_socket)
            self.write(f"ClientPrintRedirector: Removed client {client_socket.getpeername()}")

    def write(self, message):
        if message.strip():  # 빈 문자열 제외
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            formatted_message = f"[LOG] {timestamp} - {message.strip()}\n"
            for client in self.clients[:]:  # 복사본으로 순회
                try:
                    client.send(formatted_message.encode('utf-8'))
                except:
                    self.clients.remove(client)  # 연결 끊긴 클라이언트 제거
        self.buffer.write(message)

    def flush(self):
        pass

# stdout을 ClientPrintRedirector로 리다이렉트
client_print_redirector = ClientPrintRedirector()
sys.stdout = client_print_redirector


class TCPServer:
    def __init__(self, host='0.0.0.0', port=1234):
        self.sp = SpitRotationTask()
        self.qc = QCManager()
        self.ct = Casting()
        self.fg = Forge()
        self.gt = GrindingTask()
        self.mj= MovejStop()
        self.host = host
        self.port = port
        self.server_socket = None
        self.running = False
        self.status = "all"
        self.current_qc_step = 0  # QC 단계 추적
        self.operation_in_progress = False  # 작업 진행 중 여부
        self.command_lock = threading.Lock()
        self.repeat_count = 1
        self.waiting_for_repeat_count = False  # 반복 횟수 입력 대기 상태 추가

    def start(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.running = True
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 서버가 {self.host}:{self.port}에서 시작됨")
        except Exception as e:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 서버 시작 에러: {e}")
            return

        try:
            while self.running:
                client_socket, addr = self.server_socket.accept()
                client_print_redirector.add_client(client_socket)
                print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 클라이언트 {addr} 연결됨")
                client_thread = threading.Thread(target=self.handle_client, args=(client_socket, addr))
                client_thread.start()
        except KeyboardInterrupt:
            self.stop()
        except Exception as e:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 서버 에러: {e}")
        finally:
            self.stop()

    def parse_message(self,message):
        # 콜론(:)이 있는지 확인
        if ':' in message:
            # 콜론으로 분리
            text_part, num_part = message.split(':', 1)
            
            # 숫자 부분을 int로 변환
            number = int(num_part)
            return text_part,number
        
        else:
            # 콜론이 없는 경우: 문자열만 처리
            number = None
            return message,number
        
    def handle_client(self, client_socket, addr):
        try:
            client_socket.send("Enter command (do_all, fd, fg, qc, exit): ".encode('utf-8'))
            while self.running:
                data = client_socket.recv(1024).decode('utf-8').strip()
                if not data:
                    break
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                print(f"[{timestamp}] 클라이언트 {addr} → 메시지: {data}")
                if(self.status=="all"):
                    response = self.handle_message(data,client_socket)
                elif(self.status=="fd"):
                    response = self.foundry_message(data,client_socket)
                elif(self.status=="fg"):
                    response = self.forge_message(data,client_socket)
                elif(self.status=="qc"):
                    
                    response = self.qc_message(data,client_socket)
        except Exception as e:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            print(f"[{timestamp}] 클라이언트 {addr} 에러: {e}")
        finally:
            client_socket.close()
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            print(f"[{timestamp}] 클라이언트 {addr} 연결 종료")

    def foundry_message(self,message,client_socket):
        with self.command_lock:
            if self.operation_in_progress:
                    return "Error: 이미 동작 중입니다."
            msg,num=self.parse_message(message)
            if msg=="casting":
                self.operation_in_progress = True
                self.waiting_for_repeat_count = True  # 반복 횟수 입력 대기 상태
            if self.waiting_for_repeat_count:
                try:
                    count = int(num)
                    if count < 1:
                        return "Error: 반복 횟수는 1 이상의 숫자여야 합니다."
                    self.repeat_count = count
                    self.waiting_for_repeat_count = False  # 대기 상태 해제
                    # 실제 작업은 여기서 수행하거나 따로 사용자가 작업하면 됨
                    self.ct.run(repeat_m=self.repeat_count)
                    self.operation_in_progress = False
                    return f"반복 횟수 설정 완료: {self.repeat_count}"
                except ValueError:
                    return "Error: 올바른 숫자를 입력해주세요."
            elif msg=="exit":
                self.status="all"
                client_socket.send("Enter command (do_all, fd, fg, qc, exit): ".encode('utf-8'))


    def forge_message(self,message,client_socket):
        with self.command_lock:
            if self.operation_in_progress:
                    return "Error: 이미 동작 중입니다."
            # if message=="hm":
            #     pass
            msg,num=self.parse_message(message)
            print(msg)
            print(num)
            if msg=="all":
                print("all forge")
                self.operation_in_progress = True
                self.fg.all_start()
                self.gt.grinding_run()
                self.sp.spin_run()
                self.operation_sin_progress = False
                return 
            
            elif msg=="hd":
                print("holding")
                self.operation_in_progress = True
                self.fg.Holding_start()
                self.operation_in_progress = False
                return 
            
            elif msg=="hm":
                print("hammering")
                self.operation_in_progress = True
                if(num==None):
                    self.fg.hammering_start()
                else:
                    self.fg.hammering_start(num)
                self.operation_in_progress = False
                return 
            
            elif msg=="vg":
                print("vertical_grinding")
                self.operation_in_progress = True
                self.fg.vertical_grinding_start()
                self.operation_in_progress = False
                return 
            
            elif msg=="hg":
                print("horizontal_grinding")
                self.operation_in_progress = True
                if(num==None):
                    self.gt.grinding_run()
                else:
                    self.gt.grinding_run(num)
                self.operation_in_progress = False
                return 
            
            elif msg=="tw":
                print("tweaking")
                self.operation_in_progress = True
                if(num==None):
                    self.sp.spin_run()
                else:
                    self.sp.spin_run(num)
                self.operation_in_progress = False
                return 
                
            elif msg=="exit":
                self.status="all"
                client_socket.send("Enter command (do_all, fd, fg, qc, exit): ".encode('utf-8'))    ### 단조 과정은 잡기, 해머질, 수평 그라인딩, 수직 그라인딩, 비틀기

    def qc_message(self,message,client_socket):
        with self.command_lock:
            # 명령어 처리
            if self.operation_in_progress:
                    return "Error: 이미 동작 중입니다."
            if message=="auto":
                print("QC_START")
                self.operation_in_progress = True
                self.qc.weiight_classif_auto_movement()
                self.status = "running_qc"
                self.operation_in_progress = False
                return 
            
            elif message=="wc":
                print("masure")
                self.operation_in_progress = True
                self.qc.weight_classification()
                self.operation_in_progress = False
                return 
            
            
            elif message=="exit":
                self.status="all"
                client_socket.send("Enter command (do_all, fd, fg, qc, tw, exit): ".encode('utf-8'))

    def handle_message(self, message,client_socket):
        with self.command_lock:
            # 명령어 처리
            if self.operation_in_progress:
                    return "Error: 이미 동작 중입니다."
            if message == "do_all":
                self.operation_in_progress = True
                self.status = "running_all"
                self.ct.run()
                self.gt.grinding_run()
                self.sp.spin_run()
                self.fg.all_start()
                self.qc.weiight_classif_auto_movement()
                client_socket.send("Enter command (do_all, fd, fg, qc, exit): ".encode('utf-8'))
                return
                        # casting 명령이 들어오면 반복 횟수 입력 대기 상태로 전환
            elif message == "fd":#foundry
                self.status="fd"
                client_socket.send("Enter command (casting, exit): ".encode('utf-8'))
            elif message == "fg":#forge
                self.status="fg"
                client_socket.send("Enter command (all forge[all], holding[hd], hammering[hm], vertical_grinding[vg], horizontal_grinding[hg], tweaking[tw], exit): ".encode('utf-8'))
            elif message == "qc":
                self.status="qc"
                client_socket.send("Enter command (auto, wc, exit): ".encode('utf-8'))
            elif message=='exit':
                client_socket.close()
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                print(f"[{timestamp}] 클라이언트  연결 종료")

            else:
                return f"Unknown command: {message}"
                        # 반복 횟수 입력 대기 상태에서 숫자 입력 받음
            
            

    def forward_command(self, command):
        """메인 프로그램(robot_control.py)에 명령어 전달"""
        try:
            result = subprocess.run(
                ["python3", "robot_control.py", command],
                capture_output=True,
                text=True,
                check=True
            )
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Forwarded command: {command}, Response: {result.stdout.strip()}")
            return result.stdout.strip() or "Command executed"
        except subprocess.CalledProcessError as e:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Command error: {e}")
            return f"Error: {e.stderr.strip()}"
        except Exception as e:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Forward error: {e}")
            return f"Error: {e}"

    def handle_qc_step(self, step):
        steps = {
            "0": "QC_HOME",
            "1": "QC_MOVE_TO_PICK",
            "2": "QC_PICK",
            "3": "QC_MOVE_TO_WEIGH",
            "4": "QC_WEIGH",
            "5-1": "QC_MOVE_TO_DISCARD",
            "5-2": "QC_MOVE_TO_STORE"
        }
        if step in steps:
            if self.operation_in_progress and step != self.current_qc_step:
                return "Error: Operation in progress, please wait or send PAUSE/STOP"
            self.current_qc_step = step
            self.operation_in_progress = True
            response = self.forward_command(steps[step])
            return response
        return f"Invalid QC step: {step}"

    def send_command(self, command):
        """서버 내부에서 명령어 전송 (테스트용)"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect(('192.168.0.4', self.port))
                s.send(command.encode('utf-8'))
                response = s.recv(1024).decode('utf-8')
                print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Sent command: {command}, Response: {response}")
                return response
        except Exception as e:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Command send error: {e}")
            return f"Error: {e}"

    def stop(self):
        self.running = False
        if self.server_socket:
            self.server_socket.close()
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 서버 종료")
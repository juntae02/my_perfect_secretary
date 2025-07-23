# project name : 나의 완벽한 비서 (장인 보조 협동 로봇 - 대장간 프로젝트)
# date : 2025-06-05
# author : 두산 로벨스
'''
TCPServer 클래스에서 모든 동작들을 수행
init에 선언된 클래스들을 확인하면, 어떤 동작을 수행하는지 시퀀스를 이해할 수 있음
    def __init__(self, host='0.0.0.0', port=1234):
        self.ct = Casting()           - 주조
        self.gt = GrindingTask()      - 수평 연마(단조)
        self.sp = SpitRotationTask()  - 비틀기(단조)
        self.fg = Forge()             - 수직 연마, 해머, 잡기(단조)
        self.qc = QCManager()         - 무게 측정
        self.mj = Move_Js()           - movej 움직임 예외처리

print문은 자동으로 휴대폰 tcp 연결된 곳으로 전송되며, 한글 지원 안함 
-> 폰으로 확인하면 에러 상황 시, 휴대폰 연결이 끊기며 정지함
'''

from tcp.tcp_server import TCPServer

def main():
    # 서버 실행
    server = TCPServer(host='0.0.0.0', port=1234)
    server.start()

if __name__ == "__main__":
    main()

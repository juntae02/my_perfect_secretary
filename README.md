## 🦾 나의 완벽한 비서
- **장인 보조 협동로봇**, Part 1 - 대장장이 :  
  &nbsp;&nbsp;주조 및 단조 공정의 단순 반복 작업을 협동로봇이 수행하여, 장인들의 고유 기술을 지키는 프로젝트입니다.
- 개발기간 : 2025.05.23-06.05(2주)  
- 개발인원 : 4명(팀장)  
<br />

## 🔍 문제 정의
> &nbsp;&nbsp;장인들의 ***고령화***와 젊은 세대의 ***높은 진입 장벽***으로 인해, 고유 기술이 사라진다는 문제의식에서 출발했습니다.  
저희는 ***협동로봇의 도입***을 통해, 단순하고 반복적인 공정은 자동화 및 보조 기능으로 ***장인의 신체적 부담을 줄이는 것을 목표***로 했습니다.   
또한, 공정의 자동화 및 사용자 친화적인 제어(휴대폰 TCP 통신)로 기술 진입 장벽을 낮춰, ***고유 기술의 계승을 돕는 것을 목표***로 했습니다.  
<br />

## 📌 주요 기능
- ***주조 공정 자동화*** : 용탕 주입, 용탕 균일화 등 주조 과정에서의 반복적이고 위험한 작업을 협동로봇이 수행합니다.
- ***단조 공정 지원*** : 수평/수직 연마, 재련, 비틀기 등 단조 과정의 특정 작업을 협동로봇이 보조합니다.
- ***안전 기능 구현*** : 충돌 감지 및 용탕 튐 방지와 같은 위험 요소를 사전에 방지하는 기능을 수행합니다.
- ***QC 무게 측정*** : 생산된 제품의 무게를 측정하여 품질을 관리하는 기능을 수행합니다.
- ***TCP 통신 기반 제어*** : 휴대폰과의 TCP 통신을 통해 로봇을 제어하고, 공정 상태를 모니터링 합니다.
<br />

## 🎥 시연 영상
- 영상(본인) : [Demo_Me](https://www.youtube.com/watch?v=wulUciU5lNg)  
- 영상(전체) : [Demo_All](https://www.youtube.com/watch?v=4p3I4KdZMHU)  
👉 클릭해서 시연 영상 보기
- 발표 자료 : [Presentation](https://www.canva.com/design/DAGt2pGk8OM/XAmI-RrP8dZGNZCpIRM1vw/edit?utm_content=DAGt2pGk8OM&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)  
⚠️ 언어는 영어로 설정해야 폰트가 변형되지 않습니다.  
<br />

## 🛠️ 기술 스택
- **하드웨어**: Doosan Robotics M-series M0609
- **개발 언어**: Python 
- **백엔드**: rclpy
- **통신**: TCP CLIENT APP
- **협업 툴**: Notion, Draw.io
<br />

## 👨‍💻 담당한 기능
&nbsp;&nbsp;"충돌" 및 "용탕 튐"과 같은 **위험 요소를 방지**하는 방향으로, ***주조 공정 자동화*** 설계 

- **amovej 이동 시, 충돌 방지를 위한 정지 기능** :  
  > &nbsp;&nbsp;비동기 movej 이동 중, **힘 센서를 통해 로봇암에 가해지는 외력**을 실시간으로 감지하고, **임계값을 초과할 경우에 즉시 정지**하도록 하는 안전 로직을 구현했습니다.  
  > 외력이 감지되면, 로봇은 일시적으로 **XYZ방향(위치 유연 & 자세 고정)의 순응 제어를 활성화**하여 충돌을 흡수하고, 작업자나 주변 물체에 가해지는 물리적 부담을 완화합니다.  
  > 이후, **힘이 임계값 이하로 떨어지면 순응 제어를 해제**하고, 이전 목표 지점으로 **이동을 재시도**하여 작업을 이어가도록 구성했습니다.  

  👉 [충돌 방지 기능](https://github.com/juntae02/my_perfect_secretary/blob/main/blacksmith_robot/stop_motion.py#L35-L68)
  
- **래들 감지를 위한, Compliance Control 및 Force Control 적용** :  
  > &nbsp;&nbsp;용탕을 담는 레들의 위치를 정밀하게 파악할 수 있도록 **task_compliance_ctrl() 및 set_desired_force()를** 적용했습니다.  
  > 로봇이 Y축 방향으로 부드럽게 접근하면서, **check_force_condition()을** 통해 외력 변화로 레들의 존재를 감지합니다.  
  > 또한, **get_current_posx()[0]를** 통해서 감지된 좌표를 받고, 해당 위치를 바탕으로 레들을 수거하는 작업을 수행합니다.

  👉 [래들 감지 기능](https://github.com/juntae02/my_perfect_secretary/blob/main/blacksmith_robot/casting.py#L115-L145)

- **안정적인 용탕 이송을 위한, movesx 기능** :  
  > &nbsp;&nbsp;용탕 이송은 넘치는 현상을 방지하기 위해, Joint각도 움직이는 movej는 지양했습니다.  
  > 또한, 용탕의 온도 변화는 민감한 부분이기에, 직선으로 움직이는 movel이 아닌 movesx를 사용하여 곡선으로 이동했습니다.  
  > 이때, z축이 움직이는 경우 용탕이 넘칠 수 있기 때문에, z축은 고정하여 이송하도록 구현했습니다.  

  👉 [용탕 이송 기능](https://github.com/juntae02/my_perfect_secretary/blob/main/blacksmith_robot/casting.py#L147-L169)
  
- **용탕 균일화를 위한, move_periodic 기능** :  
  > &nbsp;&nbsp;용탕 내 온도 분포 및 불순물 침전을 방지하기 위해, move_periodic() 함수를 사용하여, 주기적 진동 기반 교반 동작을 구현했습니다.  
  > 진폭과 주기를 X-Y축에 각각 적용하여, 레들 내부의 용탕을 일정 패턴으로 흔들며 열 균일화 및 품질 안정화를 유도합니다.  
  > 이때, z축으로 같이 회전을 하면 용탕이 넘칠 가능성이 높아져 z축으로는 적용하지 않았습니다.  
  > 이는 주입 전 품질 제어 측면에서 제품 일관성과 신뢰성 확보에 기여합니다.

  👉 [용탕 균일화 기능](https://github.com/juntae02/my_perfect_secretary/blob/main/blacksmith_robot/casting.py#L171-L182)

- **안정적인 용탕 주입을 위한, movel 명령 시퀀스 기능** :  
  > &nbsp;&nbsp;용탕 주입은 튐 현상을 방지하기 위해, 두 단계의 movel() 직선 궤적으로 작업을 수행합니다.  
  > 실제로 물을 따를 때를 모티브하여, 먼저 천천히 로봇의 회전 각도를 변경하여 용탕의 흐름을 유도합니다.  
  > 이후에 보다 깊은 각도로 주입을 완료하여, 잔여 용탕까지 모두 투입하도록 구현했습니다.  
  > 이는 고온 액체를 다루는 정밀 주조 환경에서 안정성을 극대화하는 전략적 제어 방식입니다.

  👉 [용탕 주입 기능](https://github.com/juntae02/my_perfect_secretary/blob/main/blacksmith_robot/casting.py#L184-L198)
<br />

## 🤔 트러블슈팅 및 해결 과정 
- 문제 상황: 화분에 꽃이 심어져 있을 때, 꽃은 인식하지 못하고 화분만 인식하는 문제 발생
> - 시도했던 방법:
> - 해결 과정:
> - 결과
- 문제 상황: 멀티 쓰레드
- 문제 상황: 씨앗 pick 실패의 경우
<br />

## 💡 과정 속에서 배운 점 및 향후 계획
- 배운 점:
> - d:
- 향후 계획:
> - 긴급 정지 기능:
<br />

## 🤝 팀원 정보
- ***준태*** : 주조 공정 및 총괄 지휘(본인)   
- 정하 : 단조 공정의 수평연마 및 재련   
- 요셉 : 단조 공정의 수직연마 및 비틀기 
- 민수 : QC 무게 측정, TCP 통신 
<br />

## 📚 참고 및 출처
- 본 프로젝트의 일부 코드는 아래 공개 저장소에서 **수정 없이 그대로 사용**되었습니다:  
  > 🔗 https://github.com/ROKEY-SPARK/DoosanBootcamInt1

- 해당 저장소의 코드 중 `DSR_ROBOT2.py`, `DR_init.py` 등 일부 파일은  
  **로봇 제어 및 시스템 구성에 필요한 기능을 위해 포함하여 사용**되었습니다.
  
- 이 외에도 `DoosanBootcamInt1` 저장소 내의 일부 파일이 **직접 또는 간접적으로 참조되어 사용되었을 수 있습니다**.

- 원본 저장소에는 [LICENSE](https://github.com/ROKEY-SPARK/DoosanBootcamInt1/blob/main/LICENSE)가 포함되어 있으며,  본 프로젝트 역시 해당 라이선스를 준수하여 사용하고 있습니다.

> 본 저장소는 교육 및 학습 목적으로만 사용됩니다.

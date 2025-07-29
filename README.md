## 🦾 나의 완벽한 비서
- 장인 보조 협동로봇, Part 1 - 대장장이 :  
  &nbsp;&nbsp;**주조 및 단조 공정의 단순 반복 작업을 협동로봇이 수행**하여, 장인의 신체적 부담을 줄이고 기술 계승을 돕는 프로젝트입니다.
- 개발기간 : 2025.05.23-06.05(2주)  
- 개발인원 : 4명(팀장)  
<br />

## 🔍 문제 정의
> &nbsp;&nbsp;장인들의 ***고령화***와 ***높은 진입 장벽***으로 인해, 고유 기술이 사라진다는 문제의식에서 출발했습니다.  
***주조 및 단조*** 과정에서의 단순하고 반복적인 공정은 협동로봇이 수행하여, 장인의 ***신체적 부담***을 줄임
자동화 및 휴대폰 ***TCP 통신***을 통하여 ***기술 진입 장벽***을 낮춤으로써, 고유 기술 계승의 흐름을 회복  
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
- **하드웨어 제어**: Doosan Robot Arm
- **개발 언어**: Python 
- **프론트엔드**: Kivy
- **백엔드**: Flask, rclpy
- **AI / 데이터 처리**: YOLO, OpenAI API
- **협업 툴**: GitHub, Notion, Draw.io
<br />

## 👨‍💻 담당한 기능
- ***YOLO 모델 생성 및 학습*** :  
  &nbsp;&nbsp;프로젝트에 **최적화된 객체 탐지 모델을 구축하기 위해**, YOLO 모델을 생성 및 학습시켰습니다.  
  이를 통해, 특정 꽃의 종류나 씨앗을 탐지하는 데 기여했습니다.  
  👉 [모델 생성 및 학습 과정](https://github.com/juntae02/bloom_for_you/tree/main/yolo_models)
  
- ***상황별 꽃 추천 기능*** :  
  &nbsp;&nbsp;OpenAI API 기반 키워드 추출 함수(팀원)를 활용하여, 사용자의 **목적과 상황을 반영하는 프롬프트**를 설계했습니다.  
  추출된 키워드를 **JSON 파일과 매칭**하여, 적절한 꽃 정보를 자동으로 불러오는 **추천** 로직을 구성했습니다.  
  해당 꽃 정보는 **Kivy 기반의 GUI**에서 사용자가 결과를 확인하고, **"선택" 또는 "재선택"을** 요청할 수 있는 인터페이스를 구현했습니다.  
  👉 [꽃 추천 기능(flower_recommender.py)](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/bloom_for_you/flower_recommender.py#L205-L250)  
  👉 [GUI(Kivy) 기능(flower_recommender.py)](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/bloom_for_you/flower_recommender.py#L95-L203)  
  👉 [프롬프트 파일](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/resource/prompt/recommender_prompt.txt)  
  👉 [JSON 파일](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/resource/flower_recommendations.json)
  
- ***자동화된 씨앗 심기 기능*** :  
  &nbsp;&nbsp;**ROS2 토픽 통신**을 통해 전달받을 꽃 정보를 해석하여, **로봇의 동작 흐름**을 설계했습니다.  
  해당 꽃 정보를 바탕으로, YOLO를 통해 씨앗 및 화분의 위치를 탐지하고, **"씨앗 집기 -> 운반 -> 이식 -> 재배 구역 이동"** 로직을 구현했습니다.  
  **씨앗 미탐지 예외 처리** 및 **순응 제어 기반의 정밀 배치 기능** 로직도 구성하여, 안정적인 동작을 보장했습니다.  
  👉 [씨앗 Pick&Place(seed_planting.py)](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/bloom_for_you/seed_planting.py#L76-L156)  
  👉 [화분 Compliance Control(seed_planting.py)](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/bloom_for_you/seed_planting.py#L158-L184)
<br />

**[담당 역할]**
> 충돌 및 용탕 튐과 같은 위험 요소를 방지하는 방향으로, 주조 공정을 구현
>> - ***movej***로 이동 시, 힘이 감지되면 정지 기능  
>> - ***compliance control*** 및 ***force control***를 사용하여, 레들 감지 기능  
>> - ***move_periodic***를 사용하여, 용탕 균일화 기능  
>> - 회전값만 조절한 두 번의 ***movel*** 명령으로, 용탕 주입 기능  
>> - [프로젝트 상세 설명](https://github.com/juntae02/my_perfect_secretary)  
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
> - d:
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

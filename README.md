# my_perfect_secretary
## 3. 🦾 나의 완벽한 비서
> 장인 보조 협동로봇 - Part 1: 대장장이  
> - 개발기간 : 2025.05.23-06.05(2주)  
> - 개발인원 : 4명(팀장)  

**[프로젝트 개요]**
> 장인들의 ***고령화***와 ***높은 진입 장벽***으로 인해, 고유 기술이 사라진다는 문제의식에서 출발  
> ***주조 및 단조*** 과정에서의 단순하고 반복적인 공정은 협동로봇이 수행하여, 장인의 ***신체적 부담***을 줄임  
> 자동화 및 휴대폰 ***TCP 통신***을 통하여 ***기술 진입 장벽***을 낮춤으로써, 고유 기술 계승의 흐름을 회복  
>> - ***준태***: 주조 공정 및 총괄 지휘  
>> - 정하: 단조 공정의 수평연마 및 재련  
>> - 요셉: 단조 공정의 수직연마 및 비틀기  
>> - 민수: QC 무게 측정, TCP 통신  

**[담당 역할]**
> 충돌 및 용탕 튐과 같은 위험 요소를 방지하는 방향으로, 주조 공정을 구현
>> - ***movej***로 이동 시, 힘이 감지되면 정지 기능  
>> - ***compliance control*** 및 ***force control***를 사용하여, 레들 감지 기능  
>> - ***move_periodic***를 사용하여, 용탕 균일화 기능  
>> - 회전값만 조절한 두 번의 ***movel*** 명령으로, 용탕 주입 기능  
>> - [프로젝트 상세 설명](https://github.com/juntae02/my_perfect_secretary)  
<br />

- 발표 자료  
> 언어는 영어로 설정해야 폰트가 변형되지 않음  
>> - [🦾 나의 완벽한 비서](https://www.canva.com/design/DAGt2pGk8OM/XAmI-RrP8dZGNZCpIRM1vw/edit?utm_content=DAGt2pGk8OM&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)

## 📚 참고 및 출처
- 본 프로젝트의 일부 코드는 아래 공개 저장소에서 **수정 없이 그대로 사용**되었습니다:  
  > 🔗 https://github.com/ROKEY-SPARK/DoosanBootcamInt1
- 해당 저장소의 코드 중 `DSR_ROBOT2.py`, `DR_init.py` 등 일부 파일은, **로봇 제어 및 시스템 구성에 필요한 기능을 위해 직접 포함하여 사용**되었습니다.
- 이 외에도 `DoosanBootcamInt1` 저장소 내의 일부 파일이 **직접 또는 간접적으로 참조되어 사용되었을 수 있습니다**.

- 원본 저장소에는 [LICENSE](https://github.com/ROKEY-SPARK/DoosanBootcamInt1/blob/main/LICENSE)가 포함되어 있으며, 본 저장소는 해당 라이선스를 존중하며 **교육 및 학습 목적으로만 사용**됩니다.

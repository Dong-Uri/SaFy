

# 🚗SaFy

###  사고 및 돌발 상황에 대해 성능이 탁월한 자율주행 시스템
###  SSAFY 8기 특화 프로젝트
<br>

## 👨‍👩‍👧‍👦팀원 소개


**[장재현]** : 팀장 / 인지 / 라이다 센서 코드 구현


**[김지선]** : 인지 / 카메라 센서 코드 구현


**[채민기]** : 부팀장 / 판단 / 상황별 알고리즘 구현


**[이동우]** : 판단 / 인지,판단,제어 통합


**[박균탁]** : 발표 / 제어 / YOLO 알고리즘 구현


**[임상빈]** : 제어 / 데이터 송수신 구현



<br>

## 📆 프로젝트 소개

**⚙​ 개발 환경**

**자율주행**

<img src="https://img.shields.io/badge/ROS-22314E?style=for-the-badge&logo=ROS&logoColor=white"> <img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=Python&logoColor=white"> <img src="https://img.shields.io/badge/PyTorch-EE4C2C?style=for-the-badge&logo=PyTorch&logoColor=white"> <img src="https://img.shields.io/badge/Morai-000000?style=for-the-badge&logo=Morai&logoColor=white">
 
**협업**

<img src="https://img.shields.io/badge/gitlab-FC6D26?style=for-the-badge&logo=GitLab&logoColor=white"> <img src="https://img.shields.io/badge/jira-0052CC?style=for-the-badge&logo=Jira&logoColor=white"> <img src="https://img.shields.io/badge/MatterMOST-009688?style=for-the-badge&logo=Mattermost&logoColor=white"> <img src="https://img.shields.io/badge/Notion-EF1970?style=for-the-badge&logo=Notion&logoColor=white"> <img src="https://img.shields.io/badge/Discord-FDA061?style=for-the-badge&logo=Discord&logoColor=white">



- **진행 기간**: 2023.2.27  ~ 2023.4.7


## ✨ 기획 배경

---

[도로교통공단 | 사고유형별 교통사고](https://taas.koroad.or.kr/sta/acs/gus/selectAcdntTyTfcacd.do?menuId=WEB_KMP_OVT_MVT_TAG_ATT)

![crush_graph](https://user-images.githubusercontent.com/109489851/231025194-f01c5854-43de-4920-a0cc-fffcd7c19ef6.PNG)

도로교통공단에서 분석한 교통사고 통계에 따르면 차대차 사고가 월등히 높음
<br>
→ 주행 중 발생할 수 있는 차대차 사고에 대한 충돌 방지 알고리즘을 고도화
<br>
→ → 안정성 증대

<br>




## 🙌 주요 기능

- 서비스 설명 : 사고 및 돌발 상황에 대해 성능이 탁월한 자율주행 시스템

- 주요 기능 :
  - [ROS]
    - 노드 간의 Publisher와 Subscriber를 통한 통신 및 rviz와 rqt를 통한 시각화
  - [인지 알고리즘]
    - GPS, IMU를 통한 Localization 인식, Odometry 생성
    - Dijkstra를 통한 전역경로 및 지역경로 계획
    - 카메라 센서를 통한 차선 인지 및 YOLO를 통한 객체 탐지
    - 라이다 센서를 통한 Point cloud clustering
  - [판단&제어 알고리즘]
    - Pure pursuit 알고리즘을 통한 조향각 계획
    - 경로기반 속도 계획 및 PID 제어를 통한 속도 제어
    - ACC 알고리즘
    - 경로(차선)변경 및 Lattice planner 충돌 회피 알고리즘

## Lidar
## Camera
## YOLO

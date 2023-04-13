

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

<img src="https://img.shields.io/badge/ROS-22314E?style=for-the-badge&logo=ROS&logoColor=white"> <img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=Python&logoColor=white"> <img src="https://img.shields.io/badge/PyTorch-EE4C2C?style=for-the-badge&logo=PyTorch&logoColor=white"> <img src="https://img.shields.io/badge/scikitlearn-F7931E?style=for-the-badge&logo=scikitlearn&logoColor=white"> <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=OpenCV&logoColor=white"> <img src="https://img.shields.io/badge/YOLO-00FFFF?style=for-the-badge&logo=YOLO&logoColor=white"> <img src="https://img.shields.io/badge/Morai-000000?style=for-the-badge&logo=Morai&logoColor=white">
 
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

## 경로계획

- GPS 센서와 IMU 센서로부터 데이터를 받은 후 WGS84 좌표기반 데이터를 UTM 좌표로 변환 후 MGeo 데이터와 통합하여 정밀도로 지도를 제작하고 Odometry를 생성하였습니다.
- 이를 통해 전역 경로 계획과 지역 경로 계획을 진행하였습니다. 

![dijkstra](https://user-images.githubusercontent.com/109489851/231050240-dd9c4dc9-54d5-4e4e-b2da-4d9eb857c8fc.gif)
- Dijkstra
  - MGeo 데이터를 통해 시작점과 도착점을 찍으면 dijstra를 통한 전역 경로 계획을 실시합니다.
  - 이 전역 경로를 통해 지역 경로 계획을 실시하게 됩니다.

## Lidar

- Lidar 센서를 통해 받아온 pointcloud 데이터를 scikit-learn의 DBSCAN을 통해 clusting하여 주변 객체들을 파악할 수 있습니다.

![ACC](https://user-images.githubusercontent.com/109489851/231050779-e3cad656-22f1-4b6b-9ab0-e229cd03b877.gif)
- ACC
  - Pure pursuit 알고리즘을 통해 조향각 계획을 진행하였습니다.
  - 곡률 계산을 통한 경로기반 속도 계획 및 PID 제어를 통해 속도 제어를 진행하였습니다.
  - ACC 알고리즘을 통해 Lidar로 인식된 앞 차에 대해 안전거리를 유지하며 주행하도록 진행하였습니다.

![lane_change](https://user-images.githubusercontent.com/109489851/231051150-a7e087a2-c39a-4dc8-b0d7-378184275997.gif)
- Lane Change
  - 자연스러운 차선 변경을 위해 현재 차선 내의 시작점과 변경할 차선 내의 도착점에 대해 3차곡선을 계획하여 보다 부드러운 차선 변경이 가능하도록 진행하였습니다.

![lattice_planner](https://user-images.githubusercontent.com/109489851/231051557-e76454ea-52d3-4aae-884a-6ccace69c3c9.gif)
- Lattice Planner
  - Lidar로 장애물을 인식했을 때 피할 수 있는 회피경로를 Lattice Path Planner를 통해 생성하여 충돌을 회피할 수 있도록 진행하였습니다.

## Camera

- Camera 센서를 통해 받아온 데이터를 openCV를 통해 이진화, RoI, BEV 등을 진행하여 주행에 도움이 되는 정보를 얻을 수 있습니다.

![lane_detection](https://user-images.githubusercontent.com/109489851/231051602-ee3ad10d-0749-4ef6-b921-37de4979edb6.gif)
- Lane Detection
  - 전방 도로 이미지를 처리한 후 scikit-learn의 linear 모델인 RANSAC을 이용하여 curve fitting을 통해 차선을 인지하여 GPS정보가 없을때도 차선 인식을 통한 주행이 가능하도록 진행하였습니다.

![color_detection](https://user-images.githubusercontent.com/109489851/231051780-a92c2a62-f07d-4f7b-a0a3-ba6d10b2f301.gif)
- 도로 인식
  - 전방 도로 이미지를 처리한 후 색상 인지를 통해 어린이 보호구역, 요금소 진입 구간 등을 인식했을 때 속도를 조절할 수 있도록 진행하였습니다.

## YOLO

- YOLOv5를 통해 카메라에서 차량, 사람, 신호등 등을 인식할 수 있도록 하였습니다.

![pedestrian_detection](https://user-images.githubusercontent.com/109489851/231052040-0c3c4c70-8195-43f3-a55b-a2b0033f8b77.gif)
- 보행자 인식
  - 인식된 보행자가 차량에 가까워지는 경우 긴급 정지가 되도록 진행하였습니다.

![traffic_stop](https://user-images.githubusercontent.com/109489851/231052187-124af8ca-995e-4add-b5a0-d3cfb28933ce.gif)
![traffic_go](https://user-images.githubusercontent.com/109489851/231052233-c2191b7f-5235-459d-9d29-0eecdede6673.gif)
- 신호 인식
  - 인식된 신호등에 대해 어떤 신호인지 파악하는 알고리즘을 통해 빨간 불이라면 멈추고, 파란 불이라면 주행하도록 진행하였습니다.
 
## P.S.

- 모든 기능에 대한 통합을 진행하였지만 제공되는 시스템 사양의 한계로 인해 통신이 느리고, 시뮬레이터가 끊기는 등 진행이 불가능하게 되었기에 각 탐지기능에 따른 시나리오를 나누고 이를 고도화하는 방향으로 진행하였습니다.

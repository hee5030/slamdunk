무인 실내 경비 로봇 
- 순찰 기능
- 수동 운전 기능
- 장애물 회피 기능
- 사람 인식 후 알림 기능
- 화재 감지 기능
- 웹 모니터링 기능


Ros 종속패키지 설치
sudo apt install ros-melodic-teleop-twist-keyboard
sudo apt-get install ros-melodic-joint-state-publisher-gui
sudo apt install ros-melodic-ros-controllers
sudo apt install ros-melodic-webots-ros

CAN-USB 어댑터 설정

1.gs_usb 커널 모듈 활성화
$ sudo modprobe gs_usb

2. can 장치 불러오기
$ sudo ip link set can0 up type can bitrate 500000

3.이전 단계에서 오류가 발생하지 않았다면 이제 명령을 사용하여 can 장치를 볼 수 있어야 합니다.
$ ifconfig -a

4.can-utils를 설치 및 사용하여 하드웨어 테스트
$ sudo apt install can-utils

5.테스트 명령
# receiving data from can0 / 밑에 명령어 실행시 입력값을 불러올수 있다.
$ candump can0
# send data to can0
$ cansend can0 001#1122334455667788

https://github.com/agilexrobotics/scout_ros

1.종속 라이브러리 설치
$ sudo apt install -y libasio-dev

2.패키지를 catkin 작업 공간에 복제하고 컴파일합니다.
$ cd ~/catkin_ws/src
$ git clone https://github.com/agilexrobotics/scout_ros.git
$ cd ..
$ catkin_make

-스카우트를 위한 기본 노드 시작

$ roslaunch scout_bringup scout_robot_base.launch 

scout-mini의 기본 노드 시작

$ roslaunch scout_bringup scout_mini_robot_base.launch
키보드 원격 작동 노드 시작

$ roslaunch scout_bringup scout_teleop_keyboard.launch

- [**https://omorobot.com/docs/ros-원격-구동/**](https://omorobot.com/docs/ros-%EC%9B%90%EA%B2%A9-%EA%B5%AC%EB%8F%99/)
    
    **ROS 원격 구동 관련**
    

-Master PC

$ sudo vi ~/.bashrc

exprot ROS_MASTER_URI=http://자신의 IP:11311

export ROS_HOSTNAME=자신의 IP

-라즈베리파이3

$ sudo vi ~/.bashrc

exprot ROS_MASATER_URI=http:// Master PC IP:11311

export ROS_HOSTNAME=자신의 IP

scout_base.launch 에서 /dev/ttyUSB0 →can0로 스크립트를 수정 해주어야 한다.

-usb 연결 되어있는지 꼭 확인

$ sudo modprobe gs_usb

처음에 scout-ros 패키지 실행할 때

$ rosrun scout_bringup setup_can2usb.bash

여러 번 실행 시킨 다음

$ rosrun scout_bringup bringup_can2usb.bash

테스트

$candump can0

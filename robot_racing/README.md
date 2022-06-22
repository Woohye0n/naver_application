# 2021 세계 AI 로봇 카레이스 대회

- 이 리포지토리는 2021 세계 AI 로봇 카레이스 대회 참가를 위해 작성되었습니다.
- 이 리포지토리는 ros package를 포함하고 있습니다. 
- 모든 패키지의 이름에는 앞에 rr (Robot Racing의 약자)를 붙여 사용하도록 합니다. 예를 들어 sensors package가 있다면 rr_sensors 로 작성합니다.


이 패키지는 catkin tools를 사용해 빌드합니다. catkin_make 가 아님에 유의하기 바랍니다.

- catkin build error

<Eigen/Dense> Error
cd /usr/include
sudo ln -sf eigen3/Eigen Eigen
sudo ln -sf eigen3/unsupported unsupported

<pcap.h> Error
sudo apt-get update
sudo apt-get install libpcap-dev


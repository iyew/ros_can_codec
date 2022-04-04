ROS CAN Message Codec
=====================





## 설명(About)
raw CAN frame을 ROS message화 및 decoding, encoding



### Topics
ROS CAN message 개수만큼 Publisher와 Subscriber 생성됨    
> */[ros_cancodec_message]_pub*: published topics    
> */[ros_cancodec_message]_sub*: subscribed topics    





## 설치(Installation)


### 필요 패키지(requirements)
```
sudo apt update && sudo apt install -y python3-pip net-tools can-utils
...
python3 -m pip install cantools
```


### ROS Workspace 생성 및 빌드
```
mkdir -p [your_workspace_name]/src
cd [your_workspace_name]/src
git clone https://github.com/hjr1015/intern-can-tool
cd ..
catkin_make
```


### 장비 연결
다음 명령을 입력한 후 PCAN 장비를 PC와 연결하였을 때, 다음과 같이 장비를 인식하는지 확인
```
dmesg -w
...
[ 88.218865] peak_usb 1-2:1.0: PEAK-System PCAN-USB Pro FD v2 fw v3.2.0 (2 channels)
[ 88.239159] peak_usb 1-2:1.0 can0: attached to PCAN-USB Pro FD channel 0 (device
4294967295)
...
```


### CAN 인터페이스 활성화
다음 명령 입력 혹은 패키지 내 enable_cans.sh 실행
```
sudo ip link set can0 up type can bitrate 500000
```
비활성화 시 다음 명력 입력 혹은 패키지 내 disable_cans.sh 실행
```
sudo ifconfig can0 down
```





## 사용법(Usage)


### ros_cancodec 실행
```
roscore
..
source devel/setup.bash
rosrun ros_cancodec codec.py
```


### 임의의 ROS message publish 방법 예시
```
rostopic pub [topic_name] [ros_message_type] -- [value or data dictionary]
```
ex) rostopic pub /Steering_angle_sub ros_cancodec/Steering_angle -- 12    
rostopic pub /Control_sub ros_cancodec/Control -- "{'Lon_Con_En': 'On', 'Lat_Con_En': 'On', 'Lat_Over_Mod': 'Ignore', 'Steering_CMD': 12, 'Brake_CMD': 23, 'Accel_CMD': 34}"



### raw CAN frame 송신
```
cansend can0 [can_id]#[data_frame]
```
ex) cansend can0 2B0#D204000000    
cansend can0 2B2#21D2041601000000



### raw CAN frame 주기적으로 랜덤 생성
```
cangen can0 -g [period(ms)] -I [can_id] -L [dlc]
```
ex) cangen can0 -g 1000 -I 2B0 -L 5    
cangen can0 -g 1000 -I 2B2 -L 8





## 부록


### dbc 파일 변경
1. 패키지 내 dbc 폴더에 새 dbc 파일 추가
2. scripts 폴더 내 generate_ros_cancodec_msg.py의 10번 째 줄에서 D2S_CAN.dbc을 해당하는 파일명으로 바꾸고 스크립트 실행
```
Line 10: DBC_FILE_PATH = os.path.join(SCRIPT_DIR, '..', 'dbc', '[foo.dbc]')
...
./generate_ros_cancodec_msg.py
```
3. CMakeLists.txt에서 add_message_files 항목을 찾아 생성된 msg 파일을 기재
```
add_message_files(
  FILES
  [foo.dbc]
)
```
4. catkin_make

# ROS2 SHM Bridge 사용법

기존 PtoP SHM 제어를 유지하면서, ROS2로 Franka/Hand 상태를 읽고 타겟을 쓸 수 있는 브리지입니다.

---

## 터미널 1 — SHM + EtherCAT + Franka 제어 (메인 실시간 프로세스)

```bash
cd /home/prime/Franka_KISTAR_R_Exp_V1.2_PtoP

# 빌드 (처음 또는 코드 수정 후)
mkdir -p build && cd build
cmake .. && make -j$(nproc)
cd ..

# 실행 (sudo 필요: EtherCAT은 raw socket 사용)
sudo ./build/test/Franka_KISTAR_R_Exp_V1
```

이 프로세스가 하는 일:
- EtherCAT으로 KISTAR Hand 제어
- Franka 로봇 연결 (`172.16.0.1`) + PtoP 모션
- SHM에 상태 쓰기 / 타겟 읽기

---

## 터미널 2 — ROS2 SHM 브리지 노드

```bash
cd /home/prime/Franka_KISTAR_R_Exp_V1.2_PtoP/ros2
./run_hand_bridge.sh
```

또는 직접:

```bash
cd /home/prime/Franka_KISTAR_R_Exp_V1.2_PtoP/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=9
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

ros2 launch kistar_hand_bridge hand_bridge.launch.py
```

이 노드가 하는 일:
- SHM 읽기 → ROS2 토픽으로 publish (1kHz)
- ROS2 토픽 subscribe → SHM에 쓰기

---

## 터미널 3 — ROS2로 상태 읽기 / 타겟 쓰기

### 환경 세팅 (이 터미널에서 한 번만)

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=9
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

### 상태 읽기 (echo)

```bash
# Franka 현재 관절 각도 (7축, double)
ros2 topic echo /franka/joint_position

# Franka 현재 타겟 확인
ros2 topic echo /franka/joint_target

# Franka 속도/토크
ros2 topic echo /franka/joint_velocity
ros2 topic echo /franka/joint_torque

# Hand 현재 위치 (16축, int16)
ros2 topic echo /hand/joint_position

# Hand servo_on / hand_mode 확인
ros2 topic echo /hand/servo_on --once
ros2 topic echo /hand/hand_mode --once
```

### 타겟 쓰기 (pub)

```bash
# 1) Hand 서보 ON
ros2 topic pub /hand/target_servo_on std_msgs/msg/Int32 "{data: 1}" --once

# 2) Hand 모드 설정
ros2 topic pub /hand/target_mode std_msgs/msg/Int32 "{data: 1}" --once

# 3) Hand 타겟 보내기 (16축)
ros2 topic pub /hand/target_joint std_msgs/msg/Int16MultiArray \
  "{data: [100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0]}" --once

# 4) Franka 속도 설정 (0.001 ~ 1.0)
ros2 topic pub /franka/target_speed_factor std_msgs/msg/Float64 \
  "{data: 0.1}" --once

# 5) Franka 타겟 보내기 (7축, radian)
ros2 topic pub /franka/target_joint std_msgs/msg/Float64MultiArray \
  "{data: [0.0, -0.7, 0.0, -2.3, 0.0, 1.5, 0.7]}" --once
```

---

## 전체 토픽 요약

### 읽기 (SHM → ROS2 publish)

| 토픽 | 타입 | 크기 |
|------|------|------|
| `/franka/joint_position` | `Float64MultiArray` | 7 |
| `/franka/joint_target` | `Float64MultiArray` | 7 |
| `/franka/joint_velocity` | `Float64MultiArray` | 7 |
| `/franka/joint_torque` | `Float64MultiArray` | 7 |
| `/franka/speed_factor` | `Float64` | 1 |
| `/hand/joint_position` | `Float32MultiArray` | 16 |
| `/hand/joint_kinesthetic` | `Float32MultiArray` | 12 |
| `/hand/joint_tactile` | `Float32MultiArray` | 60 |
| `/hand/joint_target` | `Int16MultiArray` | 16 |
| `/hand/hand_mode` | `Int32` | 1 |
| `/hand/servo_on` | `Int32` | 1 |

### 쓰기 (ROS2 subscribe → SHM)

| 토픽 | 타입 | 크기 |
|------|------|------|
| `/franka/target_joint` | `Float64MultiArray` | 7 |
| `/franka/target_speed_factor` | `Float64` | 1 |
| `/hand/target_joint` | `Int16MultiArray` | 16 |
| `/hand/target_mode` | `Int32` | 1 |
| `/hand/target_servo_on` | `Int32` | 1 |

---

## 옆 연구실 PC (LAN 통신)

### 전제 조건
- 두 PC가 같은 LAN(유선/무선)에 연결
- 양쪽 모두 ROS 2 Humble 설치

### 네트워크 확인

```bash
# 로봇 PC IP 확인
hostname -I

# 옆 연구실 PC에서 ping
ping <로봇_PC_IP>
```

### 환경 세팅 (옆 연구실 PC, 모든 터미널에서)

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=9
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

이 4줄만 맞추면 위의 `ros2 topic echo` / `ros2 topic pub` 명령을 그대로 사용할 수 있습니다.

### 안 될 때 체크리스트

| 증상 | 해결 |
|------|------|
| `ros2 topic list`에 아무것도 안 보임 | `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` 양쪽 일치 확인 |
| `ROS_LOCALHOST_ONLY` 미설정 | 반드시 `export ROS_LOCALHOST_ONLY=0` |
| 방화벽 차단 | `sudo ufw allow 7400:7500/udp` |
| 다른 서브넷 | 두 PC가 같은 네트워크 대역인지 확인 |

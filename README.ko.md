# nlink_parser_ros2

[Nooploop LinkTrack / LinkTrack AoA](https://www.nooploop.com/) UWB 모듈을 위한 ROS 2 드라이버.
제조사의 ROS 1 [nlink_parser](https://github.com/nooploop-dev/nlink_parser)에서 시작해
ROS 2 관용 표현(파라미터 시스템, 메시지 헤더, event-driven publish, 자동 재연결)에 맞게 재작성됨.

> English: [README.md](README.md)

## 상태

- **지원 장치**: LinkTrack P-A 시리즈 (LP / DR / DT 모드), LinkTrack AoA
- **미지원**: ToFSense (upstream ROS 1 코드는 제거됨)
- **검증 환경**: ROS 2 Jazzy / Ubuntu 24.04 / 커널 6.x

## 워크스페이스 구조

```
nlink_parser_ros2/
├── src/
│   ├── nlink_parser_ros2/             # 메인 노드 + launch + params
│   ├── nlink_parser_ros2_interfaces/  # ROS 2 메시지 정의
│   └── serial/                         # submodule: Sunnybotics/serial (ROS 2 브랜치)
└── README.md / README.ko.md
```

써드파티 C 파서(`nlink_unpack`, `protocol_extracter`)는 git submodule이 아닌 일반
디렉터리로 `src/nlink_parser_ros2/src/utils/` 아래에 vendoring되어 있음.

## 사전 요구사항

### ROS 2 Jazzy

[공식 가이드](https://docs.ros.org/en/jazzy/Installation.html) 참고. 빌드/실행 시
매번 `source /opt/ros/jazzy/setup.bash` 필요.

### CH343 USB-UART 시리얼 드라이버

LinkTrack P-A 케이블은 WCH CH343으로 인식됨. 두 가지 방법:

1. **기본 커널의 cdc_acm** — Ubuntu 24.04 다수 환경에서 자동 인식되어
   `/dev/ttyACM*`에 노출됨. 먼저 이 방법을 시도해서 동작하면 2번은 생략.
2. **WCH ch343 드라이버** — cdc_acm으로 안 보일 때만 필요.
   [WCHSoftGroup/ch343ser_linux](https://github.com/WCHSoftGroup/ch343ser_linux)에서 빌드:
   ```bash
   cd ch343ser_linux/driver
   make            # ch343.ko 빌드
   sudo make load  # 일회성 insmod
   sudo make install  # 재부팅 후에도 유지
   ```
   커널 6.x에서는 `ch343.c`의 `<asm/unaligned.h>` 헤더가 `<linux/unaligned.h>`로 옮겨졌으므로
   make 전에 61번 줄을 수정해야 함.

성공 시 `/dev/ttyCH343USB0` (WCH) 또는 `/dev/ttyACM0` (cdc_acm)으로 인식됨.

### 사용자 dialout 그룹 가입

```bash
sudo usermod -aG dialout "$USER"
# 로그아웃 후 재로그인, 또는 `newgrp dialout`
```

## 빌드

```bash
git clone --recursive <this-repo>
cd nlink_parser_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

> Submodule은 `.gitmodules`의 `update = checkout`으로 핀 고정되어 있음.
> 재현 가능한 빌드를 위해 `git submodule update --init`만 사용 (`--remote` 사용 금지).

## 실행

```bash
source install/setup.bash
ros2 launch nlink_parser_ros2 linktrack.launch.py
# 또는
ros2 launch nlink_parser_ros2 linktrack_aoa.launch.py
```

명령줄에서 파라미터 오버라이드:

```bash
ros2 launch nlink_parser_ros2 linktrack.launch.py \
    --ros-args -p port_name:=/dev/ttyACM0 -p frame_id:=base_link
```

## 파라미터

| 이름                   | 타입   | 기본값                  | 설명                                       |
|------------------------|--------|-------------------------|--------------------------------------------|
| `port_name`            | string | `/dev/ttyCH343USB0`     | 시리얼 포트 경로                           |
| `baudrate`             | int    | `921600`                | UART baud rate                             |
| `frame_id`             | string | `uwb_link`              | 모든 발행 메시지의 `header.frame_id`       |
| `serial_read_rate_hz`  | double | `100.0`                 | 시리얼 RX 버퍼 비우는 주기 (Hz)            |

운용 중 시리얼 포트가 끊기면 (예: 모바일 플랫폼에서 USB 케이블 단절) 노드는
2초 주기로 자동 재연결을 시도하며 복구되면 INFO 로그를 남김.

## 토픽 / 모드 매핑 (LinkTrack)

LinkTrack은 NAssistant로 설정한 Mode/Role에 따라 서로 다른 프로토콜 프레임을 송출함.
현재 모드에서 송출하지 않는 프레임은 해당 토픽에 publish되지 않음 (의도된 동작).

| 모드 / 역할          | 활성 토픽                                                 |
|----------------------|-----------------------------------------------------------|
| LP — TAG             | `tagframe0`, `nodeframe2`, `nodeframe3`                   |
| LP — ANCHOR/CONSOLE  | `anchorframe0`, `nodeframe1`, `nodeframe0` (데이터 수신)  |
| DR_MODE0             | `nodeframe2`, `nodeframe3`, `nodeframe0` (데이터 수신)    |
| DR_MODE1             | `nodeframe5`, `nodeframe6`                                |

`/nlink_linktrack_data_transmission`은 `std_msgs/String` **subscription**임.
사용자 노드에서 이 토픽으로 publish하면 통신 범위 내 모든 peer 노드에 broadcast됨.

DR 모드에서 `nodeframe2`의 `pos_3d / vel_3d / quaternion / imu_*` 필드는 0으로 채워짐
(하드웨어가 위치 해를 풀지 않음). 모드별 필드 유효성은 데이터시트 §5.1.2 및 각 `.msg`
파일의 주석 참고.

## 라이선스

BSD 3-Clause. [src/nlink_parser_ros2/LICENSE](src/nlink_parser_ros2/LICENSE) 참조.
Vendoring된 써드파티 코드는 각 upstream 라이선스를 유지함.

## 메인테이너

YG_Kim — `yeogyeom1@chungbuk.ac.kr`

ROS 2 포팅 원작자: Aarush Gupta (HopeTechnik). Upstream ROS 1 코드: Samuel Hsu (Nooploop).

## 기여

버그 / 기능 요청은 이 저장소에 issue로 등록.
스타일: `.clang-format` (Google, 100 col)과 `.clang-tidy`가 포함되어 있으니 PR 전 적용 부탁.

# ArmPi ROS 2 내부 인터페이스 (Humble)

작성 2026-11-02. 상태: 구현 기준 문서. 언어: 로봇 노드는 **C++17 / rclcpp / ament_cmake**.

## 패키지와 담당

| 패키지 | 내용 | 비고 |
|---|---|---|
| `armpi_interfaces` | 메시지 정의 (`ArmTarget.msg`) | |
| `armpi_kinematics` | 팔 IK/FK 라이브러리(헤더 + 정적 라이브러리), 노드 없음. 2026-11-11: 목표 보정 `TargetProjector`(`reachability.hpp`)와 매개변수 읽기 도우미(`ros_params.hpp`, header-only, rclcpp 쓰는 패키지만) | 링크 길이: `ros2_ws/src/armpi_bringup/config/armpi.yaml`, 관절 기준·범위: 하드웨어 프로필. 보정 매개변수 `arm.target_policy`·`arm.nearest.*`(armpi.yaml `/**`, 수신기·웹 조종기·제어기 공통) |
| `armpi_vr_bridge` | Protocol v3 UDP 수신기(5001) + 검색 응답(5002) 노드 | `docs/01_PROTOCOL_V3_SPEC.md` 구현 |
| `armpi_control` | 제어기: 비상정지 latch, 2차 watchdog, 차체 cmd_vel → 바퀴, 팔 목표 보간 → 관절 명령, 상태 발행 | |
| `armpi_hardware` | 하드웨어 드라이버(버스 서보 직렬, 확장보드 모터·부저·LED·배터리). `mock:=true`면 가짜 하드웨어 | 장치 설정: `armpi.yaml`(공통)과 하드웨어 프로필(로봇마다 다른 값, `config/hardware_profile.template.yaml`) |
| `armpi_bringup` | launch, `config/armpi.yaml`, `config/fastdds_armpi.xml`(v2) | systemd 유닛은 `deploy/systemd/` |
| `armpi_common` (v2) | 공용 JSON 라이브러리, 노드 없음 | `13_SOFTWARE_ARCHITECTURE_KO.md` |
| `armpi_protocol` (v2) | Protocol v3 형식 + C++ 클라이언트, 노드 없음 | 수신기와 웹 조종기가 같이 씀 |
| `armpi_web_gateway` (v2) | 로봇 안 웹 조종기 노드(HTTP 18765). 구독 `/armpi/status`·`/armpi/battery`, 서비스 호출 `/armpi/emergency_stop`·`/armpi/reset_estop`·`/armpi/arm/home`, 발행 `/armpi/arm/joint_debug`. 조종은 UDP 5001 로만 | v1 파이썬 조종기 대체 |
| `armpi_power` (v2) | 구독 `/armpi/shutdown_requested` → `systemctl poweroff` | v1 `ros2 topic echo` 대체 |

## 토픽 / 서비스 (QoS 기본: reliable, depth 10. 표시한 것만 예외)

### 수신기(`armpi_vr_bridge`) → 제어기

| 이름 | 타입 | QoS | 내용 |
|---|---|---|---|
| `/armpi/cmd_vel` | `geometry_msgs/TwistStamped` | reliable, depth 1 | frame `base_link`. `linear.x`, `linear.y`(m/s), `angular.z`(rad/s). 20 Hz + 값 변경 시 즉시. 정지 = 모두 0 |
| `/armpi/arm/target` | `armpi_interfaces/ArmTarget` | reliable, depth 1 | 팔 목표(손끝 절대 위치). 수락된 `arm_command`마다 1회 + 팔 채널이 active인 동안 20 Hz 재발행. `active:false` = 현재 위치 유지 |
| `/armpi/estop_request` | `std_msgs/String` (JSON) | reliable, depth 10 | `{"source":"udp","latch":true}` = 비상정지 걸기, `{"source":"udp","latch":false}` = 소유자의 해제 요청 |
| `/armpi_vr_bridge/state` | `std_msgs/String` (JSON) | reliable, **transient_local**, depth 1 | 2 Hz + 변화 시. `{"owner":uuid|null,"owner_addr":"ip:port"|null,"base_state":…,"arm_state":…,"emergency_stop":bool,"packets":{…카운터}}` |

### 제어기(`armpi_control`) → 수신기·외부

| 이름 | 타입 | QoS | 내용 |
|---|---|---|---|
| `/armpi/control_state` | `std_msgs/String` (JSON) | reliable, **transient_local**, depth 1 | 2 Hz + 변화 시. `{"estop":bool,"estop_sources":[…],"fault":bool,"fault_reason":str,"udp_allowed":bool}` |
| `/armpi/status` | `std_msgs/String` (JSON) | reliable, depth 1 | 2 Hz. 사용자 표시용 종합 상태(배터리 V, 팔 목표/현재, `position_source`:"commanded"/"measured"). v2: `kernel`(실행 중 커널), `boot_warnings`(커널·부팅 설정이 이미지와 다를 때 경고 문장 목록) |
| `/armpi/battery` | `sensor_msgs/BatteryState` | | 1 Hz |
| `/joint_states` | `sensor_msgs/JointState` | | 20 Hz. 이름 `joint1`…`joint5`, `r_joint` |
| 서비스 `/armpi/emergency_stop`, `/armpi/reset_estop`, `/armpi/arm/home` | `std_srvs/Trigger` | | |
| `/armpi/arm/joint_debug` | `sensor_msgs/JointState` | | 입력. UDP 세션이 없을 때만 적용 |

### 수신기 규칙 (제어기 상태와의 관계)

- `/armpi/control_state`를 한 번도 받지 못했거나, 마지막 수신이 **2.0 s** 넘게 지났으면 활성 명령을 `not_owner`로 거부한다. 중립 명령은 수락한다.
- `udp_allowed:false`면 활성 명령을 `not_owner`로 거부한다.
- `estop:true`면 세션 비상정지 상태로 보고하고 활성 명령을 `emergency_stop`으로 거부한다.
- `fault:true`면 활성 명령을 `fault`로 거부한다.
- 노드 종료 시 정지 명령(`cmd_vel` 0, `ArmTarget active:false`)을 3회 발행한다.

### 제어기 2차 방어

- `/armpi/cmd_vel`을 **0.6 s** 동안 받지 못하면 차체를 정지한다(수신기가 죽었을 때 대비).
- 메시지 `header.stamp`가 **0.5 s**보다 오래된 `cmd_vel`은 버린다(같은 기기 내부 ROS 시계 기준).
- `ArmTarget`이 0.6 s 동안 안 오면 팔 목표를 현재 보간 위치로 고정한다. 토크는 유지한다.

## 메시지 정의

`armpi_interfaces/msg/ArmTarget.msg`
```
std_msgs/Header header
bool active        # false = 현재 위치 유지(새 목표 아님)
float64 x          # m, armpi_arm_base (REP-103: x 앞, y 왼쪽, z 위)
float64 y
float64 z
float64 pitch      # rad, 0 = 수평 앞, -pi/2 = 아래
float64 gripper    # 0 = 열림, 1 = 닫힘
```

## 기구학 라이브러리 API (`armpi_kinematics`)

```cpp
namespace armpi_kinematics {
struct Params {            // 모두 config/armpi.yaml 의 arm.kinematics 에서 읽는다
  double base_height_m;    // armpi_arm_base 원점 → 어깨 관절축 높이
  double upper_arm_m;      // 어깨 → 팔꿈치
  double forearm_m;        // 팔꿈치 → 손목
  double hand_m;           // 손목 → 손끝(그리퍼 끝점)
  // 기하 각도 → 관절 각도 변환: joint = sign * geometric + offset (rad)
  double j1_sign, j1_offset, j2_sign, j2_offset, j3_sign, j3_offset, j4_sign, j4_offset;
  double joint_min[4], joint_max[4];   // joint1..joint4 한계(rad)
};
struct Pose { double x, y, z, pitch; };
struct JointAngles { double j1, j2, j3, j4; };   // joint1(베이스)..joint4(손목 pitch)
enum class IkStatus { kOk, kOutOfReach, kJointLimit, kInvalid };
struct IkResult { IkStatus status; JointAngles joints; };

class ArmKinematics {
 public:
  explicit ArmKinematics(const Params& p);
  IkResult inverse(const Pose& target) const;          // 팔꿈치 위(elbow-up) 해
  Pose forward(const JointAngles& q) const;
  const Params& params() const;
};
struct Workspace { double x_min, x_max, y_min, y_max, z_min, z_max; };
bool inside(const Workspace& w, const Pose& p);
}  // namespace armpi_kinematics
```

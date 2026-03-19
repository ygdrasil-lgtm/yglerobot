# Gripper Haptic Feedback Modes

그리퍼에 두 가지 haptic feedback 모드를 제공하여, 안정성과 반응성 사이의 최적 지점을 찾을 수 있습니다.

## 모드 1: Proportional Mode (기본값, 현재 방식)

**설명:**
- Follower 그리퍼의 전류값에 **비례하여** leader에 피드백을 제공합니다.
- 물건을 잡는 힘이 강할수록 leader에 주어지는 저항감도 강합니다.
- 더 높은 **haptic transparency** (투명성)을 제공합니다.

**특징:**
- ✓ 물체의 중량과 저항을 직관적으로 느낄 수 있음
- ✓ 섬세한 터치 제어 가능
- ⚠ 노이즈나 급격한 전류 변화 시 **chattering** 발생 가능

**사용:**
```bash
lerobot-teleoperate \
  --config-path teleoperator=turretL \
  --teleop.gripper_feedback_mode=proportional
```

**Config 예제:**
```python
from lerobot.teleoperators.turret_leader import TurretLeaderConfig, TurretLeader

cfg = TurretLeaderConfig(
    port='/dev/ttyUSB0',
    gripper_feedback_mode='proportional',  # 비례 모드
    # 다른 설정...
)
leader = TurretLeader(cfg)
```

---

## 모드 2: Threshold-Constant Mode (새로운 스태빌리티 모드)

**설명:**
- 임계값(threshold)을 초과하는 순간 **일정한 저항력(constant force)만** 제공합니다.
- 임계값 미만: 피드백 없음 (0 mA)
- 임계값 이상: 고정된 저항감 (예: 30 mA)
- **Chattering 방지**를 위한 스태빌 모드입니다.

**특징:**
- ✓ Chattering 완전 방지
- ✓ 안정적이고 부드러운 피드백
- ✓ 물건 접촉 순간(임계값)을 기준으로 단순한 상태 제어
- ⚠ Haptic transparency 낮음 (정확한 힘 피드백 불가)
- ⚠ 물체의 무게 차이를 느끼기 어려움

**파라미터:**

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `gripper_feedback_threshold_ma` | 50.0 | 물건 접촉 판별 임계값 (mA) |
| `gripper_feedback_constant_force_ma` | 30.0 | 임계값 초과 시 제공할 고정 저항력 (mA) |

**사용:**
```bash
lerobot-teleoperate \
  --config-path teleoperator=turretL \
  --teleop.gripper_feedback_mode=threshold_constant \
  --teleop.gripper_feedback_threshold_ma=50.0 \
  --teleop.gripper_feedback_constant_force_ma=30.0
```

**Config 예제:**
```python
cfg = TurretLeaderConfig(
    port='/dev/ttyUSB0',
    gripper_feedback_mode='threshold_constant',
    gripper_feedback_threshold_ma=50.0,      # 임계값: 50 mA에서 접촉 판별
    gripper_feedback_constant_force_ma=30.0, # 고정 저항: 30 mA
)
leader = TurretLeader(cfg)
```

---

## 파라미터 튜닝 가이드

### Threshold 조절

**너무 낮은 경우 (예: 20 mA)**
- 문제: 약한 터치에도 저항이 생김
- 결과: 민감한 제어, 원치 않은 활성화 가능

**너무 높은 경우 (예: 100 mA)**
- 문제: 실제 물건 접촉 감지 안 됨
- 결과: 물건을 놓쳤을 때 느낄 수 없음

**권장 범위:** 40-70 mA (실험을 통해 최적값 찾기)

### Constant Force 조절

**너무 약한 경우 (예: 10 mA)**
- 문제: 저항감이 너무 약함
- 결과: 물건 잡고 있다는 느낌이 약함

**너무 강한 경우 (예: 60 mA)**
- 문제: 저항이 너무 강해 조작이 힘들어짐
- 결과: 피로 증가, 미세한 제어 어려움

**권장 범위:** 20-40 mA (그리퍼 강도와 작업 특성에 맞게)

---

## 비교 테이블

| 항목 | Proportional | Threshold-Constant |
|------|------------|-------------------|
| **Haptic Transparency** | 높음 | 낮음 |
| **Chattering** | 가능성 있음 | 완전 방지 |
| **안정성** | 중간 | 높음 |
| **물체 감도** | 뛰어남 | 이진 (접촉/미접촉) |
| **적합 작업** | 정밀 조작 | 반복적 픽앤플레이스 |
| **파라미터 조정** | gain, filter_alpha | threshold, constant_force |

---

## 실장 테스트 프로토콜

두 모드를 번갈아 비교하기 위한 테스트 권장사항:

### 1단계: Proportional 모드
```bash
lerobot-teleoperate \
  --config-path teleoperator=turretL \
  --teleop.gripper_feedback_mode=proportional \
  --teleop.debug_feedback_realtime=true \
  --teleop.debug_feedback_realtime_hz=1.0
```
- 다양한 전류 변화 감지
- Chattering 발생 여부 확인
- 반응성 평가

### 2단계: Threshold-Constant 모드
```bash
lerobot-teleoperate \
  --config-path teleoperator=turretL \
  --teleop.gripper_feedback_mode=threshold_constant \
  --teleop.gripper_feedback_threshold_ma=50.0 \
  --teleop.gripper_feedback_constant_force_ma=30.0 \
  --teleop.debug_feedback_realtime=true \
  --teleop.debug_feedback_realtime_hz=1.0
```
- 안정성 평가
- 임계값이 적절히 감지되는지 확인
- 물건 접촉 시 느낌 평가

### 3단계: 파라미터 튜닝
각 모드에서 데이터 수집 및 파라미터 최적화:
- **Proportional**: `feedback_gain`, `feedback_input_filter_alpha`, `feedback_deadband_ma`
- **Threshold-Constant**: `gripper_feedback_threshold_ma`, `gripper_feedback_constant_force_ma`

---

## Python 코드 예제

### 두 모드 동시 비교
```python
from lerobot.teleoperators.turret_leader import TurretLeaderConfig, TurretLeader

# 모드 1: 비례 피드백
cfg1 = TurretLeaderConfig(
    port='/dev/ttyUSB0',
    gripper_feedback_mode='proportional',
    debug_feedback_realtime=True,
    debug_feedback_realtime_hz=1.0,
)
leader1 = TurretLeader(cfg1)

# 모드 2: 임계값 기반 상수 저항
cfg2 = TurretLeaderConfig(
    port='/dev/ttyUSB1',  # 또는 같은 포트, 시간차로 테스트
    gripper_feedback_mode='threshold_constant',
    gripper_feedback_threshold_ma=50.0,
    gripper_feedback_constant_force_ma=30.0,
    debug_feedback_realtime=True,
    debug_feedback_realtime_hz=1.0,
)
leader2 = TurretLeader(cfg2)

# 테스트 시나리오
test_currents = [10, 25, 50, 75, 100]  # LSB (XM430-W350: 2.69 mA/LSB)

for raw_lsb in test_currents:
    feedback = {"gripper.current": raw_lsb}
    
    print(f"\nFollower current: {raw_lsb} LSB")
    print("Proportional mode feedback:")
    leader1.send_feedback(feedback)
    
    print("Threshold-constant mode feedback:")
    leader2.send_feedback(feedback)
```

---

## 주의사항

1. **Safety**: 두 모드 모두 `max_current_per_joint`로 최대 저항 한계를 설정합니다
2. **Hysteresis**: 임계값에 정확히 도달할 때 떨림이 없는지 확인
3. **Direction**: 음수/양수 전류 모두 올바르게 동작하는지 테스트
4. **Filtering**: Proportional 모드에서 `feedback_input_filter_alpha` 조정이 중요

---

## 업데이트 히스토리

- **v0.1** (2026-03-19): 초기 구현
  - Proportional mode: 기존 비례 피드백
  - Threshold-constant mode: 새로운 스태빌 모드
  - CLI 파라미터 지원

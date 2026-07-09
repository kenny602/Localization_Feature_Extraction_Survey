# LIDAR-based Stabilization, Navigation and Localization for UAVs Operating in Dark Indoor Environments

> **Petráček, Krátký, Báča, Petrlík, Saska** · ICUAS 2021 (CTU Prague, MRS Group)
> [Paper](https://ieeexplore.ieee.org/document/9476837) | 키워드: `2D LiDAR` `sparse scan` `point-to-point ICP` `IDC` `scan-to-map` `online mapping` `Kalman filter fusion`

---
<img width="1191" height="592" alt="image" src="https://github.com/user-attachments/assets/f53ec6f7-d155-431a-98f1-b8b1bd035d12" />


##  분류 (Category)

| 1차 축: 특징 표현 | 해당 여부 |
|---|---|
| ① Point-based | ✅ **원시 점(raw point) 기반 — 단, 학습 descriptor가 아닌 point-to-point 정합** |
| ② Voxel / Distribution-based | △ (맵 저장만 octree cell, 특징 기술은 아님) |
| ③ Geometric Primitive | ❌ **명시적으로 기각** (sparse 스캔에서 primitive 추출 불가 논증) |
| ⑤ Projection-based | ❌ |
| ⑥ Landmark-based | ❌ |

> **본 survey에서의 위치**: 특징 추출의 "하한 경계" 사례. 스캔당 200점 미만의 극단적 sparse 입력에서는
> primitive/descriptor 기반 접근 자체가 성립하지 않음을 보이고, raw point 대응 기반 정합을
> correspondence 설계 수준에서 정밀화하는 접근. ③⑥ 계열 기법의 **성립 전제조건(최소 포인트 밀도)** 을
> 역으로 규정해주는 반례로서 수록.

---

<img width="581" height="321" alt="image" src="https://github.com/user-attachments/assets/428d92c3-a84f-4648-909b-25cb4d76716e" />


## 1. Abstract

카메라 기반 위치추정이 불가능한 어두운 실내(성당, 광산, 터널)에서 소형 UAV의 안정화·항법·위치추정을
수행하는 온보드 실시간 시스템. 경량 2D LiDAR(Scanse Sweep: 5 Hz, 스캔당 <200점)의 극도로 희소하고
노이즈가 많은 스캔을 다루기 위해, 표준 ICP를 correspondence 수준에서 개조한 scan matching을 제안한다.
고주파(5 Hz) scan-to-scan 정합으로 **속도**를, 저주파(1 Hz) scan-to-map 정합으로 **절대 위치**를 추정하고
둘을 Kalman filter로 융합하여 drift-free 추정치를 얻는다. 맵은 사전 맵 없이 비행 중 온라인으로 구축된다.
Chlumin 성당 실험에서 표준 ICP 대비 global ATE 1.064 m → 0.098 m (약 91% 개선)을 달성했다.

---

## 2. Introduction

### 문제 설정
- **환경 제약**: 어두운 실내 → 카메라/VIO 불가. ToF·구조광 카메라는 해상도/거리 한계 및 야외 태양광에 취약.
- **플랫폼 제약**: 협소 공간용 소형 UAV → Velodyne급 3D LiDAR 탑재 불가. 경량 2D LiDAR만 가능.
- **센서 제약**: 저가 경량 LiDAR의 특성 = 낮은 포인트 수(<200/scan), 낮은 주사율(5 Hz), 짧은 거리,
  높은 분산의 노이즈, 잦은 오검출. 또한 2D 스캔인데 UAV는 6-DOF로 움직이므로 tilt 보상 없이는
  스캔이 주변 환경을 올바르게 표현하지 못함.

### 핵심 주장
움직이는 UAV + 느린 회전 LiDAR 조합에서 나오는 스캔은 **overlap이 작고 outlier 비율이 높은
"non-ideal scan"** 이며, 표준 ICP는 이런 입력에 부적합하다. 이를 feature 추출로 우회하는 대신,
correspondence 탐색·비용함수·가중치를 개조한 point-to-point 정합으로 해결한다.

### 시스템 구성 (파이프라인)
```
raw scan → Scan processing → ┬ Sequential matching (5 Hz) → 속도 ẋ ─┐
                             └ Global matching (1 Hz) → 절대위치 x ─┼→ Kalman filter → x̂ (drift-free)
                                        ↕
                               Mapping (online, octree)
```
- **Scan processing**: 기체 근접점 제거(r < 0.395 m) → 자세(roll/pitch) 투영 → 지면점 제거(p_z < h_min)
  → 노이즈 제거(반경 내 이웃 <2개인 점) → 비행영역 외 점 제거. 극좌표 → 직교좌표 변환 및 자세/고도 보상 포함.
- **Sequential matching**: 연속 스캔 Z_t, Z_{t−1} 정합 → 변위를 **적분하지 않고** 타임스탬프로 나눠
  속도 ẋ_t = Δx/Δt, 각속도 φ̇ = φ/Δt 산출. (변위 적분 시의 drift 누적을 필터 설계 차원에서 회피)
- **Global matching**: 현재 스캔 Z_t를 맵 M_{t−1}에 정합 → 절대 위치. 1 Hz로 충분한 이유:
  sequential 드리프트가 1초 내에는 무시 가능하므로 더 자주 돌려도 이득 없이 CPU만 낭비.
- **Kalman filter**: 축별 독립 3-상태 모델 (x, ẋ, ẍ). 속도는 measurement로 velocity 상태를,
  절대 위치는 position 상태를 갱신. 위치 전파(적분)는 필터의 등가속도 운동모델 prediction이 담당.
  yaw는 (φ, φ̇) 2-상태, 고도는 IMU 가속도 예측 + 하향 거리계 보정의 별도 필터.

---

## 3. Method — Scan Matching (Section IV)

### 3.1 왜 feature extraction을 쓰지 않는가 ★

본 survey 관점에서 가장 중요한 대목. 저자들의 논증:

| | 고밀도 스캔 (multi-channel 3D LiDAR) | 저밀도 스캔 (본 논문, <200점) |
|---|---|---|
| 정보 특성 | 중복(redundant) → feature로 압축 이득 | 이미 희소 → 버릴 점이 없음 |
| Geometric feature | edge/plane 등 추출 가능 (LOAM 계열) | **신뢰할 만한 양의 feature 추출 불가** |
| Feature 모호성 | 객체 디테일 충분 → unambiguous | 디테일 부족 → **feature 자체가 모호** |
| 결론 | feature 기반 correspondence 유리 | **point-to-point가 유일하게 성립** |

> point-to-point 대응은 어떤 점도 폐기하지 않으므로 공간 정보 손실이 없고,
> feature 기반 방법이 실패하는 조건에서도 변환 추정이 가능하다.

즉 이 논문이 "사용하는 feature"는 **전처리를 통과한 raw point 전체**이며, 정밀도는 feature의
표현력이 아니라 아래의 correspondence 설계에서 확보한다.

### 3.2 Correspondence 설계 (표준 ICP의 5가지 개조)

**(1) Linear interpolation — 가상 대응점**

<img width="581" height="432" alt="image" src="https://github.com/user-attachments/assets/172284c9-4a2f-421c-bd52-b1633b9b649c" />


최근접점 q_i와 그 이웃 q_a를 잇는 선분 위 투영점 q_v를 대응점으로 사용 (point-to-line segment).
희소 샘플링에 의한 discretization 오차를 표면 연속성 가정으로 보간. → **ablation에서 최대 단일 기여**
(global ATE 0.363 → 0.118).

**(2) IMRP — 회전 전용 대응**
유클리드 거리 metric은 병진은 빨리 수렴시키지만 회전 정보가 없어 회전 수렴이 느림(회전 오차를
병진으로 보상하려는 경향). 스캔의 native 표현인 **극좌표**에서, 각도창 [φ_j−B_ω, φ_j+B_ω] 내
range 차 ‖r_j−r_i‖² 최소점을 대응으로 탐색. 각도창은 B_ω(t)=B_ω(0)e^{−αt} (α=0.03)로
반복마다 지수적으로 축소해 후반 오대응 방지.

**(3) IDC — 이중 대응 결합** (Lu & Milios)
매 반복에서 병진(T_x, T_y)은 ICP 대응으로, 회전(ω)은 IMRP 대응으로 각각 추정해 하나의
T_iter로 합성 → T_total 갱신 → 다음 반복. 두 metric의 수렴 특성을 상보적으로 활용.

**(4) FRMSD — 비율 기반 outlier 기각**
센서 범위 이탈/occlusion으로 대응이 없는 점들을 처리. inlier 비율 f를 비용에 내장:

```
FRMSD = (1/f^λ) · sqrt( (1/(f·n)) · Σ ‖p − c(p)‖² ),  λ = 1.2 (2D 범용값)
```
거리순 상위 f 비율만 사용하되 1/f^λ 페널티로 f 축소를 억제 → inlier 비율과 정합 오차를 동시 최적화.

**(5) Weighting — 거리 기반 가중**
outlier 기각 후에도 먼 대응이 추정을 지배하는 문제 완화: w(d_i) = 1 − d_i / max(d_1..n).
가중 평균/공분산으로 닫힌형 해 (2D rigid) 계산:
ω = arctan((S_xy′−S_yx′)/(S_xx′+S_yy′)), T_x, T_y는 가중 평균에서 직접 산출.

**종료 조건**: FRMSD 반복간 감소량 ≈ 0 (지수 감소 가정) / FRMSD < 1 cm / **hard budget 50 ms**
(실시간성 우선 — 초과 시 오차 무관 중단).

### 3.3 Map 표현과 Global matching

- **Prior map 없음**: 맵은 비행 중 온라인 구축. 정합된 스캔을 절대좌표로 변환해 누적.
- **저장 구조**: octree (해상도 r = 0.2 m). 셀 중심 μ(o_j)에서 r 이상 떨어진 점만 삽입(중복 억제),
  질의 시 셀 중심들의 point cloud로 반환 → 사실상 voxel-downsampled point map.
- **통합 조건**: 마지막 업데이트 후 0.5 m 이상 이동 시에만 스캔 통합 (호버링 중 중복 누적 방지).
- **Submap crop**: 이전 추정 위치 x_{t−1} 중심 반경 **r_c = 1.2·r_max** 로 crop.
  목적 = ① correspondence 탐색 비용 절감, ② 동일 형상(코너 등)의 원거리 오매칭 원천 차단.
- **수직 슬라이스**: 맵은 높이별 단면이 다른 3D이므로, 현재 비행 고도 주변 슬라이스만 2D 정합에 사용.
- **한계**: loop closure/pose graph 최적화 없음 — 맵에 한 번 통합된 오차는 교정되지 않음
  (BALM류 lidar BA와 대비). 재정합 자체가 드리프트 억제 역할 (Stara Voda: sequential 0.79 m
  → global 0.38 m 종단 오차).

### 3.4 실험 결과 요약 (Chlumin 성당, total station GT)

| 누적 개조 | Seq. ATE [m] | Global ATE [m] | MME |
|---|---|---|---|
| 표준 ICP | 0.307 | 1.064 | 1.149 |
| + Noise filtering | 0.214 | 0.363 | 1.142 |
| + Interpolation | 0.194 | **0.118** | 0.223 |
| + Rotate | 0.187 | 0.120 | 0.268 |
| + IMRP | 0.183 | 0.108 | 0.199 |
| + Weighting | 0.177 | 0.104 | 0.278 |
| + Outliers (FRMSD) | **0.163** | **0.098** | 0.203 |

- 전처리(노이즈 제거)와 선형보간 두 단계가 개선의 대부분을 차지.
- 야외 직사광 실험(스캔당 평균 49점)에서도 안정화 성공 → 카메라 대비 조도 강건성 입증.
- DARPA SubT 광산 터널 등 실전 배치 사례 다수.

---

## 4. Takeaways (survey 관점)

1. **Feature 성립의 밀도 하한**: LOAM류 edge/plane primitive(③)나 descriptor(①④⑤)는 최소한의
   포인트 밀도·객체 디테일을 전제한다. <200점/scan 영역에서는 raw point 대응 + correspondence
   설계 정밀화가 유일한 선택지 — 센서 스펙(밀도/FoV/주사율)에 따른 기법 선택 경계를 규정하는 레퍼런스.
2. **Multi-rate 계층 구조의 변형**: LOAM(고주파 pose odometry + 저주파 mapping의 cascade)과 같은
   "고주파 relative + 저주파 absolute" 패턴을 공유하되, 고주파 출력을 pose가 아닌 **velocity**로
   정의해 odometry 모듈을 제거하고 그 역할을 KF 운동모델에 흡수 — 적분 드리프트를 필터 설계로 회피.
3. **모호성 처리 전략**: descriptor의 판별력이 아니라 **탐색 영역 제한(1.2·r_max crop)** 으로
   place ambiguity를 회피. 전역 재위치추정(kidnapped robot) 능력은 없음 — tracking-only 시스템.
4. **본 조사의 sequential global localization 관점**: 사전 맵 기반이 아니므로 엄밀한 global
   localization(기구축 맵 위 위치추정)과는 문제 설정이 다름. 다만 scan-to-map 정합의 correspondence
   설계(보간 대응, 회전 전용 metric, FRMSD, 거리 가중)는 primitive 기반 scan-to-map 정합의
   residual/가중치 설계에 직접 이식 가능한 요소.

## 5. 관련 비교

| | 본 논문 | LOAM-Livox | Pole (Schaefer) | Localising Faster |
|---|---|---|---|---|
| Feature | raw point (無추출) | edge/plane point | pole landmark | learned BEV feature |
| Map | online octree point | edge/plane map | pole map | NDT + GP |
| Prior map | ✗ | ✗ | ✓ | ✓ |
| Global reloc. | ✗ | ✗ | ✓ (PF) | ✓ (GP→MCL) |
| 대상 센서 | 2D, <200 pt | small-FoV 3D | 3D | 3D |

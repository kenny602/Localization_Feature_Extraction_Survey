# LiDAR Inertial Odometry Aided Robust LiDAR Localization System in Changing City Scenes

Ding, W., Hou, S., Gao, H., Wan, G., & Song, S. (2020). LiDAR inertial odometry aided robust LiDAR localization system in changing city scenes. In _2020 IEEE International Conference on Robotics and Automation (ICRA)_ (pp. 4322-4328).

> **분류 노트**: 본 연구는 sparse한 특징점을 추출하는 방식이 아니라, **cell(occupancy grid)의 점유 확률과 LiDAR intensity를 residual로 직접 최적화**하는 dense 정합 + 장기 localization 프레임워크다. 특징 표현 6분류와는 다른 축에 있어 별도 섹션(⑦ Dense cell-occupancy residual optimization)으로 둔다. 특징 기반 접근과 대비되는 **dense/direct 패러다임**의 대표 사례로 참조한다.

---

## Abstract

자율주행에서 환경 변화(재포장·웅덩이·눈·벽 신축/철거 등)는 online 측정 정합 기반 localization을 자주 실패시킨다. 본 논문은 **LiDAR-Inertial Odometry(LIO)** 기반의 dead-reckoning과 **전역 매칭(LGM)** 을 상보적으로 융합하여, 변화하는 도시 환경에서도 정확하고 매끄러운 위치추정을 유지하는 강건한 시스템을 제안한다.

LIO 성능을 높이기 위해 **occupancy grid 기반 LiDAR odometry에 관성(IMU) 정보와 LiDAR intensity cue를 통합**하고, **multi-resolution occupancy grid**로 coarse-to-fine 정합을 구현해 정밀도와 연산량을 균형 잡는다. odometry와 전역 매칭 결과는 **pose-graph 융합(MAP 추정)** 으로 결합한다. 또한 **맵의 어느 부분을 언제 갱신해야 하는지**를 알려주는 환경 변화 감지(ECD) 기법을 제안한다. Apollo-SouthBay와 내부 데이터셋에서 변화하는 도심 시나리오의 강건성·정확도를 검증했다.

---

## Introduction

정밀 localization은 자율주행의 핵심이지만, 동적으로 변하는 환경에서 구현이 어렵다. 재포장·웅덩이·눈더미 같은 특정 변화는 기존 기법으로 일부 극복됐으나, online 측정-맵 정합 기반 localization의 실패는 여전히 큰 난제다.

본 논문의 착안은 **map matching과 odometry가 상보적**이라는 점이다. 전역 매칭(LGM)은 전역 위치를 주지만 환경 변화/맵 오류 시 실패할 수 있고, LIO는 국소적으로 매끄러운 dead-reckoning을 주지만 드리프트가 누적된다. 둘을 pose-graph로 융합하면, 일시적 환경 변화나 맵 오류에 강건하면서도 일관된 전역 위치를 제공할 수 있다. 핵심 기여는 (1) 전역 매칭+국소 odometry를 적응적으로 융합하는 프레임워크, (2) LiDAR·관성·intensity·occupancy를 결합한 LIO, (3) 맵 갱신 시점/부위를 찾는 환경 변화 감지(ECD)다.

---

## Method

시스템은 **LIO(국소 dead-reckoning) + LGM(전역 매칭) + Pose-graph 융합 + ECD(변화 감지)** 로 구성된다.

### 1. LiDAR Inertial Odometry (LIO) — 핵심: occupancy+intensity residual

W. Hess의 occupancy-grid odometry(Cartographer 계열)를 확장한다. (i) **3D occupancy grid**로 6-DoF odometry를 구현하고, (ii) IMU pre-integration으로 모션 예측·왜곡 보정·프레임 간 제약을 더하며, (iii) 차선·노면 마커의 풍부한 정보를 위해 **LiDAR intensity cue**를 grid 정합에 결합하고, (iv) **multi-resolution**으로 coarse-to-fine 정합한다.

상태 $x^L_k=[R_k, t_k]$ 추정을 MAP 문제로 두고, 점군 측정의 likelihood를 두 residual의 합으로 정의한다. 점 $p_j$ 가 맞는 cell $s$ 의 점유 확률 $P(s)$ 와 intensity 통계 $(u_s, \sigma_s)$ 에 대해:

$$\mathrm{SSOP} = 1 - P(s), \qquad \mathrm{SSID} = \frac{u_s - I(p_j)}{\sigma_s}$$

- **SSOP(Sum of Squared Occupancy Probability)**: cell 점유 확률 비용. $P(s)$ 는 새 스캔을 submap에 누적하며 **binary Bayes filter(역측정모델·log-odds)** 로 갱신.
- **SSID(Sum of Squared Intensity Difference)**: cell intensity 평균 대비 점 intensity 차이(분산으로 정규화). 노면 텍스처 정보 제공.
- submap에서 확률·intensity 값은 **cubic interpolation**으로 얻고, 해상도별 가중치 $\sigma_{oi}, \sigma_{ri}$ 로 두 항을 조합. 최종은 비선형 최소제곱(Ceres, LM/GN)으로 최소화.

### 2. 전역 매칭 (LGM, LiDAR Global Matching)

전역 cue를 제공하는 모듈로, 구현은 Wan et al.(ICRA 2018)을 따른다. 사전 구축한 **2D BEV localization map**의 각 cell에 LiDAR **intensity(평균·분산)와 고도** 통계를 저장하고, online 스캔을 같은 grid로 투영해 **histogram filter**로 수평 (x, y) offset 후보들에 대한 정합 확률 분포를 계산한다. 분포의 봉우리가 전역 수평 위치가 되며, 고도 cue가 z(및 벽 신축 등 변화)에 대한 강건성을 높인다. 이 전역 추정이 pose-graph의 전역 factor로 들어간다.

### 3. Pose-graph 융합

국소(LIO)와 전역(LGM) cue를 하나의 MAP 추정으로 결합한다. 사후확률을 세 종류 factor로 인수분해한다: occupancy/intensity 정합 factor $r^O_{ks}$, IMU pre-integration factor $r^I_k$, 전역 매칭 factor $r^G_k$. 영평균 가우시안 가정하에 각 likelihood를 정의하고(Bayesian network), 비선형 최소제곱으로 효율적으로 푼다. LGM이 실패해도 LIO가 보완하여 전체 시스템을 보호한다.

### 4. 환경 변화 감지 (ECD)

LIO가 만든 submap을 지면에 투영해 사전 맵과 같은 2D 표현으로 만든 뒤, 대응 cell 쌍의 **intensity·고도 차이**를 비교한다.

$$z_s(r) = \frac{(u_s-u_m)^2(\sigma_s^2+\sigma_m^2)}{\sigma_s^2\sigma_m^2}, \qquad z_s(a) = (a_s-a_m)^2$$

변화 발생을 cell 단위 이진 상태로 보고 **binary Bayes filter**로 누적 추정하여(여러 차량이 같은 구역을 지나면 갱신), **맵의 어느 부분을 언제 갱신할지** 판단한다.

---

## 본 기법의 특징

1. 특징점 대신 **cell의 점유 확률(SSOP)과 intensity(SSID)를 residual로 직접 최적화**하는 dense 정합 odometry를 사용한다.

2. 3D occupancy grid + IMU pre-integration + intensity cue + multi-resolution(coarse-to-fine)으로 6-DoF LIO를 구성한다.

3. LIO(국소)와 LGM(전역, intensity+고도 histogram filter)을 pose-graph로 융합해 상호 실패를 보완한다.

4. submap과 사전 맵의 cell 단위 비교로 환경 변화를 감지(ECD)하여 맵 갱신 시점/부위를 결정한다.

---

## 본 기법의 장점

1. 전역 매칭과 odometry가 상보적으로 융합되어, 일시적 환경 변화·맵 오류·전역 매칭 실패에 강건하다(상용 자율주행 fleet에서 실검증).

2. intensity·고도 cue로 노면 텍스처와 수직 구조를 활용하여, 재포장·벽 신축/철거 등 변화에 견딘다.

3. 환경 변화 감지로 **사전 맵을 선택적으로 갱신**할 수 있어, 변화하는 도시에서 맵을 장기적으로 유지·재사용한다(장기 localization에 직접 부합).

---

## 본 기법의 단점

1. **사전 localization map(intensity·고도 BEV grid)** 과 IMU가 필요하며, 맵 구축·유지 인프라(대규모 데이터 수집)가 전제된다.

2. dense occupancy/intensity 정합이라 sparse 특징 대비 맵 용량이 크고, intensity는 센서·반사율·날씨에 민감하다.

3. 2D BEV 투영 기반 전역 매칭이라 수직 정보 활용이 제한적이며, 특징 기반 place recognition처럼 초기 추정 없는 전역 재인식(kidnapped)에는 부적합하다(국소 정합·추적 중심).

# LiLoc: Lifelong Localization using Adaptive Submap Joining and Egocentric Factor Graph

- **저자**: Yixin Fang, Yanyan Li, Kun Qian, Federico Tombari, Yue Wang, Gim Hee Lee
- **게재처**: IEEE ICRA 2025
- **분류**: 프레임워크(사전 맵 기반 지속 localization / lifelong). 단계별로 서로 다른 특징 표현을 혼용 —
  초기화는 **⑤ Projection-based**(극좌표 descriptor), scan-to-scan matching은 **point-to-plane**, scan-to-map matching은 **Voxel/Distribution-based**(NDT), 맵 표현은 **raw keyframe submap + pose graph**
- **코드**: https://github.com/Yixin-F/LiLoc

> **survey 관점 메모**: LiLoc은 corner line / planar surface 같은 기하 primitive를 새로 제안하는 특징 추출 논문이 **아니다**. 기여의 중심은 "prior map이 시간이 지남에 따라 달라지므로, **환경 변화로 인한 현재 스캔-지도 간 불일치를 고려해서 지도까지 갱신하는 위치인식**"라는 것에 있다. 그럼에도 본 조사에 포함하는 이유는, (1) LSMCL과 동일한 문제 설정(사전 맵 위 **지속적** global localization)을 factor graph로 푸는 대표 사례이고, (2) 단계별로 어떤 LiDAR 표현을 선택했는지가 특징 표현 선택의 좋은 비교 축이 되기 때문이다. 단, ILM에서 사전 맵을 갱신·확장하므로 순수 localization이 아니라 **lifelong(multi-session) SLAM과의 경계**에 걸쳐 있다.

<img width="1600" height="900" alt="image" src="https://github.com/user-attachments/assets/7d865108-0c7b-4cbc-83e0-762aed9117b6" />

---

## Abstract

LiLoc은 LiDAR 기반 **lifelong localization** 프레임워크로, 하나의 **central session**(prior map A)을 유지·활용하면서 **subsidiary session**(현재 online session B, C)의 pose를 연속 추정한다. (1) **Adaptive submap joining**으로 central session의 prior submap(keyframe + pose)을 생성·선택·갱신하고, (2) 세션 시작 시 **vertical recognition + ICP**의 coarse-to-fine 초기화로 전역 좌표계 초기 pose를 잡으며, (3) **Egocentric Factor Graph(EFG)** 가 IMU preintegration·LiDAR odometry·scan match factor를 공동 최적화한다. 특히 scan match factor는 **propagation model**로 prior 정합 결과를 인접 prior pose node들에 **정합 fitness 기반 노이즈 가중 edge**로 분배해, prior 제약의 영향력은 유지하되 그 불확실성이 정확도를 해치지 않게 한다. (4) **overlap 기반 mode switching**으로 relocalization(RLM)과 incremental localization(ILM)을 유연하게 전환한다. NCLT·M2DGR·자체 데이터셋에서 SOTA 대비 정확한 성능을 보였다.

---

## Introduction

Lifelong localization은 하나의 central session을 지속적으로 활용·갱신하면서 다른 subsidiary session들의 장기 위치추정을 달성하는 multi-session 문제로 볼 수 있다. 기존 접근의 한계:

- **GNSS/INS**: 신호 차폐 환경에서 실시간 위치를 보장하지 못한다.
- **Map-based localization**(HDL-Loc, Fastlio-Loc 등): 현재 스캔을 사전 맵에 정합한 relative pose로 odometry를 직접 보정하지만, **환경 변화로 인한 현재 스캔–사전 맵 불일치**를 고려하지 않아 relative pose의 신뢰도가 떨어지고 drift를 유발한다.
- **Factor graph 기반**(Block-Loc, range-inertial 등): scan matching 결과를 factor로 넣어 개선했으나, **정합(registration) 자체의 오차를 무시**해 prior 제약에 과의존할 위험이 있다.

LiLoc의 핵심 주장: 사전 맵 제약은 (a) 필요한 곳에서만(모드 전환), (b) 정합 품질로 가중하여(propagation model), (c) odometry·IMU와 **공동 최적화(JFGO)** 로 써야 한다.

---

## Method

<img width="1161" height="497" alt="image" src="https://github.com/user-attachments/assets/dad5e321-4ec8-4c4c-969b-d2471bcb7f7a" />


전체 파이프라인: **adaptive submap joining → coarse-to-fine pose initialization → EFG(IMU + LiDAR odometry + scan match factor) → mode switching(RLM/ILM)**.

Problem formulation: 세션 $S$는 pose graph $\mathcal{G}=(\mathcal{V},\mathcal{E})$와 keyframe DB $\mathcal{F}$를 가진다. 초기 pose $T^b_{init}$ 확보 후, 이후 추정은 joint factor graph $\{\mathcal{G}^a,\mathcal{G}^b\}$의 MAP 문제다:

$$\arg\min_{\mathcal{G}^b} \sum_{i\in\mathcal{E}^{I_b}} e^{I_b}_i + \sum_{j\in\mathcal{E}^{L_b}} e^{L_b}_j + \sum_{k\in\mathcal{E}^{a}} W(e^{a}_k)$$

IMU preintegration edge, LiDAR odometry edge, 그리고 **가중된($W$) prior scan match edge**의 합이다.

### 1. Adaptive Submap Joining — 맵 표현: raw keyframe submap

사전 맵은 기하 특징이 아니라 **keyframe 점군 묶음(submap) + pose graph**로 표현된다. odometry pose의 SE(3) 거리 기반으로, keyframe이 $N_s=20$개 쌓이거나 이동 거리가 $h_p=20$ m를 넘으면 새 submap $M_k = \bigcup_{i\in(j,k]} T_i \cdot F_i$ 를 생성한다. prior pose들과 submap centroid들은 각각 kd-tree($Tree_p$, $Tree_m$)로 관리한다. 모드에 따라 RLM이면 최근접 prior submap을 **선택**, ILM이면 새 submap을 생성해 DB를 **갱신**한다.

> **특징 관점**: 이 단계에는 특징 추출이 없다. "무엇을 저장할까"에 대한 답이 **raw 점군 그대로 + 공간 인덱스(kd-tree)** 다. 메모리 절감은 특징 압축이 아니라 **submap 단위 선택**(전체 맵을 항상 들고 있지 않음)으로 달성한다 — Block-Loc의 block map과 같은 계열의 해법.

### 2. Coarse-to-Fine Pose Initialization — 특징: 극좌표 projection descriptor

세션 시작 시 1회 수행. 대략적 초기 pose $T^b_{co}$로 관련 prior submap $M^a_{co}$를 고른 뒤:

1. 첫 keyframe $F^b_{co}$의 각 점 $\delta_k=\{x_k,y_k,z_k\}$을 극좌표 $\sigma_k=\{\rho_k,\theta_k,z_k\}$로 변환 ($\rho_k=\sqrt{x_k^2+y_k^2}$, $\theta_k=\arctan(y_k/x_k)$).
2. 방위각 $N_a=60$ sector × 반경 $N_r=20$ ring으로 균등 분할. 각 segment는 **법선 방향 $\vec{n}_{ij}$와 최대 높이 $\max z_k$** 로 기술된다:

<img width="807" height="80" alt="image" src="https://github.com/user-attachments/assets/ae8e6bdd-c5c6-43bc-8885-28e625c10a84" />

3. prior submap의 모든 keyframe에도 같은 처리를 해 후보 집합 $V^a_{co}$를 만들고, **행렬 차이(XOR) 최소** 기준으로 최적 keyframe을 고른다: $F^a_{best}=\min_{F^a_t\in M^a_{co}} \text{XOR}(F^b_{co}, F^a_t)$.
4. **ICP**로 정밀 초기 pose $T^b_{init}$을 얻는다.

> **특징 관점**: Scan Context 계열의 **⑤ 투영 기반 descriptor**다. 다만 각 bin에 occupancy나 최대 높이만 넣는 원조와 달리 **법선 방향 + 최대 높이**를 함께 넣어 수직 구조(vertical) 정보를 강화했다("vertical recognition"). FPFH+RANSAC/Teaser/Quatro(수십 초)보다 훨씬 빠르고(1.63 s) BBS++와 비슷한 정확도(xyz 0.07 m)를 달성 — **coarse는 값싼 projection descriptor, fine은 ICP**라는 역할 분담이다.

### 3. Egocentric Factor Graph (EFG)

**(1) IMU preintegration factor** — 표준 preintegration으로 두 시점 사이 상대 회전·위치·속도 제약 $e^{I_b}_{ij}$를 구성하고, IMU bias를 함께 최적화한다.

**(2) LiDAR odometry factor — 특징: raw 점 + 국소 평면 patch (ikd-tree)**

새 스캔은 IMU preintegration으로 왜곡 보정 후 **ikd-tree**로 증분 관리되는 local map에 정합된다. 새 keyframe $F^b_k$의 각 점에 대해 ikd-tree에서 **최근접 5점**을 찾아 그 점들이 이루는 국소 평면까지의 **point-to-plane 거리**를 roughness 가중치 $\omega_i$로 가중해 LM으로 최소화한다:

$$T^b_k = \min_{T^b_k}\sum_{i\in F^b_k}\omega_i\,\frac{\big|(\mathbf{p}_{k,i}-\mathbf{p}_{k-1,u})\cdot\big((\mathbf{p}_{k-1,u}-\mathbf{p}_{k-1,v})\times(\mathbf{p}_{k-1,u}-\mathbf{p}_{k-1,w})\big)\big|}{|(\mathbf{p}_{k-1,u}-\mathbf{p}_{k-1,v})\times(\mathbf{p}_{k-1,u}-\mathbf{p}_{k-1,w})|}$$

odometry factor는 연속 pose 간 상대변환 $e^{L_b}=(T^b_{k-1})^{\top}\cdot T^b_k$ 로 graph에 들어간다.

> **특징 관점**: FAST-LIO2 계열의 접근 — 명시적 edge/plane **분류 없이**, 모든 점을 최근접 5점의 국소 평면에 정합한다(스캔 순서 곡률 기반 LOAM식 추출 없음). 즉 "특징 추출을 생략하고 raw 점 + k-NN 평면 피팅"을 택했으며, 이는 Loam-Livox가 매 프레임 하던 5-NN + 기하 재구성과 같은 구조를 ikd-tree로 빠르게 만든 것이다. 엄밀히 scan-to-scan이 아니라 **scan-to-local-map**임에 유의.

**(3) Scan match factor — 특징: NDT cell 분포 + propagation model**

RLM에서만 활성화. 현재 스캔을 선택된 prior submap에 **NDT 정합**해 전역 변환 $T^a_r$을 얻는다. 기존 방법들처럼 이 relative pose를 odometry edge로 직접 넣지 않고, **propagation model**로 분배한다:

1. $Tree_p$에서 최근접 prior node $N_d=3$개 $\{T^a_{n1},T^a_{n2},T^a_{n3}\}$를 찾는다.
2. 현재 스캔 $F^b_k$와 각 prior keyframe 사이의 **정합 fitness score $f^i_b$** 를 계산한다.
3. 각 prior node로의 edge를 fitness로 가중한 노이즈($\Omega_o f^i_b$)로 추가한다:

$$e^a = \bigcup_{i\in\{n1,n2,n3\}} \big\|(T^a_i)^{-1}T^a_r\big\|_{\Omega_o f^i_b}$$

joint graph $\{\mathcal{G}^a,\mathcal{G}^b\}$의 균형을 위해 anchor node $Q^a$에 $Q^b$보다 작은 prior noise를 준다.

> **특징 관점**: prior 제약 정합에는 **② 분포 기반(NDT)** 표현을 쓴다. prior submap을 cell별 가우시안(평균+공분산)으로 만들어두면 온라인에서는 cell lookup + likelihood 평가만 하면 되고, 공분산의 고유구조가 평면/edge 기하를 암묵적으로 담아 point-to-plane/line처럼 이방성 가중 정합이 된다. **핵심 기여는 그 다음**: 정합 결과를 그대로 믿지 않고 **fitness로 노이즈를 조절해 "사전 맵을 얼마나 믿을지"를 graph 수준에서 결정**한다. 환경 변화로 현재 스캔과 사전 맵이 어긋난 곳에서는 fitness가 나빠져 prior 제약이 자동으로 약해진다 — 동적/장기 변화 강건성을 특징이 아니라 **가중치로** 확보하는 설계.

**(4) Marginalization** — 모드별 가변 sliding window(RLM $N_m=5$ / ILM $N_m=10$ keyframe)로 오래된 변수를 Schur complement로 marginalize해 실시간성을 유지한다.

### 4. Mode Switching — RLM / ILM

현재 pose $P^b_k$ 주위 한 변 $2l$의 정사각 영역과 최근접 prior submap의 xy 경계 $B$의 겹침으로 판단한다:

$$O(T^b_k) = [P^b_k(x)\pm l,\ P^b_k(y)\pm l]\ \cap\ [B(x),\ B(y)]$$

$O(T^b_k)\ge h_o$ ($h_o=0.7$)이면 **RLM**(prior 커버리지 안 → scan match factor 추가), 미만이면 **ILM**(커버리지 밖 → odometry만으로 추정하며 새 submap을 생성해 prior DB 확장).

> **주의**: RLM/ILM은 "scan-to-map vs scan-to-scan"의 구분이 **아니다**. IMU·LiDAR odometry factor는 두 모드 모두 항상 동작하며, 모드가 결정하는 것은 (a) prior scan match factor를 켤지, (b) 사전 맵을 갱신할지 뿐이다.

---

## 실험 요약

- **초기화**: FPFH+RANSAC/Teaser/Quatro 대비 압도적으로 빠르고(1.63 s vs 31~56 s) 정확(xyz 0.07 m); BBS++(0.05 m, 0.87 s)와 대등.
- **Single-session**: ILM(odometry) 모드는 FAST-LIO2와 대등, RLM은 HDL-Loc·Fastlio-Loc·Block-Loc 대비 xyz/rpy 정확도 우위. 대신 JFGO로 시간이 증가(RLM 약 37~44 ms).
- **Multi-session**: HDL-Loc·Block-Loc은 ILM 전환 불가로 대부분 완전 실패(×), LiLoc은 모드 전환 덕에 안정적. prior node 수를 줄이면 시간은 줄지만 scan match factor의 구속력이 약해져 실패 위험 증가.

---

## 핵심 정리 — Scan Matching vs Prior Map Matching의 LiDAR feature 관점

지속적 global localization에서 LiLoc의 pose 추정은 두 개의 정합으로 이루어지며, **두 정합이 서로 다른 feature 표현을 쓴다.** 이 선택의 이유를 이해하는 것이 이 논문의 특징 관점 핵심이다.

### 1. Scan Matching (LiDAR odometry factor) — "raw 점 + 즉석 기하"

- **정합 대상**: 자기 세션이 방금 누적한 **local map** (ikd-tree로 증분 관리).
- **특징 표현**: **추출·분류가 없다.** 왜곡 보정된 raw 점 전부가 후보이고, 맵 쪽에도 기하가 저장되지 않는다. 정합 시점에 각 점마다 ikd-tree **최근접 5점을 찾아 국소 평면을 즉석에서 피팅**하고, point-to-plane 거리를 roughness 가중 $\omega_i$로 최소화한다 (Eq. 6, LM).
- **기하 계산의 위치**: 온라인, 매 점, 매 프레임. Loam-Livox의 "5-NN + PCA 재구성"과 동일한 구조를 FAST-LIO2식 ikd-tree로 빠르게 만든 것이다. LOAM 원조와 달리 **edge feature는 아예 쓰지 않고 plane만** 쓴다 — 회전형 라이다의 dense한 점에서는 모든 점을 평면 정합에 넣는 것이 특징 선별보다 낫다는 FAST-LIO2 계열의 판단을 따른 것.
- **왜 raw로 충분한가**: local map은 직전 keyframe들로 방금 만들어졌으므로 **현재 스캔과의 일치가 구조적으로 보장**된다. 환경 변화·동적 불일치를 걱정할 필요가 없으니, 표현을 압축하거나 신뢰도를 따질 이유가 없고, 순수하게 빠른 상대 추정만 하면 된다.
- **한계**: 상대 추정이므로 drift가 누적된다. 이를 보정하는 것이 아래의 prior map matching이다.

### 2. Prior Map Matching (scan match factor) — "분포 통계 + 신뢰도 가중"

- **정합 대상**: central session의 **prior submap** (kd-tree로 선택된 것 하나).
- **특징 표현**: 선택된 prior submap을 **NDT cell(평균 $\boldsymbol{\mu}$ + 공분산 $\boldsymbol{\Sigma}$의 가우시안)** 로 표현해 현재 스캔을 정합한다. corner/plane의 명시적 분류는 없지만, 공분산의 고유구조가 국소 기하를 암묵적으로 담는다 — cell이 평면이면 $\lambda_3$이 작아 법선 방향 이탈만 강하게 penalize하는 soft point-to-plane으로, 선이면 수직 두 방향을 penalize하는 point-to-line으로 동작한다(Mahalanobis 거리의 이방성 가중). 온라인 비용은 점당 cell lookup + likelihood 평가로, 매 iteration k-NN이 필요한 ICP보다 예측 가능하다.
- **왜 raw 정합으로 안 되는가**: prior map은 **다른 시점에 만들어진 맵**이라 환경 변화로 현재 스캔과 어긋날 수 있고, 규모도 크다. 그래서 (a) cell 통계라는 **집약 표현**으로 정합하고, (b) 정합 결과 $T^a_r$을 그대로 pose에 반영하지 않는다.
- **신뢰도 가중(propagation model)**: $T^a_r$을 최근접 prior node 3개로의 edge로 분배하되, 각 edge의 노이즈를 **정합 fitness $f^i_b$로 스케일**한다(Eq. 8). 현재 스캔과 prior map이 어긋난 곳에서는 fitness가 나빠져 prior 제약이 자동으로 약해진다. 즉 **동적/장기 변화 강건성을 특징 추출이 아니라 factor 가중치로 확보**한다.

### 3. 대비 요약

| | **Scan matching** (odometry) | **Prior map matching** (scan match factor) |
|---|---|---|
| 정합 대상 | 자기 세션 local map (ikd-tree) | central session prior submap |
| 특징 표현 | raw 점 (기하 저장 없음) | NDT cell 가우시안 (통계 집약) |
| 기하 계산 시점 | 온라인 — 매 점 5-NN 평면 즉석 피팅 | 정합 시 cell 통계 구성 후 lookup |
| residual 형태 | 명시적 point-to-plane 거리 (roughness 가중) | Mahalanobis 거리 (공분산이 plane/line을 암묵 인코딩) |
| graph 반영 | 연속 pose 간 상대변환 edge (Eq. 7) | fitness 가중 edge를 인접 prior node 3개에 분배 (Eq. 8) |
| 신뢰 가정 | local map = 방금 생성 → 무조건 신뢰 | prior map = 과거 생성 → **fitness로 조건부 신뢰** |
| 동작 조건 | 항상 (양 모드) | RLM에서만 |
| 실패 모드 | drift 누적 | 환경 변화 시 fitness 저하 → 제약 자동 약화 |

### 4. 이 선택이 말해주는 것

두 정합의 표현이 갈리는 이유는 결국 **"맵을 얼마나 신뢰할 수 있는가"** 하나로 수렴한다. 신뢰 가능한 맵(방금 만든 local map)에는 raw 점 + 즉석 기하로 충분하고, 신뢰가 조건부인 맵(과거의 prior map)에는 집약 통계 표현 + 신뢰도 가중이 필요하다. 주목할 점은 LiLoc이 **어느 쪽에서도 명시적 corner/plane 분류를 하지 않는다**는 것이다 — 특징 추출을 없앤 것이 아니라, 기하 정보의 계산·저장 위치를 옮겼다. scan matching에서는 **정합 시점의 즉석 피팅**으로, prior map matching에서는 **cell 공분산 속**으로. 특징을 명시적으로 분류·저장하는 접근(LSMCL의 corner line/planar surface, LOAM의 edge/plane)과 비교하면, LiLoc은 분류 오류의 위험을 지지 않는 대신 raw keyframe 저장(무거운 맵)과 매 정합의 통계 계산 비용을 부담하는 트레이드오프를 택한 것이다.

---

## LSMCL과의 대응 관계 (지속적 global localization 관점)

| 역할 | LSMCL | LiLoc |
|---|---|---|
| 문제 설정 | 고정된 정적 맵 위 지속 6-DoF localization | central session 사전 맵 위 지속 localization (+ILM에서 맵 확장) |
| 맵 표현 | **정적 기하 특징 맵**(corner line $\mathbf{e}_k$ / planar surface $\mathbf{s}_k$) + 2D grid map | **raw keyframe submap** + pose graph (기하 압축 없음) |
| 초기화 | 2D grid map 위 **particle filter** 수렴 → 3D clone | 극좌표 **projection descriptor**(vertical recognition) + ICP |
| 지역 추정 (항상 동작) | scan matching (직전 프레임 특징 대비) | **LiDAR odometry factor** (ikd-tree scan-to-local-map) + IMU |
| 전역 보정 (사전 지식 활용) | map matching (누적 특징 맵 대비) — 결과를 직접 사용 | **scan match factor** (NDT 정합) — fitness 가중 edge로 공동 최적화 |
| 사전 지식 신뢰도 처리 | 맵 자체를 정적으로 정제(LSM)해 신뢰 확보 | 정합 fitness로 **factor 가중치를 조절**해 신뢰 확보 |
| RLM/ILM 대응물 | 없음 (scan/map matching이 한 파이프라인에 공존) | overlap 기반 배타적 모드 전환 |

두 시스템은 "동적/장기 변화에 대한 강건성"을 정반대 위치에서 확보한다. **LSMCL은 맵 쪽**(동적 제거 후 정적 특징만 저장), **LiLoc은 정합 신뢰도 쪽**(사전 맵은 그대로 두고 어긋나는 곳의 제약을 약화)이다.

# LOAM 계열의 Edge / Plane 기하 특징: Loam-Livox & BALM

> **분류**: ③ Geometric Primitive (edge/corner · plane/surface)
> LOAM 계열 두 논문을 **"localization에 쓰는 edge/plane 특징을 (1) 어떻게 뽑고, (2) 맵에 어떻게 표현하며, (3) 어떻게 정합하는가"** 관점에서 함께 정리한다.
> - **Loam-Livox** — 특징 *추출(front-end)* 과 *매 프레임 정합(per-frame data association)* 을 소형 FoV·비반복 스캔 라이다에 맞게 설계.
> - **BALM** — 같은 edge/plane 특징을 *voxel 단위 primitive로 표현* 하여, 매 프레임 기하를 다시 피팅하지 않고 정합하도록 재구성.

**대상 논문**

- Lin, J., & Zhang, F. (2020). *Loam livox: A fast, robust, high-precision LiDAR odometry and mapping package for LiDARs of small FoV.* In Proc. IEEE ICRA (pp. 3126–3131).
- Liu, Z., & Zhang, F. (2021). *BALM: Bundle adjustment for lidar mapping.* IEEE Robotics and Automation Letters, 6(2), 3184–3191.

---

## 왜 두 논문을 함께 보는가 (Survey 관점)

LOAM 계열은 원시 점 전체 대신 **edge(=corner/line)** 와 **plane(=surface)** 두 종류의 기하 특징만 뽑아 정합에 쓴다. 이 축에서 두 논문은 정확히 상보적이다.

- **Loam-Livox**는 "특징을 **어떻게 뽑는가**"(front-end point selection + edge/plane 추출)와, 맵에 기하를 미리 정의해두지 않고 **매 프레임 KD-tree 최근접 + PCA로 기하를 즉석에서 재구성**해 정합하는 방식을 다룬다.
- **BALM**은 반대로, 같은 edge/plane 특징을 **voxel마다 하나의 primitive(중심점 + 방향/법선)로 미리 표현**해두어 매 프레임 재피팅을 없애고, 정합 비용을 **고유값 최소화**로 재정의한다.

즉 **"특징 기하를 매 프레임 다시 피팅할 것인가(Loam-Livox) vs. 맵에 primitive로 미리 표현해둘 것인가(BALM)"** 가 두 논문을 가르는 핵심 축이며, 이 축은 곧 사전 맵 기반 localization에서 온라인 정합 비용을 좌우한다.

---

# Part A. Loam-Livox

- **저자**: Jiarong Lin, Fu Zhang (HKU)
- **게재처**: IEEE ICRA 2020
- **핵심**: 소형 FoV·비반복 스캔(solid-state) 라이다용 LOAM. front-end에서 라이다의 저수준 물리 특성을 반영한 점 선택 + edge/plane 추출, 20 Hz scan-to-map 정합.

## Abstract

Livox MID-40 같은 solid-state 라이다는 FoV가 좁고(38.4°) 스캔 패턴이 비반복적이며 in-frame 모션블러가 크다. Loam-Livox는 이런 라이다에서 안정적으로 동작하는 완결형 LOAM으로, IMU·GPS·카메라 없이 point cloud를 지역 맵에 정합해 실시간(20 Hz) odometry·mapping을 수행한다. 좁은 FoV에서의 feature 추출·선택, 강건한 outlier 제거, 동적 객체 필터링, 모션 왜곡 보정을 함께 다루며, 저수준 물리 특성(레이저 스폿 크기, 신호대잡음비 등)을 front-end에 반영해 정확도·강건성을 높였다.

## Introduction

기존 LOAM은 대부분 회전형(spinning) 라이다를 전제한다. 회전형은 여러 레이저가 수직으로 쌓여 규칙적인 ring을 그리므로 한 ring 위 점들의 depth 차이로 corner를 쉽게 계산할 수 있다. 반면 solid-state 라이다는 (1) **FoV가 좁아** 한 프레임에서 뽑히는 feature 수가 적어 정합이 degenerate되기 쉽고, (2) **스캔이 비반복적**(로제트 패턴)이라 규칙적 ring 가정을 못 쓰며, (3) 단일 레이저의 연속 스캔이라 **모션블러**가 심하다. Loam-Livox는 이 세 문제를 front-end 설계로 정면 대응한다.
<img width="718" height="302" alt="image" src="https://github.com/user-attachments/assets/230e91cb-6576-4244-a9f1-1850950b9d8a" />



## Method

### 1. 좋은 점 선택 (Point Selection)

각 점 $\mathbf{p}=[x,y,z]$(FLU 좌표)에 대해 저수준 물리량을 계산한다.

$$D(\mathbf{p}) = \sqrt{x^2+y^2+z^2}, \qquad \Phi(\mathbf{p}) = \tan^{-1}\!\sqrt{\tfrac{y^2+z^2}{x^2}}, \qquad I(\mathbf{p}) = \frac{R}{D(\mathbf{p})^2}$$

여기서 $D$는 depth, $\Phi$는 레이저 편향각(deflection angle), $I$는 intensity(반사율 $R$ 기준). 입사각(incident angle) $\theta$는 레이저와 국소 평면 사이 각이다.

$$\theta(\mathbf{p}_b) = \cos^{-1}\!\left(\frac{(\mathbf{p}_a-\mathbf{p}_c)\cdot \mathbf{p}_b}{|\mathbf{p}_a-\mathbf{p}_c|\,|\mathbf{p}_b|}\right)$$

정확도를 위해 다음 점들을 제거한다: (1) **FoV 가장자리**($\Phi$가 큰 영역 — 스캔 궤적 곡률이 커서 feature가 불안정), (2) **intensity가 너무 크거나 작은 점**(포화/왜곡 또는 낮은 SNR로 ranging 정확도 저하), (3) **입사각이 0 또는 $\pi$에 가까운 점**(레이저 스폿이 길게 늘어져 측정거리가 특정 점이 아닌 영역 평균이 됨), (4) 가려짐(occlusion) 경계 등 신뢰도가 낮은 점.
<img width="578" height="287" alt="image" src="https://github.com/user-attachments/assets/f206fbc2-2580-43f3-9b7e-c2ecc72f6ea8" />


### 2. Edge / Plane 특징 추출

살아남은 점에 대해, **시간적 scan order로 인접한 주변 점들과의 curvature**를 계산해 edge/plane을 나눈다(LOAM식 곡률: 국소적으로 곡률이 크면 edge, 작으면 plane).

좁은 FoV로 인해 기하 feature 수가 부족해지는 문제를 보완하기 위해, **intensity 값을 함께 활용**한다. 주변 점 대비 **intensity가 급격히 달라지는 점을 corner feature로 추가 추출**한다. 이는 긴 복도나 평평한 벽면처럼 기하만으로는 **degeneracy**가 발생하기 쉬운 곳에서 제약을 보강해준다.



### 3. 특징 정합 — 매 프레임 데이터 연관 (Data Association)

Loam-Livox는 **맵 위에 geometric feature를 미리 정의해두지 않는다.** 대신 매 프레임 기하를 즉석에서 재구성한다.

1. 현재 스캔의 feature 점 $\mathbf{p}_l$을 추정 pose로 전역 좌표계에 투영: $\mathbf{p}_w = \mathbf{R}_k \mathbf{p}_l + \mathbf{t}_k$.
2. 맵의 해당 feature 집합에서 **최근접 5개 점을 KD-tree로 탐색**(맵 KD-tree는 별도 병렬 스레드가 미리 구축).
3. 그 5개 점을 **PCA(공분산 $\Sigma$의 고유값 $\lambda_1\!\ge\!\lambda_2\!\ge\!\lambda_3$)** 해서 실제로 선/면을 이루는지 판정하고 residual을 만든다.

- **Edge-to-edge**: 가장 큰 고유값이 두 번째보다 3배 이상 크면($\lambda_1 \ge 3\lambda_2$) 5개 점이 **선**을 이룬다고 보고, 점–선 거리를 residual로 쓴다.

$$r_{p2e}(\mathbf{p}_w) = \frac{\left|(\mathbf{p}_w-\mathbf{p}_5)\times(\mathbf{p}_w-\mathbf{p}_1)\right|}{|\mathbf{p}_5-\mathbf{p}_1|}$$

- **Plane-to-plane**: 가장 작은 고유값이 두 번째보다 1/3 이하이면($\lambda_3 \le \tfrac{1}{3}\lambda_2$) **면**을 이룬다고 보고, 점–면 거리를 residual로 쓴다.

$$r_{p2p}(\mathbf{p}_w) = \frac{(\mathbf{p}_w-\mathbf{p}_1)^{\top}\big((\mathbf{p}_3-\mathbf{p}_5)\times(\mathbf{p}_3-\mathbf{p}_1)\big)}{|(\mathbf{p}_3-\mathbf{p}_5)\times(\mathbf{p}_3-\mathbf{p}_1)|}$$

> **핵심**: 특징의 기하(선/면 파라미터)가 맵에 저장돼 있지 않으므로, **매 프레임 KD-tree 5-NN + PCA로 기하를 다시 피팅**한다. 이 재피팅이 온라인 정합 비용의 큰 부분을 차지한다 — 바로 이 지점을 BALM(Part B)이 제거한다.

### 4. Scan-to-map 정합과 모션 보정

비반복 스캔이라 연속한 두 프레임이 서로 다른 영역을 스캔하므로, **scan-to-scan 매칭이 불안정**하다. 따라서 Loam-Livox는 scan-to-scan 없이 **20 Hz의 고주기 scan-to-map** 정합만 수행한다.

- **Piecewise 처리(모션블러 보정)**: 한 프레임을 scan order로 **3등분(sub-frame)** 해, 각 sub-frame을 **동일 전역 맵에 독립적으로 정합**한다. 각 sub-frame의 시간 구간이 원래의 1/3로 짧아져 in-frame 왜곡이 줄고, 3개의 서로 다른 pose를 얻는다. 각 pose는 해당 sub-frame의 odometry로 publish되어 **다음 스캔의 초기 pose**로 쓰인다.
- **강건 정합**: iterative pose optimization에서 매 iteration마다 최근접을 다시 찾아 residual을 만들고, **2회 iteration 후 상위 20% residual을 버린 뒤** 정합을 이어간다. 이것이 곧 **동적 객체/outlier 필터링** 역할을 한다.
- **병렬화**: sub-frame 정합과 KD-tree 구축을 멀티코어에 분산해, baseline 대비 2~3배 빠르게 실시간(20 Hz)을 달성한다.

---

# Part B. BALM

- **저자**: Zheng Liu, Fu Zhang (HKU)
- **게재처**: IEEE Robotics and Automation Letters (RA-L) 2021
- **핵심**: 라이다 BA를 "특징점–선/면 거리 최소화"로 정식화하되, **특징 파라미터를 closed-form으로 소거**해 pose만의 최적화로 축소. **adaptive voxelization**으로 특징을 voxel primitive로 표현.

## Abstract

시각 SLAM에서 널리 쓰이는 sliding-window local Bundle Adjustment(BA)는 drift를 크게 줄이지만, 라이다에서는 edge/plane 같은 **희소 특징점의 정확한 매칭이 불가능**해 잘 쓰이지 못했다. BALM은 라이다 BA를 **각 특징점에서 대응 edge/plane까지의 거리 최소화**로 정식화하고, 시각 BA와 달리 **특징(선/면) 파라미터를 해석적으로 풀어 최적화에서 제거**할 수 있음을 보인다. 그 결과 BA는 **scan pose에만 의존**하게 되어 최적화 차원이 크게 줄고, 대규모 dense edge/plane 특징을 다룰 수 있다. cost의 2차 도함수까지 closed-form으로 유도해 Gauss-Newton으로 가속하며, **adaptive voxelization**으로 특징 대응을 효율적으로 찾는다. LOAM back-end에 붙여 20 스캔을 10 Hz 수준으로 실시간 최적화하면서 drift를 낮춘다.

## Introduction

LOAM 계열은 새 스캔을 기존 맵에 **증분적으로 정합**하며 맵을 만들기 때문에 정합 오차가 누적돼 drift가 생긴다. 시각 SLAM은 여러 키프레임을 한꺼번에 최적화하는 local BA로 이를 완화하지만, 라이다는 두 스캔이 물체 위 **같은 점을 다시 스캔하지 않아** 점 단위 정확 매칭이 성립하지 않는다.

기존 라이다 BA/다중뷰 정합은 (a) 정확한 점/면 매칭을 요구하거나, (b) plane 파라미터와 pose를 **동시에** 최적화해 차원이 커지거나, (c) 원시 점군에서 plane을 **분할(segmentation)** 해야 하는 부담이 있었다. BALM은 (1) **edge와 plane 파라미터를 closed-form으로 소거**해 pose만의 저차원 문제로 만들고, (2) 2차 도함수를 유도해 빠르게 수렴시키며, (3) **분할 없이** adaptive voxelization으로 대응을 찾는다는 세 가지로 이를 해결한다.

## Method

### 1. 특징을 "점 매칭"이 아니라 "고유값 최소화"로

같은 특징(선 또는 면)에 속하는 여러 스캔의 점 $\mathbf{p}_i$가 있을 때, 그 평균과 공분산을 정의한다.

$$\bar{\mathbf{p}} = \frac{1}{N}\sum_{i=1}^{N}\mathbf{p}_i, \qquad \mathbf{A} = \frac{1}{N}\sum_{i=1}^{N}(\mathbf{p}_i-\bar{\mathbf{p}})(\mathbf{p}_i-\bar{\mathbf{p}})^{\top}$$

- **Plane**: 각 점에서 평면까지의 거리 제곱합을 최소화하면, 최적 법선 $\mathbf{n}^*=\mathbf{u}_3$(최소 고유값 고유벡터), 최적 점 $\mathbf{q}^*=\bar{\mathbf{p}}$일 때 cost는 $\mathbf{A}$의 **최소 고유값**이 된다.

$$\min_{\mathbf{n},\mathbf{q}} \frac{1}{N}\sum_i \big(\mathbf{n}^{\top}(\mathbf{p}_i-\mathbf{q})\big)^2 = \lambda_3(\mathbf{A})$$

- **Edge**: 각 점에서 직선까지의 거리 제곱합을 최소화하면, 방향 $\mathbf{n}^*=\mathbf{u}_1$(최대 고유값 고유벡터)일 때 cost는 나머지 두 고유값의 합이 된다.

$$\min_{\mathbf{n},\mathbf{q}} \frac{1}{N}\sum_i \big\|(\mathbf{I}-\mathbf{n}\mathbf{n}^{\top})(\mathbf{p}_i-\mathbf{q})\big\|^2 = \lambda_2(\mathbf{A})+\lambda_3(\mathbf{A})$$

### 2. 특징 파라미터의 closed-form 소거

위 두 식의 핵심은, 최적 특징 파라미터 $(\mathbf{n}^*,\mathbf{q}^*)$가 **BA를 풀기 전에 해석적으로 정해진다**는 것이다. 따라서 특징을 최적화 변수에서 빼버릴 수 있고, BA는 **오직 scan pose $\mathbf{T}$에 대한 고유값 최소화**로 축소된다.

$$\min_{\mathbf{T}} \ \lambda_k\big(\mathbf{p}(\mathbf{T})\big)$$

이는 "scan pose가 정해지면 point cloud(따라서 선/면 특징)도 정해진다"는 직관과 일치한다. 특징 파라미터를 제거하면 최적화 차원이 급감해 **대규모 dense 특징**을 다룰 수 있고, cost(고유값)의 gradient·Hessian을 closed-form으로 유도해 **Gauss-Newton으로 빠르게 수렴**한다.

### 3. Adaptive Voxelization — 특징을 voxel primitive로 표현

<img width="762" height="413" alt="image" src="https://github.com/user-attachments/assets/eb6af30a-e325-47f2-8661-59d435c84f5a" />

BA는 "같은 특징에 속하는 점들"을 모아야 한다. BALM은 대략적 초기 pose(예: LOAM odometry)가 있다고 가정하고 공간을 **재귀적으로 voxel화**한다.

- 기본 크기(예: 1 m)에서 시작해, 한 voxel 안의 모든 점이 하나의 평면/선을 이루면(공분산 고유값으로 판정) 그 voxel을 유지하고 그 안의 점들을 하나의 특징으로 본다.
- 아니면 8개 octant로 쪼개 최소 크기(예: 0.125 m)까지 반복한다.

결과적으로 **환경에 적응해 voxel 크기가 달라지는 voxel map**이 만들어지고, **각 voxel = 하나의 특징 = 하나의 cost 항**이 된다. edge용·plane용 두 개의 voxel map을 만들고, octree를 Hash table로 인덱싱해 깊이를 줄인다.

> **핵심**: 각 voxel에 대해 **중심점(center)과 법선/방향 벡터**를 미리 계산해 저장한다. 즉 특징이 "raw 점 집합"이 아니라 **primitive(중심 + 방향)** 로 표현된다. (큰 평면/긴 edge에서는 조기 종료로 full KD-tree보다 효율적이며, 곡면 등 비평면도 finer voxel + 큰 분산 허용으로 자연스럽게 확장된다.)

### 4. LOAM back-end 통합

BALM을 LOAM에 붙이면 feature extraction · odometry · map-refinement 3개 스레드로 동작한다. 정합 시 각 feature 점을 맵의 여러 최근접 점이 아니라 **가장 가까운 voxel(그 center로 대표)에 매칭**한다.

- Loam-Livox식이 매 점마다 **5개 최근접 점 + PCA**를 요구하는 데 비해, BALM은 **가장 가까운 voxel(primitive) 하나**만 찾으면 되므로 scan-to-map 탐색이 빨라진다.
- back-end local BA는 최근 20 스캔 sliding window를 **~10 Hz(거의 실시간)** 로 최적화하며 LOAM drift를 낮춘다.

---

# Part C. 비교 — localization feature 관점

| 축 | **Loam-Livox** | **BALM** |
|---|---|---|
| 특징 종류 | edge(corner/line) · plane(surface) | edge · plane (동일; 곡면까지 확장 가능) |
| 추출 근거 | scan order curvature **+ intensity 급변**(degeneracy 보완) | voxel 내 점 분포의 **고유값**(선/면 판정) |
| 맵에서의 표현 | **표현 없음** — 맵은 raw feature 점 집합(KD-tree) | **voxel primitive** — 중심점 + 법선/방향 (edge·plane 별도 voxel map) |
| 대응(매칭) 방식 | 매 프레임 **5-NN 탐색 + PCA**로 선/면 재구성 → 점-선/점-면 residual | 점을 **가장 가까운 voxel 하나**에 매칭 → 고유값 cost |
| 매 프레임 재피팅 | **필요**(KD-tree 5-NN PCA) | **불필요**(primitive 사전 계산) |
| 최적화 변수 | scan pose | scan pose (특징 파라미터 closed-form 소거) |
| 동적/outlier 강건성 | iteration 중 **상위 20% residual 제거** | window 누적 통계라 순간 노이즈에 둔감 |
| 계산 특징 | 20 Hz scan-to-map, sub-frame 3분할 병렬 | 20 스캔 local BA ~10 Hz, octree/Hash |
| 역할 | front-end odometry/mapping | back-end map refinement |

---

# Part D. 정리 및 시사점 — Geometric Primitive Map

두 논문을 localization feature 축에서 종합하면 하나의 방향이 드러난다.

- **Loam-Livox**는 맵에 기하를 저장하지 않고 **매 프레임 k-NN + PCA로 선/면을 재피팅**한다 — 유연하지만 온라인 정합 비용이 크고, 소형 FoV에서는 feature 수가 적어 degeneracy에 약하다(그래서 intensity를 보강).
- **BALM**은 같은 edge/plane을 **voxel마다 하나의 primitive(중심 + 법선/방향)로 미리 표현**하고, 특징 파라미터를 최적화에서 소거한다 — **매 프레임 재피팅이 사라지고**, window 누적 통계라 동적 노이즈에도 상대적으로 강건하다.

이 "primitive로 미리 표현" 아이디어는 **사전 맵 기반 localization**에 직접 적용된다. 정적 맵의 각 voxel을 raw 점 대신 **하나의 기하 primitive로 저장**하면 — surface는 `법선 = u₃`(최소 고유값 고유벡터), corner/edge는 `방향 = u₁`(최대 고유값 고유벡터), 그리고 잔차 지표(예: surface `λ₃/Σλ`, edge `(λ₂+λ₃)/Σλ`)를 residual/가중치로 — **온라인 localizer가 매 프레임 k-NN PCA 없이 point-to-plane / point-to-line 정합을 맵에 바로 수행**할 수 있다. 이는 BALM의 voxel primitive 표현을 *offline 정적 맵 생성* 쪽으로 옮긴 형태이며, surface·edge 해상도를 **클래스별 voxel filter로 독립 제어**하면 (평면은 성기게, edge는 촘촘히) 맵 크기와 정합 정밀도를 함께 조절할 수 있다.

정리하면, **Loam-Livox = 특징을 어떻게 뽑고 매 프레임 정합하는가**, **BALM = 특징을 맵에 어떻게 표현해 재피팅을 없애는가** 이며, 사전 맵 localization에서 후자의 primitive-map 표현이 온라인 비용·강건성 측면에서 이점을 준다.

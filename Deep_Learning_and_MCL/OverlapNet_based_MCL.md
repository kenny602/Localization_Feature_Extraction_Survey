# Learning an Overlap-based Observation Model for 3D LiDAR Localization

Chen, X., Läbe, T., Nardi, L., Behley, J., & Stachniss, C. (2020). *Learning an Overlap-based Observation Model for 3D LiDAR Localization*. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 4602–4608.

- 게재처: IROS
- 연도: 2020
- 분류: Projection-based (Range Image) / Learning-based Observation Model / Monte-Carlo Localization
- 입력 센서: 3D LiDAR
- 추정 상태: 3-DoF pose $(x,y,\theta)$
- Prior map: Aggregated 3D point cloud → 2D grid별 virtual scan → OverlapNet feature volume

<img width="1113" height="1408" alt="image" src="https://github.com/user-attachments/assets/bb5cb2fa-1ab7-4009-a582-2dfc68f55c71" />

---

## Abstract

본 연구는 3D LiDAR 기반 Monte-Carlo Localization(MCL)을 위한 **학습 기반 observation model**을 제안한다. 현재 query scan과 pre-built map에서 생성한 virtual scan을 OverlapNet에 입력하여 두 scan 사이의 **overlap 정도**와 **yaw angle offset**을 추정하고, 이를 각 particle의 importance weight로 사용한다.

기존 beam-end model이나 handcrafted histogram observation model과 달리, point·line·plane·landmark의 명시적인 대응관계를 구성하지 않는다. 대신 3D LiDAR scan을 range image로 투영하고 CNN이 학습한 feature volume을 비교하여 현재 위치와 방향의 일치도를 평가한다. 도시 환경에서 계절, 식생, 주차 차량 및 보행자 변화가 존재하는 장기 데이터에 적용했으며, 더 적은 particle로 높은 global localization 성공률을 보였다.

---

## Introduction

3D LiDAR를 사용하는 global localization에서는 로봇의 위치 prior가 없으므로 넓은 pose 공간을 탐색해야 한다. EKF나 일반적인 optimization 방법은 하나의 pose mode를 추적하는 데 적합하지만, 초기 위치가 불확실하고 유사 장소가 여러 개 존재하는 경우에는 다중 pose hypothesis를 유지하기 어렵다. Particle filter 기반 MCL은 여러 위치 후보를 동시에 유지할 수 있어 global localization에 적합하다.

MCL의 핵심은 현재 측정값과 prior map 사이의 일치도를 나타내는 observation model이다. 기존 LiDAR observation model은 주로 다음과 같다.

- Beam-end / likelihood-field model: scan endpoint와 map obstacle 사이의 거리 사용
- Ray-casting model: particle pose에서 예상되는 range와 실제 range 비교
- Handcrafted feature model: landmark, semantic object, histogram 등의 대응관계 사용

그러나 3D LiDAR에서는 point 수가 많고, particle마다 dense map과 대응관계를 계산하면 계산량이 크게 증가한다. 또한 정확한 point correspondence는 계절 변화, 식생, parked vehicle, moving object 등의 변화에 민감하다.

이 논문은 명시적인 대응점이나 semantic landmark를 정의하는 대신, **두 LiDAR scan이 실제로 얼마나 같은 공간을 관측하는가**를 나타내는 overlap을 학습한다. OverlapNet은 query scan과 map-side virtual scan 사이의 overlap과 yaw offset을 예측하며, 이 값을 MCL의 measurement likelihood로 사용한다. 따라서 deep network는 pose를 직접 회귀하지 않고, particle filter에서 각 pose hypothesis를 평가하는 **observation model** 역할을 수행한다.

---

## Method

### 1. 전체 구조

전체 pipeline은 다음과 같다.

```text
Offline
Pre-built 3D point cloud map
        ↓
2D grid 위치마다 virtual 360° LiDAR scan 생성
        ↓
각 virtual scan을 OverlapNet의 한쪽 leg에 입력
        ↓
Map-side feature volume 저장

Online
Current LiDAR query scan
        ↓
Range + normal image 생성
        ↓
Query-side feature volume 추출
        ↓
Particle 위치가 속한 grid의 map feature와 비교
        ↓
Overlap score + yaw prediction
        ↓
Particle weight update → resampling → 다음 시점
```

MCL은 다음 순서를 반복한다.

```text
Motion update → Observation update → Weighting → Resampling → Next timestep
```

각 particle은 다음 3-DoF pose hypothesis를 나타낸다.

$$
\mathbf{x}_t^i=(x_t^i,y_t^i,\theta_t^i)
$$

---

### 2. Online LiDAR scan의 특징 표현

#### 2.1 Range-image-like tensor

현재 3D LiDAR scan의 point $\mathbf{p}=(x,y,z)$를 spherical projection하여 2D range image 좌표 $(i,j)$에 배치한다. 각 pixel은 단순 range뿐 아니라 해당 point의 surface normal을 함께 갖는다.

$$
\mathbf{I}(i,j)=(r,n_x,n_y,n_z), \qquad r=\|\mathbf{p}\|_2
$$

따라서 입력 tensor는 다음과 같다.

$$
\mathbf{I}\in\mathbb{R}^{H\times W\times4}
$$

이 논문에서 localization에 사용되는 저수준 LiDAR feature는 다음 두 정보의 조합이다.

- **Range $r$**: 센서로부터 구조물까지의 거리와 장면의 깊이 배치
- **Normal $(n_x,n_y,n_z)$**: 벽, 지면, 차량, 식생 등 국소 표면의 방향과 기하 구조

즉, pole·corner·edge·plane을 discrete feature class로 검출하는 것이 아니라, scan 전체의 range와 local surface orientation을 dense 2D projection으로 표현한다.

#### 2.2 OverlapNet feature volume

OverlapNet은 weight를 공유하는 Siamese network이다. 두 scan의 tensor $\mathbf{I}_1,\mathbf{I}_2$를 각각 network leg에 입력하여 learned feature volume $\mathbf{F}_1,\mathbf{F}_2$를 생성한다.

$$
\mathbf{F}_1=\Phi(\mathbf{I}_1), \qquad
\mathbf{F}_2=\Phi(\mathbf{I}_2)
$$

이 feature volume은 raw point나 명시적 primitive가 아니라, range와 normal의 공간 패턴을 CNN이 집계한 **학습 기반 scan descriptor**이다. 이후 두 head가 feature volume pair를 사용한다.

- Overlap head: 두 scan 사이의 overlap percentage 예측
- Yaw head: 두 scan 사이의 relative yaw 예측

학습용 overlap과 yaw ground truth는 SLAM으로 구한 scan pose를 이용해 자동 생성하므로 별도의 수작업 annotation은 필요하지 않다.

---

### 3. Prior map의 특징 표현: Map of Virtual Scans

#### 3.1 Grid별 virtual scan

먼저 mapping sequence의 LiDAR scans를 global 좌표계에 누적하여 aggregated 3D point cloud map을 만든다. 이후 map의 $xy$ 평면을 resolution $\gamma$의 grid로 나누고, 각 grid location에서 virtual 360° LiDAR scan을 생성한다.

논문의 실험 설정은 다음과 같다.

- Grid resolution: $\gamma=20\,\text{cm}$
- Virtual scan yaw: $0^\circ$

모든 virtual scan의 yaw를 global $0^\circ$로 통일하므로, query scan과 virtual scan 사이에서 예측한 relative yaw는 query scan의 global yaw로 사용할 수 있다.

> **Virtual scan 생성에 관한 주의점**  
> 논문은 aggregated point cloud에서 각 grid location의 virtual 360° scan을 생성한다고 설명하지만, exact raycasting, z-buffer, occlusion 처리, 실제 LiDAR beam pattern 재현 등의 rendering 세부 알고리즘은 본 논문에 명시하지 않는다. 따라서 이를 정밀 sensor simulation이나 mesh raycasting으로 단정하기보다는, **pre-built point cloud map에서 grid viewpoint별 map scan을 생성한다**고 표현하는 것이 정확하다.

#### 3.2 Map-side feature volume 저장

Virtual raw scan 자체를 모두 저장하지 않고, 각 virtual scan을 OverlapNet의 한쪽 leg에 미리 통과시켜 feature volume $\mathbf{F}_g$만 저장한다.

$$
\mathbf{F}_g=\Phi(\mathbf{I}_g)
$$

장점은 다음과 같다.

- 저장량: 약 $0.1\,\text{GB/km}$로 raw scan map의 $1.7\,\text{GB/km}$보다 한 order 이상 작음
- Online에서 map-side feature extraction을 반복하지 않음
- 같은 grid에 속한 particle들은 동일한 map feature volume을 공유할 수 있음

따라서 prior map은 dense point cloud를 그대로 particle마다 검색하는 형태가 아니라,

$$
\text{Grid position}\rightarrow\text{virtual scan feature volume}
$$

의 lookup map으로 변환된다.

---

### 4. Monte-Carlo Localization

MCL은 다음 posterior를 particle set으로 근사한다.

$$
p(\mathbf{x}_t\mid \mathbf{z}_{1:t},\mathbf{u}_{1:t})
$$

Bayes filter update는 다음과 같다.

$$
p(\mathbf{x}_t\mid \mathbf{z}_{1:t},\mathbf{u}_{1:t})
=\eta\,p(\mathbf{z}_t\mid\mathbf{x}_t)
\int p(\mathbf{x}_t\mid\mathbf{u}_t,\mathbf{x}_{t-1})
p(\mathbf{x}_{t-1}\mid\mathbf{z}_{1:t-1},\mathbf{u}_{1:t-1})
\,d\mathbf{x}_{t-1}
$$

- $p(\mathbf{x}_t\mid\mathbf{u}_t,\mathbf{x}_{t-1})$: motion model
- $p(\mathbf{z}_t\mid\mathbf{x}_t)$: observation model

이 논문은 observation model이 핵심 기여이며, motion model은 standard vehicle odometry model을 사용한다.

각 iteration은 다음과 같다.

1. Odometry로 particle propagation
2. Particle의 $(x,y)$가 속한 grid 선택
3. Query feature와 해당 grid의 map feature 비교
4. Overlap 및 yaw likelihood로 particle weight 계산
5. Weight normalization
6. Effective particle number가 전체의 50% 이하가 되면 resampling
7. 다음 LiDAR frame에서 반복

수렴 후에도 MCL을 종료하지 않고 같은 과정을 반복하여 3-DoF pose를 지속적으로 추적한다.

---

### 5. Observation Model

Observation model은 위치와 방향을 분리한다.

$$
p(\mathbf{z}_t\mid\mathbf{x}_t)
=p_L(\mathbf{z}_t\mid\mathbf{x}_t)
\,p_O(\mathbf{z}_t\mid\mathbf{x}_t)
$$

#### 5.1 Location likelihood: overlap score

Particle $i$의 위치 $(x_i,y_i)$가 속한 grid의 virtual scan을 $\mathbf{z}_i$라고 하면, OverlapNet이 현재 query scan $\mathbf{z}_t$와 $\mathbf{z}_i$ 사이의 overlap을 예측한다.

$$
p_L(\mathbf{z}_t\mid\mathbf{x}_t^i)
\propto f(\mathbf{z}_t,\mathbf{z}_i;\mathbf{w})
$$

- $f$: OverlapNet의 overlap head
- $\mathbf{w}$: 학습된 network weights

Overlap score가 높다는 것은 해당 grid 위치에서 생성된 map observation과 현재 scan이 넓은 공통 관측 영역을 갖는다는 의미이다.

이 score는 point-to-point distance처럼 정확한 대응 거리의 합이 아니다. 이를 error 형태로 해석하면 다음처럼 볼 수 있지만, 실제 알고리즘은 network output을 likelihood로 직접 사용한다.

$$
E_L^i\simeq-\log f(\mathbf{z}_t,\mathbf{z}_i;\mathbf{w})
$$

Overlap likelihood는 true pose와 정확히 같은 grid가 아니어도 주변 grid에 비교적 높은 값을 줄 수 있어, sparse particle set에서도 실제 위치 부근의 hypothesis가 살아남을 가능성을 높인다.

#### 5.2 Orientation likelihood: yaw prediction

OverlapNet의 yaw head가 query scan과 virtual scan 사이의 relative yaw를 예측한다.

$$
\hat\theta_i=g(\mathbf{z}_t,\mathbf{z}_i;\mathbf{w})
$$

Virtual scan의 yaw가 모두 $0^\circ$이므로 $\hat\theta_i$는 query scan의 global yaw estimate가 된다. Particle yaw $\theta_i$와의 차이를 Gaussian likelihood로 변환한다.

$$
p_O(\mathbf{z}_t\mid\mathbf{x}_t^i)
\propto
\exp\left(
-\frac{1}{2}
\frac{\left(g(\mathbf{z}_t,\mathbf{z}_i;\mathbf{w})-\theta_i\right)^2}
{\sigma_\theta^2}
\right)
$$

실험에서는 $\sigma_\theta=5^\circ$를 사용한다.

#### 5.3 최종 particle weight

따라서 particle $i$의 measurement weight는 다음과 같이 해석할 수 있다.

$$
\tilde w_t^i
\propto
f(\mathbf{z}_t,\mathbf{z}_i;\mathbf{w})
\exp\left(
-\frac{\left(g(\mathbf{z}_t,\mathbf{z}_i;\mathbf{w})-\theta_i\right)^2}
{2\sigma_\theta^2}
\right)
$$

즉,

- **위치**: query와 map virtual scan이 얼마나 overlap하는가?
- **방향**: particle yaw가 network의 yaw prediction과 얼마나 일치하는가?

를 함께 사용한다.

---

## Localization을 위한 LiDAR Feature 관점의 분석

### 1. Online scan matching에서의 feature

이 논문은 consecutive scan 사이에서 point-to-point·point-to-plane·point-to-line residual을 계산하는 전통적인 scan matching을 제안하지 않는다. Motion update는 외부 odometry를 입력받으며, 논문의 feature는 observation update에 사용된다.

Online query 측 feature는 다음의 2단계 표현이다.

1. **Handcrafted projection feature**
   - Spherical range image
   - Pixel channel: range + surface normal

2. **Learned scan feature**
   - Siamese CNN leg가 생성하는 feature volume
   - Scan 전체의 depth·surface orientation 패턴을 압축

따라서 이 논문에서의 scan feature는 sparse keypoint나 explicit geometric primitive가 아니라 **dense range-normal projection을 CNN으로 인코딩한 scan-level feature**이다.

### 2. Map matching에서의 feature

Map 측 feature는 aggregated point cloud map 자체가 아니라 다음 구조이다.

```text
2D grid location
    ↓
Virtual 360° map scan
    ↓
Range + normal tensor
    ↓
Precomputed OverlapNet feature volume
```

Particle마다 $(x,y)$가 속한 grid를 조회하고, 해당 grid의 map feature volume과 현재 query feature volume을 비교한다.

즉 map matching 단위는

$$
\text{Query scan feature}\leftrightarrow\text{grid-level virtual scan feature}
$$

이다. Map point와 query point의 correspondence를 만들지 않으며, map-side descriptor는 offline에 미리 저장된다.

### 3. Matching score / residual의 성격

이 논문의 matching은 classical registration이 아니라 **learned similarity evaluation**이다.

- Explicit correspondence: 없음
- Point-to-point residual: 없음
- Point-to-line / point-to-plane residual: 없음
- ICP/NDT optimization: 없음
- Score: learned overlap + yaw consistency
- Pose update: gradient-based optimization이 아니라 MCL weighting/resampling

따라서 network가 직접 pose를 연속적으로 미분 최적화하는 것이 아니라, 각 particle pose hypothesis가 얼마나 관측과 일치하는지를 평가한다.

### 4. Feature의 공간적 해상도

Overlap score의 위치 분해능은 map grid resolution $\gamma$의 영향을 받는다. 같은 grid에 속하는 particle들은 동일한 query–map overlap과 yaw prediction을 공유하며, particle별 차이는 yaw likelihood에서 particle의 $\theta_i$와 비교할 때 발생한다.

이는 계산 효율에는 유리하지만, map feature가 grid 단위로 이산화되어 있어 fine registration보다 coarse한 observation surface를 형성한다. 최종 위치 정밀도는 grid resolution, odometry, particle distribution, network prediction이 함께 결정한다.

### 5. 3D LiDAR를 사용하지만 3-DoF인 이유

입력 feature는 3D 구조의 range와 normal을 포함하지만, particle state와 map indexing은 다음으로 제한된다.

$$
(x,y,\theta)
$$

Network도 overlap과 yaw만 출력하므로 $z$, roll, pitch를 직접 관측하거나 추정하지 않는다. 따라서 이 방법은 **3D LiDAR observation을 사용하는 planar 3-DoF localization**이다.

---

## OverlapNet 기반 MCL의 특징

1. Range와 normal을 함께 사용한 projection-based LiDAR representation이다.
2. 명시적인 landmark·semantic object·edge·plane 검출 없이 scan 전체의 appearance를 학습한다.
3. Deep network를 MCL의 observation model로 사용하며, pose regression model로 사용하지 않는다.
4. Location likelihood와 yaw likelihood를 분리하여 3-DoF particle weight를 계산한다.
5. Prior map을 grid별 precomputed feature volume으로 저장하여 online map-side feature extraction을 줄인다.
6. MCL을 initialization에만 사용하지 않고 수렴 후에도 지속적으로 3-DoF pose를 추정한다.
7. Multi-modal particle distribution을 유지하므로 유사 장소가 여러 개 존재하는 global localization에 대응할 수 있다.

---

## OverlapNet 기반 LiDAR Feature의 장점

1. **Exact correspondence가 필요하지 않음**  
   Point 또는 primitive 간 one-to-one correspondence를 만들지 않아 sparse LiDAR sampling과 부분적인 환경 변화에 덜 민감하다.

2. **근접 pose에 넓은 likelihood를 제공**  
   Exact endpoint 일치 대신 overlap을 평가하므로 true pose 주변 particle도 상대적으로 높은 weight를 받을 수 있다. 그 결과 beam-end model보다 적은 particle로 global localization 성공률을 높일 수 있다.

3. **Range와 normal을 동시에 활용**  
   단순 range histogram보다 scan의 공간 배열과 local surface orientation을 더 많이 보존한다.

4. **Map feature의 offline precomputation**  
   Grid별 map feature volume을 미리 저장하므로 online에서는 query feature만 계산하면 된다.

5. **Grid 단위 inference 공유**  
   같은 grid에 존재하는 particle은 동일한 network 결과를 공유하여 particle 수보다 실제 network evaluation 횟수를 줄일 수 있다.

6. **Online GPS가 필요하지 않음**  
   Localization 단계에서는 3D LiDAR와 odometry만 사용하며, MCL을 이용해 global pose를 찾는다.

7. **계절·동적 변화에 대한 실험적 강건성**  
   서로 다른 계절에 수집된 sequence를 map과 query로 사용했으며, 식생·주차 차량·보행자 변화가 있는 환경에서도 localization을 수행했다.

---

## OverlapNet 기반 LiDAR Feature의 단점 및 한계

1. **환경별 재학습 필요**  
   논문은 map scans와 생성된 virtual scans를 이용해 localization map별로 새 OverlapNet model을 학습한다. 새로운 환경에 map만 교체하여 즉시 적용하는 handcrafted 방식은 아니다.

2. **센서 일반화가 검증되지 않음**  
   Range image는 LiDAR의 FoV, channel 구성, angular sampling pattern에 영향을 받는다. 저자도 향후 다른 LiDAR sensor에서 검증할 계획이라고 제시한다.

3. **3-DoF에 제한**  
   $z$, roll, pitch를 추정하지 않으므로 경사·층간 이동·자유로운 3D motion이 있는 환경의 6-DoF localization에는 직접 적용할 수 없다.

4. **Grid quantization trade-off**  
   Grid를 조밀하게 하면 위치 표현은 세밀해지지만 저장량과 map preprocessing 비용이 증가한다. Grid를 크게 하면 서로 다른 위치가 동일한 map feature를 공유하여 위치 분해능이 낮아진다.

5. **Fine geometric alignment 부재**  
   Overlap과 yaw는 pose hypothesis의 plausibility를 평가하지만, point-to-plane 또는 point-to-line residual로 sub-grid pose를 정밀 최적화하지 않는다. 따라서 최종 정밀도 측면에서는 별도의 geometric refinement가 유리할 수 있다.

6. **Perceptual aliasing 가능성**  
   반복되는 도로·건물 구조에서는 서로 다른 위치의 range-normal pattern이 유사할 수 있다. MCL의 sequence와 motion model이 ambiguity를 줄이지만, feature 자체가 잘못된 위치에 높은 overlap을 줄 가능성은 남는다.

7. **Virtual map scan 품질 의존**  
   Aggregated map의 density와 viewpoint coverage가 부족하면 virtual scan이 실제 query viewpoint의 관측을 충분히 재현하지 못할 수 있다. 또한 본 논문은 virtual scan rendering의 visibility·occlusion·beam simulation 세부사항을 제공하지 않는다.

8. **GPU 및 초기 global search 비용**  
   GTX 1080 Ti 환경에서 large-scale initialization의 worst case는 한 frame 처리에 약 43초였으며, 수렴 후에도 10,000 particles 기준 평균 약 1초가 필요했다.

9. **장기 변화에 대한 명시적 static filtering 없음**  
   계절·동적 변화에 대한 실험은 수행했지만, map 또는 query에서 dynamic object를 명시적으로 제거하거나 long-term static feature만 선택하는 구조는 아니다. 강건성은 learned descriptor의 일반화 능력에 의존한다.

---

## Experimental Results 요약

동일한 MCL framework에서 observation model만 교체하여 beam-end 및 histogram 기반 방법과 비교했다.

- Particle 수: $1{,}000$, $5{,}000$, $10{,}000$, $50{,}000$, $100{,}000$
- Success condition: 수렴 후 위치 오차 $5\,\text{m}$ 이내
- Proposed method는 더 적은 particle에서 높은 성공률을 보임
- 위치 RMSE:
  - Sequence 00: $0.81\pm0.13\,\text{m}$
  - Sequence 01: $0.88\pm0.07\,\text{m}$
- Yaw RMSE:
  - Sequence 00: $1.74\pm0.11^\circ$
  - Sequence 01: $1.88\pm0.09^\circ$

위치 오차는 beam-end model과 유사하지만, decoupled yaw observation model로 yaw estimation이 향상되었고 global localization 성공률과 계산 효율이 개선되었다.

---

## 핵심 정리

이 논문에서 localization feature의 핵심은 다음과 같다.

```text
명시적인 edge / surface / pole / semantic landmark가 아니라,
3D LiDAR scan을 range + normal image로 투영한 뒤
CNN이 생성한 learned feature volume을 사용한다.
```

Online query feature와 grid별 map virtual-scan feature를 OverlapNet으로 비교하여,

```text
Overlap = 위치 hypothesis의 일치도
Yaw prediction = 방향 hypothesis의 일치도
```

를 얻고, 이를 MCL의 particle weight로 사용한다.

따라서 본 방법은 **projection-based learned scan descriptor를 MCL observation model로 사용하는 sequential 3-DoF global localization**으로 분류할 수 있다.

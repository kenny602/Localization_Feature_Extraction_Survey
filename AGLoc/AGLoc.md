# Robust Lifelong Indoor LiDAR Localization Using the Area Graph

- **저자**: Fujing Xie, Sören Schwertfe
- **게재처**: IEEE Robotics and Automation Letters 9(1) (2024) 531–538
- **연도**: 2024
- **분류**: Geometric Primitive (Area Graph polygon 기반 장기 실내 전역 위치추정)

---

<img width="2097" height="488" alt="image" src="https://github.com/user-attachments/assets/d3ce4c12-9385-4a5b-b4c0-b1c7b6d153fc" />

---
## Abstract

AGLoc은 사전에 구축된 Area Graph와 3D LiDAR scan point를 이용하여, 가구·사람 등 clutter가 많은 **실내** 환경에서도 **장기간** 안정적으로 위치를 추정하는 시스템이다.
Area Graph는 방과 복도를 polygon으로 표현하고, 건물–층–방의 계층 구조와 passage(door), 유리벽 등의 semantic information을 함께 저장하는 hierarchical topometric map이다. 
AGLoc은 다음 두 단계로 구성되어 Localization( global localization + pose tracking )을 수행한다.

1. **Global localization**: WiFi와 기압계로 탐색 범위를 대략적으로 줄인 뒤, 여러 pose 후보와 Area Graph의 일치도를 평가하여 초기 위치를 찾는다.
2. **Pose tracking**: clutter-free 2D LiDAR point set과 Area Graph polygon을 weighted point-to-line ICP로 정합(보정)하여 pose를 지속적으로 추적한다.

또한 긴 복도처럼 한 방향의 평행 벽이 관측을 지배하는 환경에서는 corridorness downsampling을 적용하여 ICP의 위치추정 성능 저하를 완화한다.

---

## Introduction

실내에서는 GPS를 사용할 수 없기 때문에 occupancy grid map, 3D point cloud map, visual bag-of-words 등의 사전 지도를 이용한 localization이 주로 사용된다. 그러나 이러한 지도는 대규모 환경에서 데이터가 커질 수 있으며, 가구 배치나 시각적 환경이 바뀌면 현재 관측과 지도 사이에 불일치가 발생한다. 따라서 장기간 사용하려면 지도를 지속적으로 갱신해야 하는 문제가 있다.
본 논문은 장기간 실내 localization을 위한 map representation이 다음 조건을 만족해야 한다고 본다.

- 캠퍼스나 대형 건물에도 사용할 수 있도록 **compact**해야 한다.
- 벽·문처럼 장기간 변하지 않는 구조를 표현하여 **지속적인 지도 갱신이 필요 없어야 한다**.
- 로봇이 모든 공간을 직접 SLAM하지 않아도 기존 건축 정보로 **쉽게 구축할 수 있어야 한다**.

이를 위해 AGLoc은 방·복도를 polygon으로 표현하는 Area Graph를 사용한다. 각 공간은 area node로 표현되고, 인접한 공간을 연결하는 문이나 통로는 **passage**로 표현된다. 또한 건물–층–방의 계층 구조와 층 높이, 공간 종류 등의 semantic information을 저장할 수 있다.
AGLoc의 목표는 가장 상세한 환경 지도를 이용해 순간적인 최고 정확도를 얻는 것이 아니라, **환경 변화에도 동일한 지도를 장기간 재사용할 수 있는 robust lifelong indoor localization**을 제공하는 것이다.

---

## Method

논문 본문은 방법론을 **A–G 절**로 구분하여 설명한다. 특히 **section C의 intersection 및 sd 계산은 pose가 주어진 뒤 수행되는 공통 연산**이므로, global localization에서는 D에서 생성한 각 pose 후보에 대해 반복적으로 사용된다.

### 실제 실행 순서

#### Global Localization

1. A. Area Graph Loading  
   WiFi와 기압계로 현재 층과 대략적인 위치 범위를 추정하고, 해당 층의 Area Graph를 불러온다.
2. B. 2D Clutter-Free Point Set  
   3D LiDAR scan을 2D로 투영하고, 각 azimuth column에서 가장 먼 point만 남겨 clutter를 줄인다.
3. D. Pose Guess Generation  
   WiFi가 제공한 약 6 m 반경 안에서 여러 $(x,y,\mathrm{yaw})$ pose 후보를 생성한다.
4. C. Intersection and Signed Distance Calculation  
   각 pose 후보마다 LiDAR point와 Area Graph polygon의 intersection 및 signed distance를 계산한다.
5. E. Weight Function <br>
   Signed distance에 따라 clutter와 이상치의 영향을 줄이는 weight를 계산한다.
6. D. Guess Scoring  
   C와 E에서 계산한 signed distance와 weight를 이용해 각 pose 후보의 score를 구하고, 가장 높은 score의 pose를 global localization 결과로 선택한다.

#### Pose Tracking

1. B. 2D Clutter-Free Point Set을 생성한다.
2. C. Intersection and Signed Distance Calculation으로 현재 pose에서 point-to-line residual을 구한다.
3. E. Weight Function으로 각 residual의 신뢰도를 계산한다.
4. G. Corridorness Downsampling으로 한 방향의 wall point가 과도하게 많은 경우 dominant orientation의 point를 추가로 줄인다.
5. F. Weighted Point-to-Line ICP로 weighted residual을 최소화하도록 pose를 보정한다.
6. 이후 frame에서는 이전 tracking 결과를 다음 ICP의 initial guess로 사용한다.

---

### A. Area Graph Loading and Reloading

Area Graph는 지역–건물–층–방으로 이어지는 계층 구조를 가진다. AGLoc은 WiFi localization으로 대략적인 수평 위치를 구하고, 로봇과 base station의 기압 차이 및 Area Graph에 저장된 층 높이 정보를 이용하여 현재 층을 추정한다.
WiFi localization의 수평 오차는 반경 약 6 m로 가정하며, 이 정보는 정확한 pose를 직접 결정하기보다 global localization의 탐색 범위를 줄이는 데 사용된다.
정합에는 osmAG의 topology 정보보다는, 현재 층에 속한 leaf area의 polygon geometry만을 사용한다. 
특히 polygon을 구성하는 vertex의 metric position과 **passage** 정보가 중요하다.
<img width="843" height="635" alt="image" src="https://github.com/user-attachments/assets/e4f9eace-4698-4216-b6d4-5b0a9f9efd59" />

---

### B. 2D Clutter-Free Point Set and Subsampling

Area Graph는 벽과 passage를 2D polygon으로 표현하므로, 3D LiDAR point cloud도 2D 정합에 적합한 형태로 변환해야 한다.
AGLoc은 다음 가정을 사용한다.
> 동일한 azimuth 방향의 수직 LiDAR column에서 가장 멀리 측정된 point는 wall에 도달할 가능성이 높다.

LiDAR scan을 수직 channel 수 $(N)$, 수평 방향 수 $(M)$의 행렬 $N×M$로 표현할 때, 각 azimuth(세로) column에서 2D 거리 $(p_{i,j})$가 가장 큰 point 하나만 선택한다. 선택된 point는 높이 $(z)$를 제거하여 2D point로 사용한다.
이를 통해 사람·의자 등 가까운 clutter point를 줄이고, 3D point cloud를 2D로 투영하며, 전체 point 수를 줄여 계산량을 감소시킨다.
대형 캐비닛처럼 가장 먼 point 자체가 clutter일 수 있으나, 이러한 point는 이후 **section E**의 weight function에서 추가로 억제한다.

---

### C. Intersection Calculation

<img width="677" height="261" alt="image" src="https://github.com/user-attachments/assets/258136bc-115b-4a44-96b6-4db37763e372" />

C는 독립적인 한 번의 처리 단계라기보다, **주어진 pose에서 LiDAR point와 Area Graph polygon의 관계를 계산하는 공통 연산**이다.
각 pose에서 2D clutter-free point $(p_j)$ 방향으로 ray를 생성하고, 현재 area의 Area Graph polygon과 교차하는 지점 $(i_j)$를 계산한다. 또한 $(p_j)$에서 polygon까지 가장 가까운 점을 $(c_j)$라 하면 signed distance $(sd_j)$는 다음과 같이 정의된다.

```math
sd_j=\lVert p_j-c_j\rVert
```

센서 point가 예상되는 polygon wall보다 로봇에 가까운 내부에 위치하면 음수로 설정한다.
```math
sd_j=-sd_j
\qquad \text{if}\qquad
\lVert p_j\rVert<\lVert i_j\rVert
```

- $sd_j<0$: point가 예상 wall보다 안쪽에 있음
- $sd_j=0$: point가 polygon wall과 일치함
- $sd_j>0$: point가 예상 wall을 지나 바깥쪽에 있음

Global localization에서는 D에서 생성한 **각 pose 후보마다 C를 수행**하고, pose tracking에서는 현재 ICP pose가 갱신될 때마다 C를 다시 수행한다. Passage나 transparent polygon은 ray가 다음 area까지 통과할 수 있으므로 일반 wall과 구분하여 처리한다.
Passage 판단에 사용하는 임계값은 다음과 같다.

```math
th_{\mathrm{passage}}=0.1\,\mathrm{m}
```

---

### D. Guess Scoring

WiFi localization이 제공한 약 6 m 반경의 탐색 범위에서 여러 pose 후보를 균일하게 생성한다.
pose 후보 g는 다음과 같이 정의된다.
```math
g_h=(x_h,y_h,\theta_h,v_h)
```
```math
G=\{g_h\}_{h=1}^{n}
```
G는 pose 후보의 집합이다.
각 후보에 대해 C의 intersection 및 signed distance 계산을 수행한 뒤, 다음 error 또는 fit 항으로 clutter-free point set과 Area Graph의 일치도를 평가한다.

#### Nearby Error (E_1)

Polygon과 가까운 point의 signed distance를 오차로 사용하고, 임계값(threshold) $(d=0.8\text{ m}$)를 초과하는 point에는 고정 penalty를 부여한다.


<img width="660" height="156" alt="image" src="https://github.com/user-attachments/assets/2ce6fbdf-c155-4894-827d-e0671ab9f8fa" />


#### Weighted Outside Fit (E_2)

Polygon 바깥쪽에 있는 point만 대상으로 E에서 정의하는 weight function을 적용한다.

<img width="585" height="156" alt="image" src="https://github.com/user-attachments/assets/89f9aa6a-279c-4c40-9ada-7155589640d1" />

#### Unweighted Outside Error (E_3)

Polygon 바깥쪽 point의 signed distance를 가중치 없이 누적한다.

<img width="533" height="167" alt="image" src="https://github.com/user-attachments/assets/03b1a74d-43ef-4d4a-8aef-03ae32da46ef" />

더하여, score function은 다음과 같이 정의된다.

```math
S_1=\frac{1}{E_1},
\qquad
S_2=E_2,
\qquad
S_3=\frac{E_2}{E_1},
\qquad
S_4=\frac{1}{E_3}
```

이 항들을 조합한 여러 score function을 비교하고, 실험에서 정확도와 계산시간이 우수한 score(여기서는 S1)를 사용한다. 최종적으로 가장 높은 score를 가진 pose 후보를 global localization 결과로 선택한다.

---

### E. Weight Function

Global localization의 pose scoring과 F의 pose tracking에서 clutter와 비정상 측정의 영향을 줄이기 위해 signed distance 기반 weight function을 사용한다.

<img width="630" height="481" alt="image" src="https://github.com/user-attachments/assets/baa1fabd-1840-48b3-b0af-f5a0e5bad423" />

<img width="796" height="260" alt="image" src="https://github.com/user-attachments/assets/02000ec9-41a2-499a-99d5-a6a257271d0e" />

- $sd_j\le-1$: wall보다 1 m 이상 안쪽에 있는 point
- $-1<sd_j\le0$: wall 안쪽에 있으며, wall에 가까울수록 높은 weight를 부여
- $0<sd_j<3$: 예상 wall보다 beam이 길게 측정된 point
- $sd_j\ge3$: 반사 또는 이상치일 가능성이 높은 point

즉 polygon wall과 가까운 point는 강하게 반영하고, clutter·오정합·이상치는 약하게 반영하거나 제외한다.

---

### F. Pose Tracking

Global localization 이후에는 weighted point-to-line ICP를 이용하여 pose를 추적한다. 첫 frame의 initial guess는 global localization 결과를 사용하고, 이후 frame부터는 이전 tracking 결과를 initial guess로 사용한다.
최적화하기 위한 오차를 구하는 함수는 다음과 같다.

```math
\sigma=
\min_{R,t}
\sum_{j=1}^{M}
W\left(\pi(p_j,R,t)\right)
\left\|\pi(p_j,R,t)\right\|^2
```
여기서 회전행렬과 평행이동 벡터는 다음과 같다.

```math
R\in\mathbb{R}^{2\times2},
\qquad
t\in\mathbb{R}^{2}
```
여기서 $(\pi(p_j,R,t))$는 pose $(R,t)$로 변환한 LiDAR point와 대응 Area Graph polygon segment 사이의 signed distance를 의미한다.
즉 C에서 계산한 point-to-line residual에 E의 weight를 적용하고, 전체 weighted squared residual이 최소가 되도록 $(R,t)$를 보정한다. Pose가 갱신되면 correspondence와 signed distance를 다시 계산하며, 이 과정을 수렴할 때까지 반복한다.

---

### G. Corridorness Downsampling

긴 복도나 한쪽 wall만 주로 관측되는 환경에서는 많은 LiDAR point가 동일한 방향의 평행한 polygon segment에 대응한다. 이 경우 wall과 수직인 위치는 보정할 수 있지만, wall을 따라가는 방향의 위치 정보가 부족하여 ICP가 초기 위치에 머물 수 있다.
이를 완화하기 위해 현재 scan point들이 대응된 Area Graph polygon segment의 orientation을 $(5^\circ)$ 간격의 histogram에 누적한다. 선분 orientation은 $(180^\circ)$ 주기를 가지므로 총 36개의 bin을 사용한다. ( 식참고 )
```math
0^\circ\le\theta<180^\circ
```

```math
\frac{180^\circ}{5^\circ}=36
```

<img width="855" height="350" alt="image" src="https://github.com/user-attachments/assets/aea72c57-13a2-4742-955d-c6f046dff6e7" />

Corridorness는 다음과 같이 정의한다.

```math
Cor=\frac{N_{\max}}{N_m}
```
- $N_{\max}$: point가 가장 많이 들어간 orientation bin의 point 수
- $N_m$: 전체 대응 point 수
- $Cor\approx0.5$: 두 방향의 wall이 비슷하게 관측됨
- $Cor=1$: 한 방향의 wall만 관측됨

가장 많은 point가 들어간 dominant orientation에만 다음 downsample rate를 적용한다.

```math
R_{\mathrm{cor}}=
\left\{
\begin{array}{ll}
1, & 0<Cor\le0.5, \\[4pt]
10Cor-4, & 0.5<Cor\le1
\end{array}
\right.
```
- $(Cor\le0.5)$: 특정 방향의 지배가 크지 않으므로 downsampling하지 않음
- $(Cor>0.5)$: 한 방향으로 편중될수록 dominant orientation point를 더 많이 감소
- $(R_{cor}=2)$: 해당 방향 point를 대략 (1/2) 유지
- $(R_{cor}=6)$: 해당 방향 point를 대략 (1/6) 유지

이 방법은 비평행 point를 새로 만드는 것이 아니라, 과도하게 많은 평행 wall point를 줄여 문·기둥·끝벽·꺾인 wall 등 다른 방향의 구조가 ICP에서 상대적으로 더 큰 영향을 갖게 한다.
또한 LiDAR scan 자체에서 line detection을 수행하지 않고, 현재 scan point와 대응된 Area Graph polygon segment의 orientation을 사용한다. 단, initial guess가 올바른 Area Graph area 안에 있어야 하며 Area Graph의 polygon geometry가 정확하다는 전제가 필요하다.

---

## AGLoc의 특징

1. 상세한 occupancy grid나 point cloud map 대신 방·복도를 polygon으로 표현한 compact한 Area Graph를 사용한다.
2. WiFi·기압계로 탐색 범위를 제한한 뒤 LiDAR–polygon 정합으로 global localization을 수행한다.
3. 각 azimuth column의 가장 먼 point만 선택하여 clutter를 줄인 2D point set을 생성한다.
4. signed distance 기반 score function으로 여러 pose 후보의 Area Graph 일치도를 평가한다.
5. clutter와 이상치의 영향을 줄이는 weight function과 weighted point-to-line ICP를 사용한다.
6. corridorness downsampling으로 복도나 단일 wall 환경에서 발생하는 방향 편중을 완화한다.
7. global localization 이후에는 이전 frame의 pose만 사용하여 tracking하므로 odometry나 IMU에 의존하지 않는다.

---

## AGLoc의 장점

1. **장기간 재사용 가능한 compact map**  
   벽·문처럼 비교적 변하지 않는 건축 구조만 Area Graph polygon으로 표현하므로, 가구 배치나 사람 이동에 따른 지도 갱신 부담이 작다.
2. **하나의 map representation으로 global localization과 pose tracking을 모두 수행**  
   WiFi·기압계로 탐색 범위를 줄인 뒤 Area Graph 기반 pose scoring으로 초기 위치를 찾고, 이후 동일한 polygon map을 weighted point-to-line ICP에 사용한다.
3. **Clutter와 이상치에 대한 강건성**  
   가장 먼 LiDAR point 기반 subsampling과 signed distance 기반 weight function을 함께 사용하여 사람·가구·반사 측정의 영향을 줄인다.
4. **복도형 환경에 대한 추가 보완**  
   Corridorness downsampling으로 동일 방향의 평행 벽 point가 과도하게 많은 경우 그 영향을 줄이고, 다른 방향의 구조 point가 정합에 더 잘 반영되도록 한다.

---

## AGLoc의 한계

1. **정확한 Area Graph와 올바른 초기 area가 필요함**  
   Polygon geometry가 부정확하거나 initial guess가 잘못된 area에 위치하면 point-to-segment correspondence가 틀어져 localization 성능이 저하될 수 있다.
2. **가장 먼 point가 항상 wall이라는 보장은 없음**  
   대형 캐비닛이나 높은 clutter가 wall을 가리면 clutter-free point set에 잘못된 point가 포함될 수 있다.
3. **2D 정합에 따른 높이 정보 손실**  
   3D LiDAR 데이터를 2D로 투영하여 사용하므로, 층 판단은 기압계에 의존하고 높이 방향의 구조 차이는 직접적인 정합 정보로 활용하지 않는다.
4. **비평행 구조가 전혀 없는 긴 복도에서는 근본적인 한계가 남음**  
   Corridorness downsampling은 기존 비평행 point의 상대적 영향만 높일 수 있으며, 관측되지 않은 종방향 정보를 새로 만들어낼 수는 없다.

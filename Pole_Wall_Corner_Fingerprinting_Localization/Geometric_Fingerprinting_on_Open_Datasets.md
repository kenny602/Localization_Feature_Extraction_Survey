# Robust LiDAR Feature Localization for Autonomous Vehicles Using Geometric Fingerprinting on Open Datasets

Steinke, N., Ritter, C.-N., Goehring, D., & Rojas, R. (2021). Robust LiDAR feature localization for autonomous vehicles using geometric fingerprinting on open datasets. _IEEE Robotics and Automation Letters_, 6(2), 2761-2768.

---

## Abstract

본 연구는 자율주행 차량을 위한 **특징 기반 localization**으로, 베를린 시의 **공개 데이터(open GIS)** 로 생성한 feature map에 LiDAR가 검출한 특징을 **기하 fingerprinting**으로 강건하게 연관시킨다. pole류(나무·신호등·표지판·가로등)·벽·건물 코너처럼 도시에 흔하고 검출이 쉬우며 시간/계절에 불변한 특징을 사용한다. 핵심 기여는 **여러 해 지난 부정확한 공개 지도에서도** 누락/추가된 특징에 강건하게 정합되는 fingerprint 매칭(데이터 연관) 알고리즘이다. IMU(선택)·차량 오도메트리·대략적 초기 pose만으로 평균 lateral 오차 7.3 cm를 달성해 고가 GNSS를 능가했고, 360° 스캔당 44.7 ms로 실시간 동작한다.

---

## Introduction

자율주행에는 정밀 자기 위치추정이 필수지만, GNSS는 도심 다중경로로 정확도가 부족하다. LiDAR 특징 기반 localization은 cm급 정확도를 주지만 보통 SLAM으로 **feature map을 직접 구축**해야 하는 부담이 크다.

이 연구의 착안은, 풍부해진 **도시 공개 지리정보(open data)** 를 feature map으로 쓰는 것이다. 다만 공개 데이터는 수 년 지나 부정확하고 누락/추가가 있을 수 있어, **지도 부정확성에 강건한 데이터 연관**이 필요하다. 이를 위해 각 특징을 그 종류와 **주변 특징들과의 기하 관계(fingerprint)** 로 식별한다. 핵심 처리 흐름은 (1) LiDAR에서 pole·벽·코너 특징 추출 → (2) fingerprint로 맵 특징과 데이터 연관 → (3) 연관된 특징 쌍으로 point-to-point residual 정의 → (4) SVD로 변환을 풀고 EKF로 융합이다.

---

## Method

### 1. localization을 위해 추출하는 특징 (Feature Extraction)

특징은 크게 **점 특징(point feature)** 과 **에지 특징(edge feature)** 으로 나뉜다.

- **Pole 특징 (점)**: scan-line 기반 검출기. 회전형 LiDAR에서 한 레이저의 시간 궤적은 지면 위 원을 그리는데, pole이 있으면 그 부분 점들이 **주변보다 차량에 1 m 이상 가깝게** 들어온다. 수평 1 m 이내로 모인 그런 점들을 찾고, 다수의 ring(8개 사용)에서 보이면 pole로 판정한다. pole을 원/사각으로 가정해 양 끝점으로 방향벡터를 만들고 중심점을 추정(ring별 추정의 평균). 검출기는 단순해 오검출이 많지만 빠르고(평균 9.1 ms), 오검출은 이후 fingerprint 인식 단계에서 걸러진다.
- **Wall 특징 (에지)**: scan-line으로 점들을 순차적으로 직선에 적합(최대 선 거리 0.25 m, 직전 점과 0.5 m). 한 선분의 누적 길이가 **5 m 이상이면 벽 가설**로 보고 **RANSAC 직선 적합**을 수행한다. 벽은 보통 일부만 관측되므로 **횡방향(lateral) 보정에만** 기여한다.
- **Corner 특징 (점)**: 주로 건물 코너. 두 벽이 **±80°~±100°** 로 만나면 코너로 정의한다.
- **특징 추적(tracking)**: pole·corner는 점 특징으로 보고 최근접(NN) 추적(LiDAR ~10 Hz). 벽은 거리뿐 아니라 **각도까지** 고려해 추적한다.

### 2. 데이터 연관 — 기하 Fingerprint 매칭 (Data Association)

검출 특징을 맵 특징과 연관(association)시키는 것이 핵심이다. 지도가 오래되고(2015) 부정확하므로, **누락/추가 특징에 강건한** fingerprint 매칭을 쓴다.

**(a) Fingerprint 정의.** 점 특징 $a$ 의 fingerprint는 반경(50 m) 내 다른 모든 특징 $b$ 에 대한 **(유클리드 거리, 각도)** 튜플 집합이다.

$$f_a = \{\,(d(a,b),\,\alpha(a,b)),\,\dots\,\}$$

각도는 **UTM 격자 북(north) 기준 시계방향**으로 측정한다(식 2). 따라서 초기 pose의 북 방향을 ±10° 이내로 알아야 한다(저가 GNSS로 충분; 거리 필드만 매칭하면 우회 가능하나 초기화가 느려짐).

$$\alpha(p_1,p_2) = \cos^{-1}\!\Big(\tfrac{p_{1y}-p_{2y}}{d(p_1,p_2)}\Big)\,\mathrm{sgn}(p_{1x}-p_{2x})$$

에지(벽)는 **무한 직선**으로 취급한다. 일부만 관측돼도 식별 가능하도록, 에지-점 거리는 **최소 유클리드 거리**(점에서 직선에 내린 수선, 식 3–6), 에지-점 각도는 rejection 벡터로(식 7–8) 정의한다. 에지-에지는 각도(식 9)로 정의하되, 두 무한직선은 평행이 아니면 어딘가에서 거리 0이 되므로 **에지-에지 "거리"를 각도차로 대체**한다(식 11). 이는 fingerprint 고유성을 약화시키지만, 벽은 사실상 **각도만 안정적으로 검출**되기 때문이다.

**(b) 맵 생성·저장.** 베를린 WFS(GIS)에서 가로수·신호등·표지판·가로등(=pole)과 건물 외곽 폴리곤을 받아, 폴리곤 변으로 벽/코너를 계산(같은 벽 판정 각도 $\pi/10$, 최소 벽 길이 5 m, 코너 각도 $\approx \pi/2$). 특징 집합 $M$ 에 대해 50 m 이내 모든 쌍의 (거리, 각도) fingerprint를 미리 계산해 **SQLite/Spatialite DB**에 위치·id·종류(pole/corner/wall)와 함께 저장하고, **R\*-tree 공간 인덱스**로 빠르게 검색한다(평균 33.75 MiB/km²).

**(c) 매칭(관계대수 질의).** 검출 특징의 fingerprint를 실시간 계산해 DB와 비교한다. 한 검출 특징(query) $q$ 와 한 맵 특징의 fingerprint 일치 여부는 **거리·각도 허용오차** $(\epsilon_d, \epsilon_\alpha)$ 안에 드는 튜플이 하나라도 있는지로 본다(식 12).

$$p = \bigvee_{i=0}^{n}\big(\,|q_{d_i}-f_d|<\epsilon_d \;\wedge\; |q_{\alpha_i}-f_\alpha|<\epsilon_\alpha\,\big)$$

점 특징 인식 질의(식 13)는 **반경 $r$ 내·동일 종류·$p$ 를 만족**하는 맵 특징에 대해, 일치하는 fingerprint 개수·id·위치를 반환한다.

$$\sigma_{\,d(f,q)<r \,\wedge\, f_t=t \,\wedge\, p}\big(\Sigma(\mathrm{id}),\,\mathrm{id},\,x,\,y\big)$$

에지 질의(식 14)는 다중 매칭을 피하기 위해 **다른 특징들의 위치가 $\epsilon_{oR}$ 보다 가깝다**는 조건과 에지 시작/끝점 선택을 추가한다. **가장 많은 fingerprint가 일치하는 맵 특징**을 해당 검출 특징의 대응으로 연관한다(정렬 리스트의 최상위). 마지막으로 매칭된 특징들의 **위치 offset에 median filter**를 적용해 분산을 줄인다. 계산량을 낮추기 위해 매칭 시 맵 검색 반경 $r$ 은 보통 2–5 m로 제한한다.

### 3. Residual 정의 (Point-to-Point Error Metric)

데이터 연관이 끝나면, 검출 특징(차량 국소 좌표) $q_i$ 와 그에 대응된 맵 특징(전역 좌표) $m_i$ 의 **쌍 집합** 이 생긴다. 이 논문은 ICP에서 쓰는 **표준 point-to-point 오차 척도** 를 residual로 정의한다. 즉 강체 변환 $(R, t)$ 로 검출 특징을 맵 특징에 정렬했을 때의 **대응 쌍 간 유클리드 거리 제곱합**이다.

$$E(R,t) = \sum_{i} \big\lVert\, (R\,q_i + t) - m_i \,\big\rVert^2$$

(median filter로 다듬은) 매칭 특징 쌍의 위치 offset이 이 residual의 실체이며, 벽(에지)은 횡방향 성분으로만 기여한다.

### 4. Optimization (SVD 닫힌해 + EKF 융합)

residual $E(R,t)$ 는 **PCL의 SVD 기반 구현(TransformationEstimationSVD)** 으로 **닫힌 형식(closed-form)** 최소화한다(ICP의 point-to-point와 동일 척도, Arun/Umeyama류). 즉 반복 최적화가 아니라 대응 쌍으로부터 한 번에 최적 강체 변환 $(R,t)$ 를 구한다.

이 변환 추정치는 기존 위치추정을 **보정**하는 데 쓰인다. 최종적으로 초기 위치·휠 오도메트리·IMU를 **EKF(ROS `robot_localization`)** 로 융합하여 매끄럽고 강건한 최종 pose를 만든다. 정리하면 프레임마다 **(연관) → point-to-point residual → SVD 닫힌해 → EKF 융합** 으로 동작한다.

---

## Geometric Fingerprinting의 특징

1. pole·벽·코너를 점/에지 특징으로 추출하고(scan-line 검출기 + RANSAC), 주변 특징들과의 (거리, 각도) 집합인 **기하 fingerprint**로 식별한다.

2. 데이터 연관은 **관계대수 fingerprint 질의**(거리·각도 허용오차 + 동일 종류 + 반경 $r$)로 수행하며, 가장 많은 fingerprint가 일치하는 맵 특징을 대응으로 택하고 median filter로 정제한다.

3. residual은 연관된 특징 쌍의 **point-to-point 오차(유클리드 거리 제곱합)** 로 정의한다.

4. 최적화는 **SVD 닫힌해**로 강체 변환을 구한 뒤 **EKF**로 오도메트리·IMU와 융합한다.

5. feature map을 SLAM이 아니라 **공개 도시 데이터**로부터 생성한다(자체 매핑 불필요).

---

## Geometric Fingerprinting의 장점

1. 공개 데이터로 대규모(수 km²) feature map을 만들 수 있어 비용 큰 자체 SLAM 매핑이 불필요하다.

2. fingerprint 다수결 매칭 + median filter로 **지도 부정확성·누락/추가 특징에 강건**하다(평균 lateral 7.3 cm, 360° 스캔당 44.7 ms).

3. 알고리즘이 특정 센서에 종속되지 않아(스테레오/모노+깊이 등) 다른 센서로 확장 가능하다.

---

## Geometric Fingerprinting의 단점

1. 초기 pose의 **북 방향을 ±10° 이내로 알아야** 매칭이 가능하다(거리 필드만 매칭하면 우회 가능하나 초기화 시간 증가).

2. 벽은 무한직선·각도 기반이라 fingerprint 고유성이 약하고 **횡방향 보정에만** 기여한다.

3. pole·벽이 희소한 매우 좁은 골목 등에서는 가용 특징이 줄어 매칭 신뢰도가 떨어지며, 점/에지 같은 **저차원 기하 특징**이라 단조로운 환경에서 변별력이 제한된다.

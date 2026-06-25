# TripletLoc: One-Shot Global Localization Using Semantic Triplet in Urban Environments

Ma, W., Yin, H., Wong, P. J. Y., Wang, D., Sun, Y., & Su, Z. (2025). TripletLoc: One-shot global localization using semantic triplet in urban environments. _IEEE Robotics and Automation Letters_, 10(2), 1569-1576.

---

## Abstract

TripletLoc은 한 장의 LiDAR scan을 **대규모 reference map에 직접 정합**하여 6-DoF 전역 위치를 빠르고 강건하게 추정하는 시스템이다. place recognition 후 점군 정합을 거치는 기존 2단계 방식과 달리, **경량 의미정보(semantics) 위에서 곧바로 대응을 생성**한다(인간이 객체로 장소를 인지하는 방식에 가까움).

핵심 흐름: 질의 scan과 reference map에서 각각 인스턴스를 추출해 두 semantic graph를 만들고, **의미 triplet 기반 히스토그램 descriptor**로 인스턴스 수준 매칭을 수행한다. 원시 대응에서 **graph-theoretic outlier pruning**으로 inlier를 선별해 강건한 6-DoF pose를 추정하고, 새로운 **RSN(Road Surface Normal) map**으로 사전 회전 제약을 더해 정확도를 높인다. 이전 연구 Triplet-Graph(scan-to-scan)를 scan-to-map으로 확장해 메모리 효율을 개선하였다.

---

## Introduction

전역 위치추정은 초기 추정 없이 사전 맵에 대해 로봇 위치를 찾는 문제다. GNSS는 도심에서 신뢰성이 떨어져, LiDAR 기반 전역 위치추정이 필요하다.

대규모 맵에서 3D 점 수준 대응으로 직접 정합하는 것은 계산적으로 거의 불가능하다. 그래서 여러 연구가 인스턴스를 추출해 대응을 만들지만, all-to-all 전략은 대규모 환경에서 대응 수가 폭증해 outlier pruning이 느려지는 문제가 있다.

TripletLoc은 (1) 인스턴스를 맵에 **한 번만 저장하는 scan-to-map** 구조로 메모리를 절감하고, (2) RANSAC 대신 **graph-theoretic pruning + Truncated Least Squares(TLS)** 정합으로 outlier에 강건하며, (3) 의미·위상·기하를 통합한 **정보량 높은 정점 descriptor**를 설계하고, (4) 도로 표면 법선으로 회전 제약을 주는 **RSN map**을 제안한다.

---

## Method

TripletLoc은 **인스턴스 맵 + RSN 맵 구축 → 정점 descriptor 추출·매칭 → 강건 6-DoF pose 추정**으로 구성된다.

### 1. Instance-Level Map & RSN Map

- **Instance-level map**: 각 scan에 사전학습 **SPVNAS**로 semantic segmentation(파인튜닝 없음)을 적용하되, long-term에 안정적인 **trunk·pole·traffic sign** 클래스만 사용한다. 동일 클래스 점을 Euclidean Cluster로 군집화하고, 여러 scan의 인스턴스를 상대 pose로 같은 좌표계에 변환·연결한 뒤 0.5 m 미만으로 가까운 인스턴스는 하나로 융합한다. 각 인스턴스는 ID·클래스·중심(centroid)·점 개수를 가진다.
- **RSN map**: 도로 점을 10 m×10 m grid로 나누고, grid 내 점이 임계값을 넘으면 RANSAC으로 도로 평면을 추정해 표면 법선을 근사한다(도시 도로의 국소 평탄성 가정). 여러 scan의 법선을 융합하고 상대 각도로 불확실성(표준편차)을 산출한다.

### 2. 정점 Descriptor 추출 & 매칭

- **Semantic graph**: 인스턴스를 정점으로, 거리가 $\tau_{edge}$(=20 m) 미만인 두 정점을 간선으로 연결한다.
- **Vertex descriptor**: 정점 $v_j$ 를 중간으로 하는 triplet들에서 (i) 위상(의미 조합)+기하(상대 각도)를 인코딩한 $\mathrm{Des}^{\alpha}_{v_j}$ (Triplet-Graph 계승)와, (ii) triplet 평균 간선 길이 $d=(d_{ij}+d_{jk})/2$ 를 인코딩한 $\mathrm{Des}^{d}_{v_j}$ 를 결합해 최종 descriptor $\mathrm{Des}_{v_j}$ 를 만든다.
- **Vertex matching**: **같은 클래스 정점끼리만** 코사인 유사도로 비교하고, 각 질의 정점에 대해 **top-k(=25)** 를 선택해 원시 대응 $A_{raw}$ 를 얻는다(K-D tree 가속).

### 3. 강건 6-DoF Pose 추정

- **Graph-theoretic outlier pruning**: $A_{raw}$ 로 구성한 consistency graph에서 **Maximum Clique(MCQ)** 를 탐색(PMC 라이브러리)해 inlier 대응 $A$ 를 선별한다.
- **Point-to-point registration (TLS)**: 선별된 대응으로 변환 $T=[R,t]$ 를 **Truncated Least Squares**(TEASER 계열)로 추정하여 잔여 outlier에 강건하게 한다.
- **RSN 회전 제약**: point-to-point 대응이 회전 제약을 충분히 못 줄 때를 대비해, RSN map의 도로 표면 법선으로 **사전 회전 제약**을 부여해 pose 추정을 안정화한다.

---

## TripletLoc의 특징

1. place recognition + 정합의 2단계를 거치지 않고, 의미 인스턴스 위에서 직접 대응을 생성하는 one-shot scan-to-map 전역 위치추정이다.

2. 의미·위상·기하를 통합한 triplet 히스토그램 정점 descriptor와, 도로 법선 기반 RSN map의 사전 회전 제약을 제안한다.

3. Maximum Clique 기반 outlier pruning + TLS 정합으로 대규모 맵에서도 강건하고 계산 가능한 6-DoF pose를 얻는다.

4. long-term 안정 클래스(trunk·pole·sign)만 사용해 장기 위치추정에 유리하다.

---

## TripletLoc의 장점

1. scan-to-map 구조로 인스턴스를 맵에 한 번만 저장해 메모리 효율이 높다.

2. top-k 매칭으로 대응 수를 제한해 대규모 환경에서 실시간성에 유리하다(descriptor 추출 ~3.75 ms).

3. 안정 의미 클래스 + 도로 법선 제약으로 동적/시점 변화에 강건하다.

---

## TripletLoc의 단점

1. RSN map이 **평탄한 실외 도로의 지상 로봇** 에만 적용 가능하다(실내·기복 지형 미지원).

2. semantic segmentation 품질과 인스턴스 수에 의존하여, 다리 위처럼 반복적·인스턴스 빈약 장면에서는 성능이 저하된다.

3. vertex matching과 MCQ 탐색이 처리 시간의 대부분을 차지한다.

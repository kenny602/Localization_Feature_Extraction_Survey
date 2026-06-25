# LSMCL: Long-term Static Mapping and Cloning Localization for Autonomous Robot Navigation using 3D LiDAR in Dynamic Environments

- **저자**: Yu-Cheol Lee
- **게재처**: Expert Systems With Applications 241 (2024) 122688
- **연도**: 2024
- **분류**: Geometric Primitive (동적 환경, 사전 static map 기반)

---

## Abstract

LSMCL은 주차장처럼 주변 객체가 자주 움직이는 동적 환경에서, **변하지 않는 정적 구조(벽·기둥 등 자연 랜드마크)만으로** 실시간 6-DoF 전역 위치추정을 수행하는 시스템이다. 인공 랜드마크 설치가 필요 없고, 3D LiDAR 한 종류로 매핑과 위치추정을 모두 처리한다.

시스템은 두 축으로 구성된다. **LSM(Long-term Static Mapping)** 은 동적 객체를 걸러내고 장기적으로 위치가 고정된 객체만 담은 2D grid map과 3D 기하 특징 맵을 만든다. **CL(Cloning Localization)** 은 이 맵 위에서, 2D particle filter로 초기 위치를 잡은 뒤 이를 3D로 복제(clone)하여 3D 기하 특징 정합으로 정밀 위치를 추적한다. 고동적 주차장 실험에서 초기 위치추정 성공률·정확도·처리시간·혼잡공간 적용성을 검증하였으며, 정적 특징 맵을 쓸 때 동적 객체를 포함한 맵보다 위치추정 실패가 크게 줄어듦을 보였다.

---

## Introduction

동적 환경에서의 위치추정은 자율 로봇의 핵심 과제다. 기존 해법과 한계를 정리하면:

- **인공 랜드마크(beacon·marker·reflector)**: 동적 환경에서 안정적이나 설치·유지·교체 비용이 크고, 공간 레이아웃이 바뀌면 다시 설치해야 한다.
- **비전 카메라**: 접근성은 좋으나 조명 변화·문맥 부족 환경에서 불안정하다.
- **2D LiDAR**: 조명에 강건하지만 동적 환경에서 측정 데이터가 부족해 실패하기 쉽다.
- **3D LiDAR**: 위 둘의 장점을 결합해 공간 구조 특징을 안정적으로 뽑을 수 있어 유리하다.

LSMCL의 핵심 아이디어는 **"정적 객체 인식을 매핑 과정에 녹여, 위치추정에 필요한 정적 정보만 담은 맵을 만든다"** 는 것이다. 일반적인 객체 인식 기법을 그대로 쓰면 처리시간 오버헤드와 오인식이 위치추정 오류로 이어지므로, 객체 인식과 매핑을 결합·최적화하여 동적/정적을 분리하고 정적 구조만 맵에 남긴다. 그 위에서 CL이 2D→3D 복제 방식으로 6-DoF 위치를 추정한다.

---

## Method

LSMCL은 **3D SLAM**, **static map generator**, **global estimator** 세 모듈로 구성되며, 매핑은 LSM 흐름, 위치추정은 CL 흐름을 따른다.

### 1. 특징점 추출 (Feature Extraction)

LiDAR의 거리·방위로부터 로컬 좌표 점 $\mathbf{p}_{(v,h)}$ 를 계산한다($V$개 수직 채널 × 채널당 $H$개 수평 점). 모든 점을 쓰면 실시간성이 떨어지고 동적 객체에 취약하므로, 벽·코너 같은 핵심 기하를 정의하는 점만 선택한다.

같은 수직 채널의 인접 점쌍으로 곡률(curvature) $\kappa$ 를 계산한다:

$$\kappa^{(a,b)}_{(v,i)} = \frac{\lVert \mathbf{p}_{(v,a)}, \mathbf{p}_{(v,b)} \rVert}{\lVert \mathbf{p}_{(v,i)} \rVert}, \quad i = \tfrac{a+b}{2}, \;\; |a-b| < \tfrac{H}{4}$$

코너가 보통 수직 형태임을 고려해 인덱스 차를 $H/4$ 이내로 제한한다. $\kappa$ 가 코너 임계값 $\kappa_c$ 보다 크면 **corner point**, planar 임계값 $\kappa_p$ 보다 작으면 **planar point** 로 분류한다. 단, 이미 뽑힌 점들과 각각 최소 거리 $d_c$ / $d_p$ 이상 떨어진 경우에만 채택해 한 영역에서 특징이 중복 생성되는 것을 막는다.

### 2. 기하 특징 생성 (Geometrical Feature)

점 정보만으로는 공간 구조를 정의하기 어려우므로 점 특징을 묶어 기하 특징을 만든다.

- **Corner line** $\mathbf{e}_k = (\mathbf{p}^c_k, \mathbf{u}^c_k)$: 인접한 두 corner point의 중심점과 단위 방향벡터로 정의되는 선분.
- **Planar surface** $\mathbf{s}_k = (\mathbf{p}^s_k, \mathbf{n}^s_k)$: 인접한 세 planar point의 중심점과 법선벡터로 정의되는 면.

### 3. Scan / Map Matching

현재 프레임의 점 특징을 누적된 기하 특징 맵 $(\bar{\mathbf{E}}^m, \bar{\mathbf{S}}^m)$ 과 정합하여 6-DoF 위치를 추정한다. 추정 위치로 현재 특징을 전역 좌표계로 변환해 맵에 누적하되, **voxel filter** $f_v(\cdot)$ 로 동일 기하 특징의 중복 저장을 막아 인접 특징 탐색의 계산 효율을 높인다.

### 4. Static Map Generator

추정 전역 위치로 LiDAR 점을 전역 좌표계로 변환해 point cloud를 누적하고 **voxel map** 을 구성한다. 이후 **GAF(Grid Association Filter)** 가 voxel map의 occupancy 확률값을 이용해, 장기적으로 위치가 변하지 않는 객체만 남긴 **정적 grid map과 정적 feature map** 을 생성한다. 이 과정에서 보행자 이동이 남긴 corner line 자취 등은 동적 객체로 필터링된다(실험: 보행자로 생긴 다수의 corner line이 제거되어 corner line 2,475개·planar surface 3,904개만 남음).

### 5. Cloning Localization (CL)

정적 맵과 LiDAR 데이터로 6-DoF 전역 위치를 coarse-to-fine으로 추정한다.

1. **2D 전역 위치(particle filter)**: grid map의 객체 위치와 2D로 투영한 LiDAR 데이터로 particle filter가 2D 전역 위치를 찾는다.
2. **Cloning**: 추정된 2D 위치를 3D 좌표계의 초기 6-DoF 위치로 복제한다.
3. **3D map matching**: 초기 위치와 정적 기하 특징으로 3D map matching을 수행해 정밀 6-DoF 위치를 추정한다.
4. 동적 환경에서의 6-DoF 전역 위치를 출력한다.

---

## LSMCL의 특징

1. 매핑(LSM)과 위치추정(CL)을 기능 단위로 모듈화·통합하여, 정적 맵 생성부터 위치추정까지 하나의 시스템으로 동작한다.

2. 곡률 기반 corner/planar 점을 추출하고, 이를 corner line·planar surface 기하 특징으로 묶어 공간 구조를 표현한다.

3. GAF로 voxel occupancy 확률을 이용해 장기 정적 객체만 남긴 static feature/grid map을 만든다.

4. 2D particle filter 초기화 → 3D cloning → 3D 기하 특징 정합으로 이어지는 coarse-to-fine 6-DoF 위치추정 구조를 제안한다.

---

## LSMCL의 장점

1. 주차장·보행자 혼잡 등 동적 환경에서 정적 맵만으로 안정적인 6-DoF 위치추정이 가능하며, 동적 객체를 포함한 맵 대비 위치추정 실패가 크게 줄어든다.

2. 정적 구조만 남겨 맵이 경량이고, 매핑 시점과 환경이 달라져도 장기적으로 재사용할 여지가 크다.

---

## LSMCL의 단점

1. 특징점 추출이 **동일 수직 채널의 곡률** 에 기반하므로 spinning형 LiDAR의 스캔 구조(채널·행 단위)에 의존적이다. 즉 센서 타입에 따라 추출 방식이 달라질 수 있다.

2. 수평 측정이 수직보다 많은 일반 3D LiDAR 특성상 수직(높이) 위치 정확도가 수평보다 낮아, 높이 변화가 적은 실내·평탄 환경에 더 적합하다(드론 등 높이 민감 응용에는 dome형 LiDAR 권장).

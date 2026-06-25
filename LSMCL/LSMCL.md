# LSMCL: Long-term Static Mapping and Cloning Localization

- **저자**: Yu-Cheol Lee
- **게재처**: Expert Systems With Applications 241 (2024) 122688
- **연도**: 2024
- **분류**: Geometric Primitive (동적 환경, 사전 static map 기반)

---

## Abstract

자율 로봇 내비게이션에서 주변 객체의 위치가 자주 바뀌는 **동적 환경에서 정확한 위치를 인식하는 것**은 핵심 난제이다. 본 연구는 3D LiDAR만으로 동적 환경에서도 **자연 랜드마크(natural landmark)만을 이용해** 실시간으로 정확한 위치를 추정하는 LSMCL(Long-term Static Mapping and Cloning Localization)을 제안한다.

LSMCL은 두 모듈로 구성된다. **LSM(Long-term Static Mapping)** 은 공간에서 위치가 변하지 않는 객체에 대한 2D grid map과 3D 기하 특징 맵(geometric feature map)을 생성한다. **CL(Cloning Localization)** 은 생성된 2D grid map과 particle filter로 초기 단계의 2D 전역 위치를 추정한 뒤, 이 2D 위치를 3D 공간으로 복제(clone)하고 3D feature map과 map matching을 통해 위치를 추적한다. 고동적(highly dynamic) 주차장에서의 로봇 내비게이션 실험을 통해 초기 위치추정 성공률, 정확도/정밀도, 처리 시간, 혼잡 공간 적용성 측면에서 실제 환경 적용 가능성을 검증하였다.

---

## Introduction

지능형 로봇은 동적 환경에서 동작 가능한 매핑·위치추정 기술을 필요로 한다. 기존에 널리 쓰이는 **인공 랜드마크 기반** 기법(beacon, tag, marker, reflector 등)은 동적 공간에서 일관된 성능을 주지만, 설치·유지보수·교체 비용이 크고 공간 레이아웃이 바뀌면 새로 설치해야 하는 한계가 있다.

이에 따라 로봇에 기본 탑재되는 **비전 카메라나 LiDAR**를 활용한 자연 마커 기반 위치추정이 연구되어 왔다. 비전은 접근성이 좋으나 조명 변화나 문맥 정보 부족 환경에서 불안정하고, 2D LiDAR는 조명에 강건하나 동적 환경에서 측정 데이터가 부족해 실패하기 쉽다. 3D LiDAR는 이 둘의 장점을 결합하여 공간 구조의 특징을 비교적 쉽게 추출할 수 있어 최근 각광받는다.

본 연구는 동적 환경에서도 안정적인 전역 위치를 제공하기 위해, **위치가 변하지 않는 정적 자연 랜드마크(벽, 기둥 등 공간 구조)만으로 맵을 만들고**, 그 맵 위에서 위치추정을 수행하는 LSMCL을 제안한다. 핵심 아이디어는 **정적 객체 인식 기능을 매핑 과정에 반영**하여, 위치추정에 필요한 정적 객체 정보만 담은 맵을 생성하는 것이다. 일반 목적의 객체 인식 기법을 그대로 쓰면 위치추정 처리 시간 측면에서 오버헤드가 되고 오인식이 위치추정 오류로 이어질 수 있으므로, 객체 인식과 매핑을 결합·최적화한다.

> **본 연구(사용자)와의 관련성**: 정적 feature만 남겨 동적 환경에서 localization을 성공시킨다는 문제의식이 사용자 연구와 동일하다. 단, 특징점 추출이 **vertical channel 곡률(curvature) 기반**이라는 점에서, 포인트 분포 기반·센서 비의존을 지향하는 사용자 방식과 대비된다.

---

## Method

LSMCL은 (1) 3D SLAM, (2) static map generator, (3) global estimator로 구성되며, LSM과 CL의 두 흐름으로 동작한다.

### 1. 3D SLAM — 특징점/기하 특징 추출

LiDAR의 거리·방위 정보로부터 로컬 좌표의 3D 점 `p(v,h)`를 계산한다(수직 V채널 × 수평 H포인트). 모든 점을 쓰면 실시간성이 떨어지고 동적 객체에 취약하므로, **벽·코너 같은 핵심 기하를 정의하는 점 특징만** 선택한다.

- **점 특징 추출**: 동일 수직 채널 상의 인접 점쌍을 이용해 곡률값 `κ`를 계산한다. `κ`가 corner 임계값 `κc`보다 크면 **corner point**, planar 임계값 `κp`보다 작으면 **planar point**로 분류한다. 이때 이미 뽑힌 corner/planar 점들과 각각 최소 거리 `dc`/`dp` 이상 떨어진 경우에만 채택하여 특정 영역의 중복 특징 생성을 방지한다.
- **기하 특징 생성**: 점 정보만으로는 공간 구조를 정의하기 어려우므로, 인접 corner 점들로 **corner line `E`**(중심점 + 단위 방향벡터)를, 인접 planar 점 3개로 **planar surface `S`**(중심점 + 법선벡터)를 구성한다.
- **정합**: scan matching과 map matching으로 6-DoF 위치를 추정한다. 누적 feature map은 voxel filter `fv`로 동일 기하 특징의 중복 저장을 막아 인접 특징 탐색의 계산 효율을 높인다.

### 2. Static Map Generator — 정적 맵 생성

추정된 전역 위치로 LiDAR 점을 전역 좌표계로 변환해 point cloud를 누적하고 **voxel map**을 구성한다. 그 후 **GAF(Grid Association Filter)** 가 voxel map의 occupancy 확률값을 이용해, 장기적으로 변하지 않는 정적 객체만 남긴 **정적 grid map과 정적 feature map**을 생성한다. (예: 보행자 이동으로 생긴 corner line 자취는 동적 객체로 걸러진다.)

### 3. Cloning Localization (CL)

생성된 맵과 LiDAR 데이터로 전역 위치를 추정한다.

1. **2D 전역 위치(particle filter)**: grid map의 객체 위치와 2D로 투영한 LiDAR 데이터를 이용해 global estimator의 particle filter가 2D 전역 위치를 찾는다.
2. **Cloning**: 추정된 2D 전역 위치를 3D 좌표계의 초기 6-DoF 위치로 복제한다.
3. **3D map matching**: 초기 위치와 정적 feature를 이용한 3D map matching으로 전역 위치를 추정한다.
4. 동적 환경에서의 6-DoF 전역 위치를 출력한다.

### 검증 요약

256 m × 151 m 공간의 주차장에서, 차량이 간헐적으로 주차된 상태로 LSM 맵을 만들고, 차량이 가득 찬 상태에서 CL로 위치추정을 수행하여 매핑/위치추정 시점의 동적 차이를 재현했다. 보행자가 많은 혼잡 공간에서도 정적 feature map을 쓰면 위치추정에 성공한 반면, 동적 객체가 포함된 feature map을 쓰면 동일 실험에서 4회 실패하였다.

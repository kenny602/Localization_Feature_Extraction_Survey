# Long-term Vehicle Localization in Urban Environments Based on Pole Landmarks Extracted from 3D LiDAR Scans

Schaefer, A., Büscher, D., Vertens, J., Luft, L., & Burgard, W. (2021). Long-term vehicle localization in urban environments based on pole landmarks extracted from 3-D lidar scans. _Robotics and Autonomous Systems_, 136, 103709.

---

## Abstract

기둥형 객체(pole-like object)는 도시 환경에 흔하고 장기적으로 안정적이어서 차량 위치추정용 랜드마크로 적합하다. 본 연구는 3D LiDAR에서 추출한 **pole 랜드마크 기반의 완전한 매핑·장기 위치추정 시스템**을 제안한다. 새로운 pole 검출기, 매핑 모듈, 온라인 위치추정 모듈로 구성되며 오픈소스로 공개된다. 초기 맵만으로 **15개월**에 걸쳐 도심에서 이동 로봇을 위치추정하며, 경로·날씨·계절 변화와 공사 구간에 대응하는 장기 신뢰성을 입증하고, 기존 SOTA 대비 정확도 우위를 보였다.

---

## Introduction

지능형 차량은 정확하고 신뢰할 수 있는 자기 위치추정을 필요로 한다. RTK/D-GPS는 cm급이지만 도심 건물에 가려 정확도가 수 m로 떨어져 신뢰성이 부족하다.

dense map(grid·point cloud·mesh) 기반 위치추정은 신뢰성이 높지만 대규모에서 메모리가 과도하다. **랜드마크 맵**은 수십억 점을 소수의 핵심 특징으로 압축해 메모리를 수 자릿수 줄이고, **센서 모달리티로부터 추상화**되며(다른 센서로도 맵 생성·활용 가능), 계절 변화에 강건하다(잎은 변해도 줄기·기둥은 불변). pole은 가로등·표지·볼라드·나무 줄기 등으로 나타나며 흔하고 장기 안정적이고 기하가 잘 정의되어 랜드마크로 적합하다. 위치추정은 매핑(pole 검출기 + GT 궤적으로 전역 맵 구축)과 위치추정(particle filter로 온라인 pole과 맵을 정렬) 단계로 나뉜다.

---

## Method

### 1. Pole 추출 (Ray Tracing 기반)

정합된 3D scan에서, 레이저 광선의 **끝점뿐 아니라 시작점까지 고려**하여 점유(occupied)·자유(free) 공간을 명시적으로 모델링한다(핵심 차별점). 대부분 기법은 반사가 없으면 자유 공간으로 가정하지만, 반사 부재는 "실제 비어 있음"과 "가림/미관측"을 구분해야 한다. 공간을 voxel로 분할·광선 추적하여 각 voxel의 반사 사후확률을 Beta 분포로 모델링하고 occupancy map $O$ 로 변환한다. 이어 pole feature detector가 $O$ 를 지면의 **2D pole score map $S$** 로 바꾸고, pole 위치는 $S$ 의 극값을 **mean shift**(가우시안 커널)로 찾으며, 폭은 가중 평균으로 추정한다.

### 2. 매핑 (Mapping)

메모리 한계를 위해 궤적을 짧은 구간으로 나눠 순차 처리하고, 로컬 grid map을 전역 축에 정렬한다. 중첩으로 생긴 중복 pole은 지면 투영 후 중심·폭의 가중 평균으로 병합한다. **슬라이딩 윈도우 동적 필터링**: 로컬 랜드마크가 과거 $w$ 개 로컬 맵에서 최소 $c$ 회 이상 관측될 때만 전역 맵에 통합하여 동적 객체를 랜드마크 수준에서 제거한다.

### 3. 위치추정 (Localization)

**Particle filter**로 위치추정한다. 각 particle은 2D pose 가설(3×3 동차변환)이며, 상대 odometry $\chi=[x,y,\phi]$ 에 가우시안 노이즈를 가정해 전파한다. 측정 단계에서는 온라인 랜드마크와 맵 랜드마크를 k-D tree 최근접으로 연관하고, 측정 모델(랜드마크 거리에 대한 가우시안 + 미등록 pole 발견 확률 $\epsilon$)로 particle 가중치를 갱신한다.

---

## Pole 기반 위치추정의 특징

1. 광선의 시작점·끝점을 모두 이용해 점유/자유 공간을 모델링하는 ray-tracing 기반 pole 검출기를 제안한다.

2. pole score map의 극값을 mean shift로 찾아 2D pole 위치·폭을 추정한다.

3. 슬라이딩 윈도우로 동적 객체를 랜드마크 수준에서 걸러 장기 안정 맵을 만든다.

4. particle filter로 온라인 pole과 맵을 정렬해 장기 위치추정을 수행한다(15개월 검증).

---

## Pole 기반 위치추정의 장점

1. 랜드마크 맵이 매우 경량이고 센서 모달리티에 독립적이며, 계절·날씨 변화에 강건해 장기 위치추정에 유리하다.

2. 자유 공간 모델링으로 가림/미관측을 구분해, 단순 반사 기반 기법보다 pole 검출이 정확하다.

---

## Pole 기반 위치추정의 단점

1. 2D 평면 투영을 전제하므로 지형 기복이 큰 환경에서는 오차가 증가한다(3D 확장이 향후 과제).

2. **pole이 부족한 환경**에서는 랜드마크가 희소해 위치추정이 어렵다. 또한 공사용 배럴 등이 pole로 오검출되면 mislocalization이 발생할 수 있다(맵 변화에 대한 취약점).

# Localization Feature Extraction Survey

> A curated survey of **feature extraction methods for localization**
> LiDAR 기반 Localization에서 사용되는 **특징(feature) 추출 기법**을 정리한 기술조사 저장소입니다.

**Maintainer:** [@kenny602](https://github.com/kenny602)
**Status:** 🚧 Work in progress · **Last updated:** 2026-06-25

---

## 📌 About

**전역 위치인식(global localization)을 위한 3차원 라이다 특징점 기술조사.**

이미 구축된 맵 위에서의 위치추정에 한정하며, 특징점을 **"어떤 표현(representation) 위에서 추출/기술하는가"** 를 1차 분류 축으로 삼는다.

각 논문은 `Abstract / Introduction / Method` 3단 흐름으로 정리한다.

---

## 분류 체계 (Category) — 1차 축: 특징 표현

| # | 범주 | 특징을 뽑는 표현 | 핵심 아이디어 | 대표 연구 |
|---|---|---|---|---|
| ① | **Point-based** (점 기반) | 원시/학습된 점 | 점 단위 특징을 학습·집계해 전역 descriptor 생성 | PointNetVLAD, MinkLoc3D |
| ② | **Voxel / Distribution-based** (복셀·분포 기반) | voxel · NDT cell | 셀 내 점 분포(공분산·확률밀도)로 특징 기술 | NDT, NDT-Transformer |
| ③ | **Geometric Primitive** (기하 사전 정보) | edge/corner · plane/surface · surfel | 구조적 기하 요소를 추출해 정합 | LSMCL |
| ④ | **Segment / Semantic / Graph** (세그먼트·의미·그래프) | 세그먼트 · 인스턴스 객체 · 그래프 | 고수준 객체/관계를 descriptor로 | SegMatch, TripletLoc, STD, BTC |
| ⑤ | **Projection-based** (투영 기반) | BEV · range image · spherical | 2D 투영 후 영상 descriptor 적용 | Scan Context++, M2DP, OverlapNet |
| ⑥ | **Landmark-based** (랜드마크 기반) | pole · tree · 구조물 | 시간 불변 랜드마크를 맵으로 | Pole(Schaefer), TreeLoc |
| ⑦ | **Dense Cell-Occupancy Residual** (Occupancy Cell의 probability와 intensity residual을 optimize |

### 범주별 특성 비교

| 범주 | 공간정보 보존 | 센서타입 의존성 | 동적/장기 강건성 | 메모리 | 계산 효율 |
|---|---|---|---|---|---|
| ① 점 기반 | 높음(원시 점) | 중 | 낮음(학습 분포 의존) | 큼 | 학습 추론 비용 |
| ② 복셀·분포 | 중(해상도 의존) | **낮음** | 중 | 중 | 비교적 빠름 |
| ③ 기하 사전 정보 | 중 | 추출 방식에 따라 상이 | 중~높음(정적 구조) | 작음 | 빠름 |
| ④ 세그먼트·의미·그래프 | 고수준 | 중(분할 품질 의존) | **높음(안정 클래스 선택)** | 작음 | 분할 비용 |
| ⑤ 투영 기반 | 투영 손실 | 중 | 중 | 작음 | **빠름(2D CNN)** |
| ⑥ 랜드마크 | 희소 | **낮음(모달리티 추상화)** | **높음(시간 불변)** | **매우 작음** | 빠름 |
| ⑦ Occupancy Cell의 probability와 intensity residual을 optimize | 높음(dense) | 중(intensity 민감) | 높음(맵 갱신) | 큼(dense map) | 중

> 위 표는 정성적 경향이며 개별 기법에 따라 달라질 수 있다.

---

## 범위 (Scope)

- **포함**: 사전 맵 기반 localization + place recognition / global localization descriptor
- **제외**: odometry / SLAM / mapping(dynamic 제거 포함). 단, 특징 추출 기법 비교상 필요한 경우 본문에서 참조만 함.
- **연대**: 원칙적으로 최근 7년(2019~). 단 ①②⑤의 원조(PointNetVLAD'18, M2DP'16, NDT 등)는 "원조 참고"로 포함.

---

## 논문 색인 (Index)

### ① Point-based
| 파일 | 논문 | 게재처 | 연도 | 상태 |
|---|---|---|---|---|
| `1_Point-based/PointNetVLAD.md` | PointNetVLAD: Deep Point Cloud Based Retrieval for Large-Scale Place Recognition | CVPR | 2018 | ⬜ |
| `1_Point-based/MinkLoc3D.md` | MinkLoc3D: Point Cloud Based Large-Scale Place Recognition | WACV | 2021 | ⬜ |

### ② Voxel / Distribution-based
| 파일 | 논문 | 게재처 | 연도 | 상태 |
|---|---|---|---|---|
| `2_Voxel-Distribution-based/NDT-Transformer.md` | NDT-Transformer: Large-Scale 3D Point Cloud Localisation using NDT Representation and Transformer | ICRA | 2021 | ⬜ |
| `2_Voxel-Distribution-based/NDT-PlaceRecognition.md` | Appearance-Based Place Recognition using NDT (Magnusson et al.) | 원조 참고 | 2009 | ⬜ |

### ③ Geometric Primitive
| 파일 | 논문 | 게재처 | 연도 | 상태 |
|---|---|---|---|---|
| `3_Geometric-Primitive-based/LSMCL.md` | LSMCL: Long-term Static Mapping and Cloning Localization | Expert Systems With Applications | 2024 | ✅ |

### ④ Segment / Semantic / Graph
| 파일 | 논문 | 게재처 | 연도 | 상태 |
|---|---|---|---|---|
| `4_Segment-Semantic-Graph-based/SegMatch.md` | SegMatch: Segment Based Place Recognition in 3D Point Clouds | ICRA | 2017 | ⬜ |
| `4_Segment-Semantic-Graph-based/TripletLoc.md` | TripletLoc: One-Shot Global Localization Using Semantic Triplet | RA-L | 2025 | ✅ |
| `4_Segment-Semantic-Graph-based/TripletGraph.md` | Triplet-Graph: Global Metric Localization via Semantic Triplet Graph | RA-L | 2024 | ⬜ |
| `4_Segment-Semantic-Graph-based/STD.md` | STD: A Stable Triangle Descriptor for 3D Place Recognition | ICRA | 2022 | ⬜ |
| `4_Segment-Semantic-Graph-based/BTC.md` | BTC: A Binary and Triangle Combined Descriptor for 3D Place Recognition | T-RO | 2024 | ⬜ |
| `4_Segment-Semantic-Graph-based/SGTD.md` | SGTD: A Semantic-Guided Triangle Descriptor for One-Shot Global Localization | RA-L | 2025 | ⬜ |

### ⑤ Projection-based
| 파일 | 논문 | 게재처 | 연도 | 상태 |
|---|---|---|---|---|
| `5_Projection-based/ScanContextPP.md` | Scan Context++: Structural Place Recognition Robust to Rotation/Lateral Variations | T-RO | 2022 | ⬜ |
| `5_Projection-based/M2DP.md` | M2DP: A Novel 3D Point Cloud Descriptor for Place Recognition | 원조 참고 (IROS) | 2016 | ⬜ |
| `5_Projection-based/OverlapNet.md` | OverlapNet: Loop Closing for LiDAR-based SLAM | RSS | 2020 | ⬜ |

### ⑥ Landmark-based
| 파일 | 논문 | 게재처 | 연도 | 상태 |
|---|---|---|---|---|
| `6_Landmark-based/Pole-Landmark-Localization.md` | Long-term Vehicle Localization Based on Pole Landmarks | Robotics and Autonomous Systems | 2021 | ✅ |
| `6_Landmark-based/TreeLoc.md` | TreeLoc: 6-DoF LiDAR Global Localization in Forests | arXiv | 2026 | ⬜ |

### ⑦ Dense Cell-Occupancy Residual (특징 비추출·대비 섹션)

| 파일 | 논문 | 게재처 | 연도 | 상태 |
|---|---|---|---|---|
| '7_Dense-Cell-Occupancy-Residual/LIO-Aided-Localization.md' | LiDAR Inertial Odometry Aided Robust LiDAR Localization System in Changing City Scenes | ICRA | 2020 | ✅ |

0 Reference Surveys

파일논문게재처연도상태0_Reference-Surveys/Global-LiDAR-Localization-Survey.mdA Survey on Global LiDAR Localization: Challenges, Advances and Open ProblemsIJCV2024⬜0_Reference-Surveys/LPR-Survey.mdLiDAR-based Place Recognition for Autonomous Driving: A SurveyACM CSUR2024⬜

### 0 Reference Surveys
| 파일 | 논문 | 게재처 | 연도 | 상태 |
|---|---|---|---|---|
| `0_Reference-Surveys/Global-LiDAR-Localization-Survey.md` | A Survey on Global LiDAR Localization: Challenges, Advances and Open Problems | IJCV | 2024 | ⬜ |
| `0_Reference-Surveys/LPR-Survey.md` | LiDAR-based Place Recognition for Autonomous Driving: A Survey | ACM CSUR | 2024 | ⬜ |

---

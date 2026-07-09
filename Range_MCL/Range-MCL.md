# Range Image-based LiDAR Localization for Autonomous Vehicles
- 저자: Xieyuanli Chen, Ignacio Vizzo, Thomas Labe, Jens Behley, Cyrill Stachniss
- 게재처: 2021 IEEE International Conference on Robotics and Automation (ICRA)
- 연도: 2021
- 분류: Range Image / Mesh Map 기반 Global Localization

---
<img width="770" height="461" alt="image" src="https://github.com/user-attachments/assets/f6a770e1-29d1-473b-85ee-39f04efa2eaa" />

## Abstract

본 논문은 3D LiDAR scan으로부터 생성한 range image와 triangular mesh 기반 사전 map을 이용하여, 대규모 실외 환경에서 autonomous vehicle 또는 mobile robot의 **global localization**을 수행하는 방법을 제안한다. 기존 LiDAR localization에서 자주 사용되는 point cloud 직접 정합, handcrafted feature, semantic landmark, learning-based observation model 대신, LiDAR scan을 range image로 변환하고, 사전 triangular mesh map에서 각 particle pose에 해당하는 synthetic range image를 렌더링하여 두 range image의 차이를 기반으로 particle likelihood를 계산한다.

시스템은 Monte Carlo Localization(MCL)을 기반으로 한다. 또한 triangular mesh map과 OpenGL 기반 렌더링, tile map 구조를 사용하여 수렴 이후 LiDAR sensor frame rate에서 online pose tracking이 가능함을 보였다.

---
## Introduction

정확한 localization은 autonomous mobile system에서 필수적인 기능이며, 특히 GPS가 불안정하거나 사용할 수 없는 환경에서는 LiDAR 기반 global localization이 중요하다. 기존 LiDAR localization 방법은 beam-end model, likelihood field, ray-casting model, handcrafted feature 기반 model, 또는 learning-based observation model을 사용해왔다. 그러나 이러한 방법들은 2D LiDAR에 더 적합하거나, 정교하게 설계된 feature가 필요하거나, 학습 환경과 다른 환경·LiDAR sensor에 대한 generalization이 제한될 수 있다.

본 논문은 raw point cloud나 semantic feature를 직접 사용하는 대신, 3D LiDAR scan을 cylindrical range image로 변환하여 사용한다. Mesh map으로부터 range image를 렌더링하는 과정은 OpenGL과 같은 computer graphics 기법으로 효율적으로 수행할 수 있다. 따라서 range image와 mesh map의 조합은 3D LiDAR 기반 global localization에 적합하다.

본 논문의 핵심 아이디어는 **현재 LiDAR scan으로부터 생성된 real range image와, particle pose에서 mesh map을 바라보았을 때 렌더링되는 synthetic range image를 비교하여 observation model을 구성한다**는 것이다. 이를 Monte Carlo Localization에 통합하여 각 particle의 likelihood를 계산하고, 차량의 pose를 추정한다.

---
## Method

본 방법은 range image generation, mesh-based map representation, synthetic range image rendering, Monte Carlo Localization, range image-based observation model, tiled map representation으로 구성된다.

### 1. Range Image Generation

현재 LiDAR scan의 point cloud를 spherical projection을 통해 range image로 변환한다. 각 LiDAR point $p_i=(x,y,z)$에 대해 range와 angle을 계산한다.

$$
r = \|p_i\|_2
$$

$$
수평각 = \arctan2(y,x)
$$

$$
수직각 = \arcsin\left(\frac{z}{r}\right)
$$

이후 yaw와 pitch를 각각 range image의 수평/수직 coordinate $(u,v)$로 변환한다. 이렇게 생성된 range image는 이후 observation model에서 particle별 synthetic range image와 비교되는 실제 관측값 $z_t$로 사용된다.

즉, 3D point cloud를 직접 정합하는 대신, LiDAR scan을 2D angular grid 형태의 range image로 변환하여 localization에 사용한다.

### 2. Mesh-based Map Representation

본 논문은 환경의 사전 map $M$을 triangular mesh로 표현한다. Triangular mesh는 대규모 point cloud map보다 compact하며, particle pose에서 synthetic range image를 빠르게 렌더링할 수 있다.

Map 생성은 localization 전에 수행되는 **offline** 단계이다. 먼저 SLAM system으로부터 얻은 LiDAR scan과 map pose를 이용해 모든 point cloud를 global reference frame으로 정렬한다. 이후 Poisson Surface Reconstruction(PSR)을 사용하여 oriented point cloud로부터 triangular mesh map을 생성한다. PSR은 input point cloud 전체를 **한 번에** 고려하여 전역적으로 일관된 mesh를 생성한다.

또한 map 저장 크기를 줄이기 위해 ground segmentation을 수행한다. 각 point의 normal 방향이 지면 normal 방향과 유사한지, 그리고 point의 높이 $z$가 threshold보다 낮은지를 기준으로 ground point와 non-ground point를 labeling하고, reconstruction 후 ground mesh와 non-ground mesh를 분리한다.

Ground mesh에 대해서는 voxel 내부 vertex를 하나로 병합하고, adjacent vertex 평균 filter를 적용한 뒤 invalid edge를 제거하여 mesh를 단순화한다. 이후 단순화된 ground mesh를 non-ground mesh와 다시 결합한다. 이를 통해 mesh model size를 원래의 약 50% 수준까지 줄인다.

### 3. Rendering Synthetic Range Images

각 particle $j$는 차량의 pose hypothesis

$$
x_t^j = (x,y,\theta)_t^j
$$

를 나타낸다. 주어진 particle pose와 triangular mesh map $M$에 대해, OpenGL을 사용하여 해당 particle에서 보이는 synthetic range image $z^j$를 렌더링한다.

구체적으로, particle pose를 기준으로 mesh triangle의 vertex들을 spherical projection을 통해 range image 좌표에 투영한다. 이후 OpenGL은 투영된 vertex들이 이루는 triangle surface를 픽셀 단위로 채우며, occlusion을 고려하여 shading한다. 따라서 synthetic range image는 단순히 mesh vertex만 투영한 결과가 아니라, triangle 면 전체를 렌더링하여 각 픽셀에 해당 표면까지의 range 값을 저장한 이미지이다.

### 4. Monte Carlo Localization

본 논문은 Monte Carlo Localization(**MCL**)을 기반으로 차량의 pose posterior를 추정한다. 각 particle은 시간 $t$에서의 2D pose 후보 $(x,y,\theta)$를 의미한다. 차량이 이동하면 control input $u_t$와 motion model을 이용하여 각 particle pose를 예측하고, 이후 observation model을 이용하여 각 particle의 weight를 갱신한다.

각 particle pose에서 mesh map을 렌더링하여 expected observation, 즉 synthetic range image를 생성하고, 이를 실제 LiDAR scan에서 얻은 current range image $z_t$와 비교한다. 실제 관측과 잘 맞는 particle은 높은 weight를 얻고, 맞지 않는 particle은 낮은 weight를 얻는다. 이후 particle들은 weight distribution에 따라 resampling되며, 이 과정을 반복하면 particle들이 true pose 주변으로 수렴한다.

본 논문은 motion model 자체를 새롭게 제안하기보다는, range image 기반 observation model 설계에 초점을 둔다.

### 5. Range Image-based Observation Model

Observation model은 현재 LiDAR scan으로 생성된 real range image $z_t$와 각 particle pose에서 렌더링된 synthetic range image $z^j$를 비교하여 particle likelihood를 계산한다.

$j$번째 particle의 likelihood는 Gaussian distribution 형태로 근사된다.

$$
p(z_t \mid x_t^j, M) \propto
\exp\left(
-\frac{1}{2}
\frac{d(z_t,z^j)^2}{\sigma_d^2}
\right)
$$

여기서 $d$는 real range image $z_t$와 synthetic range image $z^j$ 사이의 difference 또는 similarity를 의미한다. 본 논문은 빠르고 사용하기 쉬운 방법을 위해 pixel-wise absolute difference의 평균을 사용한다.

$$
d(z_t,z^j) = \frac{1}{N}
\sum_{(u,v)\in\Omega_{\mathrm{valid}}}
\left| z_t(u,v) - z^j(u,v) \right|
$$

여기서 $\Omega_{\mathrm{valid}}$는 현재 range image에서 유효한 range 값을 가지는 pixel 집합이며, $N$은 valid pixel의 수이다. 즉, 실제 LiDAR range 값이 존재하는 유효한 픽셀들에 대해, real range image와 synthetic range image의 같은 pixel 위치끼리 range 차이를 계산하고 평균낸다. 차이가 작을수록 해당 particle pose에서 map을 보았을 때 실제 LiDAR 관측과 잘 일치한다는 뜻이므로 likelihood와 particle weight가 높아진다.

기존 연구에서는 observation model을 location likelihood와 heading likelihood로 분리했지만, 본 논문에서는 range image 비교 하나로 전체 state space $(x,y,\theta)$에 대한 likelihood를 한 번에 계산한다.

### 6. Tiled Map Representation

렌더링 효율을 높이기 위해 global mesh map을 tile 단위로 분할한다. 각 particle의 synthetic range image를 렌더링할 때 전체 mesh map을 모두 사용하는 것이 아니라, particle 위치와 가까운 tile에 포함된 mesh 부분만 사용한다. 이를 통해 particle별 rendering cost를 줄일 수 있다.

Tile은 localization 수렴 여부를 판단하는 데도 사용된다. 모든 particle이 최대 $N_{\mathrm{conv}}$개의 tile 안에 위치하면 localization이 수렴했다고 가정하고, 이후 pose tracking을 위해 사용하는 particle 수를 줄인다. 즉, 초기 global localization 단계에서는 map 전체에 많은 particle을 뿌려 pose를 찾고, 수렴 이후에는 적은 수의 particle과 주변 tile만 사용하여 tracking을 수행한다.

---
## 논문의 특징

- 3D LiDAR scan을 point cloud 그대로 사용하지 않고, spherical projection을 통해 range image로 변환하여 localization에 사용한다.
- 사전 map을 triangular mesh로 표현하고, OpenGL 기반 rendering을 통해 particle pose마다 synthetic range image를 생성한다.
- 현재 real range image와 particle별 synthetic range image의 pixel-wise difference를 이용하여 기존과는 새로운 MCL observation model을 제안한다.
- Feature extraction, semantic landmark, NN에 의존하지 않고 range 정보만 사용한다.
- Particle state를 사용하여 autonomous vehicle의 2D pose global localization을 수행한다.
- Tile map 구조를 사용하여 렌더링 속도와 수렴 후 online tracking 성능을 개선한다.

## 논문의 장점

- Localization 단계에서 GPS pose prior에 의존하지 않고, 3D LiDAR와 사전 mesh map만으로 **global localization**을 수행할 수 있다.
- **Range image와 mesh map**을 이용하므로 point cloud 직접 정합보다 compact하고, OpenGL rendering을 통해 synthetic observation을 빠르게 생성할 수 있다.
- Feature extraction이나 semantic segmentation, learning-based model이 필요하지 않아, 새로운 환경으로 이동할 때 추가 training data가 필요하지 않다.
- 서로 다른 환경과 서로 다른 LiDAR sensor에 대해 generalization 가능성을 보인다.
- 수렴 이후에는 tile map과 particle 수 감소를 통해 LiDAR sensor frame rate에서 **online** pose tracking이 가능하다.
- 기존 location likelihood와 heading likelihood를 분리하는 방식과 달리, 하나의 observation model로 $(x,y,\theta)$ 전체 pose likelihood를 한 번에 계산한다.

## 논문의 단점

- 사전에 구축된 triangular mesh map이 필요하므로, 새로운 환경에서는 바로 사용할 수 없다.
- Mesh map 생성은 LiDAR scan과 map pose를 이용한 offline mapping 과정에 의존한다.
- 논문에서 말하는 online operation은 localization convergence 이후의 tracking 단계에 해당하며, 초기 global localization 단계에서는 많은 particle이 필요해 계산량이 크다.
- Particle state가 $(x,y,\theta)$로 제한되므로, $z$, roll, pitch 변화가 큰 6-DoF 환경에는 그대로 적용하기 어렵다.
- Real range image와 synthetic range image의 단순 pixel-wise absolute difference를 사용하므로, 동적 객체, map 변화 등이 큰 환경에서는 likelihood가 부정확해질 수 있다.

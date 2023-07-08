# sda5f_tutorials

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

- ROS packages for YASKAWA MOTOMAN SDA5F tutorial.
- Docker for simulation and control environments for YASKAWA MOTOMAN SDA5F.

## Dependencies

### Docker build environment

- Ubuntu 22.04 (arch=amd64)
  - NVIDIA GeForce RTX3070
  	- NVIDIA Driver 470.199.02
    - CUDA 11.4
  - Docker 23.0.1
  - Docker Compose 2.4.1
  - NVIDIA Docker 2.12.0

### SDA5F with OnRobot grippers
- Ubuntu 20.04
  - ROS Noetic
    - [onrobot](https://github.com/Osaka-University-Harada-Laboratory/onrobot)
- YASKAWA Motoman SDA5F  
- OnRobot RG6
- OnRobot VG10

## Installation

```bash
git clone git@github.com:Osaka-University-Harada-Laboratory/sda5f_tutorials.git --depth 1  
cd sda5f_tutorials
COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel  
docker-compose up  
```

## Usage with docker

```bash
./utils/sda5f_pp_fake.sh
```
<img src=image/sim.gif height=200>

```bash
./utils/sda5f_pp.sh
```
<img src=image/real.gif height=200>

## Author / Contributor

[Takuya Kiyokawa](https://takuya-ki.github.io/)

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).

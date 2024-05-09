# sda5f_tutorials

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![License: BSD](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
![repo size](https://img.shields.io/github/repo-size/Osaka-University-Harada-Laboratory/sda5f_tutorials)

- ROS packages for YASKAWA MOTOMAN SDA5F tutorial.
  - [sda5f_motion_plan](/catkin_ws/src/sda5f_motion_plan): A package to execute a product display task with dual arms.
  - [sda5f_stack](/catkin_ws/src/sda5f_stack): A stack of ros packages to control the motoman sda5f.
- Docker for simulation and control environments for YASKAWA MOTOMAN SDA5F.

## Dependencies

### Docker build environment

- [Ubuntu 22.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=22.04+LTS)
  - NVIDIA GeForce RTX3070
  	- NVIDIA Driver 470.199.02
    - CUDA 11.4
  - Docker 23.0.1
  - Docker Compose 2.4.1
  - NVIDIA Docker 2.12.0

### SDA5F with OnRobot grippers

- [Ubuntu 20.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=20.04+LTS)
  - [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
    - [onrobot](https://github.com/Osaka-University-Harada-Laboratory/onrobot)
    - [Byobu](https://www.byobu.org/)
- [YASKAWA Motoman SDA5F](https://www.motoman.com/en-us/products/robots/industrial/assembly-handling/sda-series/sda5f)  
- [OnRobot RG6](https://onrobot.com/en/products/rg6-gripper)
- [OnRobot VGC10](https://onrobot.com/en/products/vgc10)

## Installation

```bash
git clone git@github.com:Osaka-University-Harada-Laboratory/sda5f_tutorials.git --depth 1  
cd sda5f_tutorials
sudo apt install byobu
COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel  
docker compose up  
```

## Usage with docker

### Simulation
- Visualizing the model
```bash
./utils/sda5f_rviz.sh
```

- Executing the moveit
```bash
./utils/sda5f_moveit_sim.sh
```

- Executing a pick-and-place demonstration
```bash
./utils/sda5f_pp_fake.sh
```
<img src=image/sim.gif height=200>

### Real robot
- Executing the moveit
```bash
./utils/sda5f_moveit_real.sh
```

- Executing a pick-and-place demonstration
```bash
./utils/sda5f_pp.sh
```
<img src=image/real.gif height=200>

## Author / Contributor

[Takuya Kiyokawa](https://takuya-ki.github.io/)

## License

Please refer to each package.xml of the ROS packages

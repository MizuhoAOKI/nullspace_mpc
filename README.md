# Nullspace MPC
**Nullspace Model Predictive Controller for a Swerve Drive Robot**

<div align="center">

[![ROS Distro: Noetic](https://img.shields.io/badge/ROS-Noetic-red.svg)](https://wiki.ros.org/noetic)
[![Docker](https://img.shields.io/badge/-Docker-EEE.svg?logo=docker&style=flat)](https://www.docker.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

<!-- eyecatch movie -->
![nullspace_mpc_anim_3drviz](./media/nullspace_mpc_anim_3drviz.gif)

</div>

## Citation
> [!NOTE]
> Nullspace MPC is a core component of my Ph.D. dissertation, which will be made publicly available on October 1, 2025. Links to the dissertation and related publications will be added here as soon as they are available.

The baseline MPPI controller is based on the following work and is available in a dedicated repository: [mppi_swerve_drive_ros](https://github.com/MizuhoAOKI/mppi_swerve_drive_ros)  
```bibtex
@inproceedings{mizuho2024iros,
  author={Aoki, Mizuho and Honda, Kohei and Okuda, Hiroyuki and Suzuki, Tatsuya},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={Switching Sampling Space of Model Predictive Path-Integral Controller to Balance Efficiency and Safety in 4WIDS Vehicle Navigation}, 
  year={2024},
  volume={},
  number={},
  pages={3196-3203},
  doi={10.1109/IROS58592.2024.10802359}}
```

## Setup

### [Option 1] Native Environment

Note: This is the recommended setup for optimal performance.

<details>
<summary>Click here to expand</summary>

1. Prerequisites
    - [Ubuntu 20.04](https://releases.ubuntu.com/focal/)
    - [ROS Noetic](https://wiki.ros.org/noetic)

2. Clone the repository.
    ```bash
    cd <path-to-your-workspace>
    git clone https://github.com/MizuhoAOKI/nullspace_mpc
    ```

3. Install packages not handled by rosdep.
    ```bash
    cd <path-to-your-workspace>/nullspace_mpc
    sudo make install_deps
    ```

4. Initialize rosdep, update it, and install ROS dependencies.
    ```bash
    cd <path-to-your-workspace>/nullspace_mpc
    sudo rosdep init # Skip if already initialized
    rosdep update
    rosdep install -y --from-paths src --ignore-src --rosdistro noetic
    ```

5. Build the project.
    ```bash
    cd <path-to-your-workspace>/nullspace_mpc
    make build
    ```

</details>  


### [Option 2] Docker Environment

#### GPU Accelerated Environment

<details>
<summary>Click here to expand</summary>

1. Prerequisites
    - [Docker](https://docs.docker.com/engine/install/ubuntu/)
        - For Ubuntu users, you can use the convenience script:
            ```bash
            curl -fsSL https://get.docker.com -o get-docker.sh
            sudo sh get-docker.sh
            ```
    - [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
        - This is required to allow Docker containers to access the host's GPU.
    - NVIDIA GPU & Driver
        - An NVIDIA GPU and a compatible driver for the base image (nvidia/cuda:12.4.1-devel-ubuntu20.04) are required.

2. Clone the repository.
    ```bash
    cd <path-to-your-workspace>
    git clone https://github.com/MizuhoAOKI/nullspace_mpc
    ```

3. Build the Docker image (first-time setup).
    ```bash
    cd <path-to-your-workspace>/nullspace_mpc
    make setup_docker_gpu
    ```

4. Run the Docker container and start a bash session inside.
    ```bash
    cd <path-to-your-workspace>/nullspace_mpc
    make run_docker_gpu
    ```

5. [Inside the docker container] Build the project.
    ```bash
    cd ~/nullspace_mpc
    make build
    ```

</details>

#### CPU-Only Environment

<details>
<summary>Click here to expand</summary>

Warning: This setup runs entirely on the CPU. Performance is significantly lower, and on my test system it was not sufficient for stable control. Use GPU or native setup whenever possible.

1. Prerequisites
    - [Docker](https://docs.docker.com/engine/install/ubuntu/)
        - For Ubuntu users:
            ```bash
            curl -fsSL https://get.docker.com -o get-docker.sh
            sudo sh get-docker.sh
            ```

2. Clone the repository.
    ```bash
    cd <path-to-your-workspace>
    git clone https://github.com/MizuhoAOKI/nullspace_mpc
    ```

3. Build the Docker image (first-time setup).
    ```bash
    cd <path-to-your-workspace>/nullspace_mpc
    make setup_docker_cpu
    ```

4. Run the Docker container and start a bash session inside.
    ```bash
    cd <path-to-your-workspace>/nullspace_mpc
    make run_docker_cpu
    ```

5. [Inside the docker container] Build the project.
    ```bash
    cd ~/nullspace_mpc
    make build
    ```

</details>


## Usage

This package supports two operation modes:

1. **Manual Goal Mode** — you set a 2D Nav Goal in RViz and the robot navigates to it.  
2. **Demo (Multi-Goal) Mode** — the robot automatically visits a sequence of goals defined in an agenda file.

---

### 1) Manual Goal Mode

Set a **2D Nav Goal** in RViz, and the robot will navigate to the goal.

- **Nullspace MPC (Proposed)**
    ```bash
    cd <path-to-your-workspace>/nullspace_mpc
    make navigation_nullspace_mpc
    ```

- MPPI (Baseline)
    ```bash
    cd <path-to-your-workspace>/nullspace_mpc
    make navigation_mppi
    ```

### 2) Demo (Multi-Goal) Mode

Runs an evaluation script that automatically sends multiple goals in sequence (defined in `data/eval_demo/agenda.yaml`) and logs results to `./result/`.

- **Nullspace MPC (Proposed)**
    ```bash
    cd <path-to-your-workspace>/nullspace_mpc
    make eval_demo_nullspace_mpc
    ```

- MPPI (Baseline)
    ```bash
    cd <path-to-your-workspace>/nullspace_mpc
    make eval_demo_mppi
    ```

> [!NOTE]
> Due to the asynchronous nature of ROS simulations and the sampling-based algorithm relying on multi-threading computation, the controllers' performance can vary depending on your system environment.

## License & Acknowledgements

The majority of this project is licensed under the MIT License.

However, the core QP solving capability is provided by [QpSolverCollection](https://github.com/isri-aist/QpSolverCollection), which is included in the `src/third_party` directory and is licensed under the BSD 2-Clause License. See `src/third_party/QpSolverCollection/LICENSE` for details.

We are deeply grateful to the authors of [isri-aist/QpSolverCollection](https://github.com/isri-aist/QpSolverCollection) for developing and open-sourcing the useful library.

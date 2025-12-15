네, 스크린샷을 보니 코드 블록이 별도로 렌더링되면서 복사하기 불편하게 표시된 것 같습니다. **`README.md` 전체 내용을 하나의 박스 안에 처음부터 끝까지 끊김 없이** 담아드리겠습니다.

오른쪽 상단의 **'Copy'** 버튼을 누르시면 전체 내용을 한 번에 복사하실 수 있습니다.

````markdown
# Isaac Sim Duckiebot Autonomous Driving Project 🤖

This project implements a **Duckiebot autonomous driving system** using **NVIDIA Isaac Sim** and **ROS2**. It features a full USD-based robot model, a sensor processing pipeline via OmniGraph, and a computer vision-based reactive control algorithm to track a target object (Red Cube).

> **Project Report**: [Download PDF](link-to-your-pdf-if-available)  
> **Author**: Geonwoo Yoo (Dept. of AI Applications)  
> **Date**: 2025.12.15

---

## 📌 Project Overview
The goal of this project is to build a high-fidelity digital twin of a Duckiebot and implement autonomous navigation logic without physical hardware.
- **Simulator**: NVIDIA Isaac Sim 4.5.0
- **Middleware**: ROS2 Humble
- **OS**: Ubuntu 22.04 (WSL2)
- **Key Tech**: USD Modeling, OmniGraph, OpenCV, P-Controller

## ✨ Key Features
1.  **Custom Robot Modeling**:
    - Modeled Duckiebot using **USD (Universal Scene Description)** format.
    - Configured differential drive physics (Chassis, Left/Right Wheels) and sensors (Camera, LED Sphere).
2.  **Action Graph (OmniGraph) Architecture**:
    - Built a bridge between Isaac Sim and ROS2 **without external plugins**.
    - **Motor Control**: Subscribes to `/cmd_vel` → Differential Controller → Articulation Controller.
    - **Camera**: Captures viewport → Publishes `/camera/image/raw`.
    - **LED Control**: Subscribes to ROS service → Changes USD Prim attribute (Color).
3.  **Autonomous Tracking Algorithm**:
    - **Vision Processing**: RGB to HSV conversion → Red color thresholding → Centroid calculation.
    - **Reactive Control**: Implemented a **P-Controller** to adjust angular velocity based on the horizontal error of the detected object center.

## 🛠️ System Architecture

### Action Graph Nodes
The OmniGraph handles the data flow entirely within the simulator:
- **ROS2 Subscribe Twist**: Receives linear/angular velocity commands.
- **Differential Controller**: Computes wheel velocities based on wheel radius and separation.
- **ROS2 Camera Helper**: Publishes rendered frames to ROS2 network.

### Control Logic
- **Input**: Camera Image (`sensor_msgs/Image`)
- **Process**:
  1. Convert image to HSV.
  2. Mask red pixels (`cv2.inRange`).
  3. Calculate centroid $(C_x, C_y)$.
  4. Compute Error $E = (Width / 2) - C_x$.
- **Output**: Twist message (`/cmd_vel`) to steer the robot towards the target.

## 🚀 How to Run

### 1. Prerequisites
- NVIDIA Driver & Isaac Sim installed.
- ROS2 Humble installed on Ubuntu 22.04.

### 2. Build the Package
```bash
# Clone the repository
mkdir -p ~/duckie_ws/src
cd ~/duckie_ws/src
git clone [https://github.com/YOUR_GITHUB_ID/isaac-sim-duckiebot-ros2.git](https://github.com/YOUR_GITHUB_ID/isaac-sim-duckiebot-ros2.git)

# Build
cd ~/duckie_ws
colcon build --symlink-install
source install/setup.bash
````

### 3\. Execution Steps

1.  **Launch Isaac Sim** and open the `supervisor_ath.usd` stage.
2.  **Press Play** in the simulator to start the Action Graph.
3.  **Run the Autonomous Driving Node**:
    ```bash
    ros2 run duckie_bot tracking_node
    ```
4.  **Test LED Control (Optional)**:
    ```bash
    rosservice call /duckie/led_color "data: 'green'"
    ```

## 📊 Results

  - **Simulation**: Successfully implemented the digital twin with realistic physics.
  - **Vision**: Stable detection of the red cube under variable lighting using HSV.
  - **Control**: Smooth tracking behavior utilizing the proportional controller.

-----

*This project was conducted as a final term project for the Robotics and AI course.*

```
```

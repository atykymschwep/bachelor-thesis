# 🎓 Odometry Calibration (Bachelor's Thesis)

![ROS 2](https://img.shields.io/badge/ROS_2-Humble%2FIron-blue)
![C++17](https://img.shields.io/badge/C%2B%2B-17-blue)
![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![Least%20Squares](https://img.shields.io/badge/Method-Least%20Squares-orange)
![Bachelor's%20Thesis](https://img.shields.io/badge/Academic-Bachelor's%20Thesis-success)

A ROS 2 package written in **C++17** for automatic calibration of the kinematic parameters of a differential-drive mobile robot.

Developed as part of a **Bachelor's Thesis**, the project estimates the wheel radius (**R**) and wheelbase (**B**) by comparing wheel encoder odometry with a high-accuracy **Ground Truth** trajectory captured by an **OptiTrack** motion capture system. The calibration is based on the **Least Squares** method and processes recorded ROS 2 bag files (`.mcap`).

## 🚀 Key Features

* 📏 **Automatic Odometry Calibration:** Estimates corrected wheel radius (**R**) and wheelbase (**B**) parameters.
* 📐 **Least Squares Optimization:** Computes calibration coefficients by minimizing the error between encoder-based odometry and Ground Truth.
* 📊 **Ground Truth Comparison:** Uses OptiTrack motion capture data recorded in ROS 2 bag files.
* ✅ **Verification Pipeline:** Recomputes odometry using the calibrated parameters and exports the results for evaluation.
* 📈 **Python Visualization:** Includes a plotting script for comparing the original odometry, calibrated odometry, and Ground Truth trajectories.
* 🧩 **ROS 2 Workflow:** Designed to work directly with recorded ROS 2 `.mcap` bag files.

---

## 📁 Project Structure

```text
odom_calib/
├── src/
│   ├── main.cpp             # Calibration node
│   ├── verification.cpp     # Verification node
│   └── show.py              # Python visualization script
├── include/
│   └── robot_params.hpp     # Robot parameters and constants
├── CMakeLists.txt
└── package.xml
```

---

## 📂 Required ROS 2 Bag Data

The calibration requires a recorded ROS 2 bag (`.mcap`, CDR serialization) containing the following topics:

| Topic                 | Message Type                    | Description                                       |
| --------------------- | ------------------------------- | ------------------------------------------------- |
| `/Kobuki/pose`        | `geometry_msgs/msg/PoseStamped` | Ground Truth trajectory from the OptiTrack system |
| `/kobuki/sensors_raw` | `std_msgs/msg/Int32MultiArray`  | Raw wheel encoder ticks                           |

---

## 🛠 Requirements

* Ubuntu 22.04 or newer
* ROS 2 (Humble or Iron)
* C++17
* Python 3
* ROS 2 bag recorded in `.mcap` format

---

## ⚙️ Installation & Build

Clone the repository into your ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/YOUR_GITHUB_USERNAME/odom_calib.git
```

Replace `YOUR_GITHUB_USERNAME` with your GitHub username.

Build the package:

```bash
cd ~/ros2_ws

colcon build --packages-select odom_calib

source install/setup.bash
```

---

## 🚀 Calibration Workflow

The complete calibration consists of three stages.

### 1. Estimate Calibration Parameters

Run the calibration node and provide the path to the recorded ROS 2 bag:

```bash
ros2 run odom_calib odom_calib /path/to/rosbag2.mcap
```

The node computes:

* linear correction coefficient (`k_s`)
* angular correction coefficient (`k_th`)
* calibrated wheel radius (`R`)
* calibrated wheelbase (`B`)

The computed values are printed in the terminal.

---

### 2. Verify the Calibration

Run the verification node using the calibrated parameters obtained in the previous step:

```bash
ros2 run odom_calib verification /path/to/rosbag2.mcap <new_R> <new_B>
```

The node:

* aligns the initial robot pose,
* recomputes odometry using the calibrated parameters,
* exports the results to:

```text
src/odom_calib/trajectory_data.csv
```

---

### 3. Visualize the Results

Generate trajectory comparison plots:

```bash
python3 src/odom_calib/show.py
```

The visualization compares:

* Ground Truth (OptiTrack)
* Original odometry
* Calibrated odometry


# Center-of-Mass Estimation for xArm7

ROS 2 workspace for estimating the center of mass (CoM) of objects held by an xArm7 manipulator, using joint torque measurements. Built on ROS 2 Humble and PyBullet.

---

## Setup

**Docker (recommended):**
```bash
docker compose build
docker compose up -d
docker exec -it xarm_container bash
```

**Manual:**
```bash
./scripts/setup_xarm_ros2.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Commands

| Command | Description |
|---------|-------------|
| `ros2 run sim_robot_pkg sim_world` | Run CoM estimation in PyBullet simulation |
| `ros2 run sim_robot_pkg sim_world --no-gui` | Headless (no GUI) |
| `ros2 run sim_robot_pkg sim_world --object INT` | Skip object selection (1–10) |
| `ros2 run sim_robot_pkg sim_world --noisy` | Add friction noise |
| `ros2 run sim_robot_pkg sim_world --compare` | Run all three estimators and print table |
| `ros2 run sim_robot_pkg pick_place_demo` | Visual pick-and-place demo (3 phases) |
| `ros2 launch real_robot_pkg real_robot.launch.py robot_ip:=<IP>` | Run on real xArm7 |


---

## Test Objects

| ID | Object | Mass |
|----|--------|------|
| 1 | cube\_1 | 0.5 kg |
| 2 | cube\_2 (CoM offset −17 mm Z) | 0.5 kg |
| 3 | parallelepipede | 1.23 kg |
| 4–5 | cube\_1/2\_heavy | 2.5 kg |
| 6–7 | cube\_1/2\_large | 2.5 kg |
| 8 | parallelepipede\_large | 2.5 kg |
| 9 | bottle\_mybottle | 2.0 kg |
| 10 | bottle\_mybottle\_heavy (CoM −40 mm) | 2.0 kg |


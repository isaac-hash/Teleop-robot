
# Genesys Teleop Example

This project demonstrates how to use **Genesys** to spawn robots into a Gazebo simulation and control them via a custom **teleoperation node**.  
It shows how to work with URDF/SDF models, worlds, and ROS 2 namespaces in a developer-friendly way.

---

## 🛠 Requirements
- [ROS 2 Humble or newer](https://docs.ros.org/en/humble/Installation.html)
- [Gazebo Classic](http://gazebosim.org/) with `gazebo_ros_pkgs`
- Python ≥ 3.8
- Genesys (install from PyPI or local dev build)

```bash
pip install genesys-framework-cli
````

---

## 📂 Project Structure

```
|-- src/
├── sim/
│   ├── worlds/          # Place .world or .sdf world files here
│   └── models/          # Place URDF/SDF robot models here
├──            # Custom teleoperation node
├── README.md
```

---

## ▶️ Running Simulation

1. Build your workspace:

   ```bash
   genesys build
   ```

2. Launch a simulation with your world file:

   ```bash
   genesys sim empty.world
   ```

   * This starts Gazebo and spawns any valid robots found in `sim/models/`.

3. You should see robots automatically assigned namespaces:

   ```
   /robot1/
   /robot2
   ...
   ```

---

## 🎮 Teleoperation

The included `teleop.py` node allows you to control a robot with **WASD** keys:

* **W** → forward
* **S** → backward
* **A** → turn left
* **D** → turn right
* **Space** → stop
* **Q** → quit

### Run teleop for a specific robot

```bash
ros2 run mypkg teleop --ros-args -r __ns:=/robot1
```

* Publishing to `/cmd_vel` will move **all robots** listening to that topic.
* Publishing to `/robot1/cmd_vel` will move **only robot1**.

---

## ⚡ Example Workflow

1. Add your robot SDF/URDF to `sim/models/`.
2. Add a world (or use `empty.world`) in `sim/worlds/`.
3. Start simulation:

   ```bash
   genesys sim empty.world
   ```
4. In another terminal, run teleop:

   ```bash
   genesys run demo
   ```

---

## 📌 Notes

* Genesys handles spawning robots automatically — no need to hand-craft launch files for each world/model combo.
* If you add multiple robots, run multiple teleop nodes with different namespaces.
* Use `rviz2` or `genesys topic echo` to visualize/control topics during simulation.

---

## 📜 License

Apache 2.0 (same as Genesys core).


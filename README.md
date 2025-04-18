# 🤖 Motion Planning with RRT, PRM, and A* Algorithms

This project is a coursework submission for the Robotics Systems module (6CCE3ROB/7CCEMROS) at King's College London. It focuses on implementing motion planning algorithms for a 2D robot navigation task using RRT, PRM, and A* methods.

---

## 📌 Project Objective

The goal is to design a motion planning system for a mobile robot navigating a 2D space from a start position `(0, 0)` to a goal position `(6, 10)`, avoiding a set of defined circular obstacles.

---

## 🧠 Algorithms Implemented

### Task 1: **RRT (Rapidly-Exploring Random Tree)**
- Implements the RRT algorithm to explore the space.
- Outputs include path length, computation time, and visual representation of the search tree.
- 🔗 `RRTPlanner.py`

### Task 2: **PRM (Probabilistic Roadmap)**
- Constructs a roadmap of collision-free configurations.
- Results include the final path, total planning time, and PRM graph visualization.
- 🔗 `PRMPlanner.py`

### Task 3: **A\* Algorithm**
- Uses Euclidean distance as a heuristic.
- Performs grid-based path planning with full path visualization.
- 🔗 `ASTARPlanner.py`

---

## ⚙️ Environment & Dependencies

- Python 3.x
- NumPy
- Matplotlib

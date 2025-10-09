# Pyroki – Syncro 5 Robot Integration

[![Python](https://img.shields.io/badge/python-3.10+-blue)](https://www.python.org/)  

Quickly use **Pyroki** with the **Syncro 5** robot for inverse kinematics, trajectory planning, and collision-aware motion.

---

## ⚡ Quick Start

### 1️⃣ Clone the repository
```
git clone https://github.com/yourusername/pyroki_framework.git  
cd pyroki_framework
```

### 2️⃣ Install dependencies
```
cd pyroki
pip install -e .
```

### 3️⃣ Run Online Planning Example
```
python3 examples/06_online_planning.py
```

This will start Pyroki’s online planning demo for syncro 5.


## 📁 Robot Models

Your **Syncro 5** URDF and meshes are located in the repository:

robot_models/  
├── urdf/  
│   └── syncro_5.urdf  
└── meshes/  
    └── syncro_5/  
        ├── base_link.STL  
        ├── link1.STL  
        ├── link2.STL  
        ├── link3.STL  
        ├── link4.STL  
        ├── link5.STL  
        └── end-effector.STL

> Make sure the `robot_models` folder is in the same directory as your scripts so Pyroki can load the meshes correctly.

---


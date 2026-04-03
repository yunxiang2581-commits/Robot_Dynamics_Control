"""
@Author  : Administrator  # 创建者（自动获取系统用户名）
@Time    : 2026/4/1 21:36  # 创建时间（自动生成）
@File    : init_project.py  # 文件名
@Description :  # 文件用途描述
    此处填写文件的功能说明、实现逻辑等
"""
from pathlib import Path

root = Path(r"D:\桌面\project\Robot Dynamics Control")

dirs = [
    "configs",
    "controllers",
    "dynamics",
    "envs",
    "experiments",
    "logs",
    "notes",
    "plots",
    "tests",
    "utils",
]

files = [
    "README.md",
    "requirements.txt",
    "notes/day1_setup.md",
    "tests/test_mujoco_basic.py",
]

for d in dirs:
    (root / d).mkdir(parents=True, exist_ok=True)

for f in files:
    path = root / f
    path.parent.mkdir(parents=True, exist_ok=True)
    path.touch(exist_ok=True)

print("Project structure created successfully.")
print(root)
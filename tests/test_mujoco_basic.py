"""
@Author  : Administrator  # 创建者（自动获取系统用户名）
@Time    : 2026/4/1 21:07  # 创建时间（自动生成）
@File    : test_mujoco.py.py  # 文件名
@Description :  # 文件用途描述
    此处填写文件的功能说明、实现逻辑等
"""
import mujoco

xml = """
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <body name="box" pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.05 0.05 0.05" mass="1"/>
    </body>
    <geom name="floor" type="plane" size="1 1 0.1"/>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

print("MuJoCo version:", mujoco.__version__)
print("nq =", model.nq)
print("nv =", model.nv)
print("timestep =", model.opt.timestep)

for _ in range(1000):
    mujoco.mj_step(model, data)

print("simulation time =", data.time)
print("final qpos =", data.qpos)
print("MuJoCo step OK")
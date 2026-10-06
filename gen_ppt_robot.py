#!/usr/bin/env python3
"""
Sinh file MuJoCo XML cho robot "single twistable tendon" (PPT: push-pull-twist) nhiều đốt.

Cách dùng:
    python gen_ppt_robot.py --n 2  -o model_n2.xml
    python gen_ppt_robot.py --n 16 -o model_n16.xml
    # dùng mesh STL (đơn vị mm trong file STL):
    python gen_ppt_robot.py --n 2 --mesh notch.stl --mesh-scale 0.001 -o model_mesh.xml

Ý tưởng mô hình (theo bài báo, mô hình PCC):
  - Mỗi đốt (notch) = 1 body, có 2 hinge: bend (trục y) rồi twist (trục z của chính đốt đó).
  - Chuỗi đốt luôn nối dọc theo +z (KHÔNG xoay cả chuỗi bằng euler).
  - Chỉ có 2 actuator (tương ứng 1 sợi tendon: kéo/đẩy + xoắn) -> điều khiển
    tổng góc bend và tổng góc twist, chia đều cho các đốt bằng fixed tendon.
Đơn vị SI (m, kg, rad).
"""
import argparse
import math

p = argparse.ArgumentParser()
p.add_argument("--n", type=int, default=2, help="số đốt (notch)")
p.add_argument("--seg-len", type=float, default=3.1e-3, help="chiều dài 1 đốt [m]")
p.add_argument("--ro", type=float, default=1.75e-3, help="bán kính ngoài [m] (OD 3.5 mm)")
p.add_argument("--ri", type=float, default=0.9e-3, help="bán kính trong [m] (ID 1.8 mm)")
p.add_argument("--E", type=float, default=5e6, help="Young's modulus vật liệu mềm [Pa] (ước lượng)")
p.add_argument("--mass-total", type=float, default=1.05e-4, help="khối lượng cả robot [kg] (0.105 g)")
p.add_argument("--mesh", default=None, help="tên file STL cho 1 đốt (tùy chọn)")
p.add_argument("--mesh-scale", type=float, default=0.001, help="scale mesh (STL mm -> m = 0.001)")
p.add_argument("--mesh-pos", type=float, nargs=3, default=[0, 0, 0], help="dịch mesh trong frame đốt [m]")
p.add_argument("--mesh-euler", type=float, nargs=3, default=[0, 0, 0], help="xoay mesh (rad) để trục dài về +z")
p.add_argument("--meshdir", default="assets/models")
p.add_argument("-o", "--out", default="model.xml")
a = p.parse_args()

N, L, ro, ri, E = a.n, a.seg_len, a.ro, a.ri, a.E
m_seg = a.mass_total / N

# --- độ cứng khớp từ cơ học vật liệu: k = EI / L ; kt = GJ / L, G = E/3, J = 2I
I = math.pi / 4 * (ro**4 - ri**4)
k_bend = E * I / L
k_twist = (E / 3) * (2 * I) / L
# --- armature nhỏ nhưng đủ để ổn định với timestep 0.5 ms
arm = 1e-6
damp_b = 2 * math.sqrt(k_bend * arm)      # ~ tới hạn
damp_t = 2 * math.sqrt(k_twist * arm)
# --- servo của "tendon tổng": kp ~ 5 lần độ cứng 1 khớp
kp_b, kp_t = 5 * k_bend, 5 * k_twist
# --- giới hạn: tổng bend ±180°, tổng twist ±(~255°)
rng_b = math.pi / N
rng_t = 4.5 / N

out = []
w = out.append

w(f'<mujoco model="single_twist_tendon_n{N}">')
w('  <compiler angle="radian" autolimits="true"' + (f' meshdir="{a.meshdir}"' if a.mesh else '') + '/>')
w('  <option gravity="0 0 -9.81" timestep="0.0005" integrator="implicitfast"/>')
w('')
w('  <asset>')
w('    <material name="silicone" rgba="0.2 0.8 0.9 1"/>')
if a.mesh:
    w(f'    <mesh name="notch_mesh" file="{a.mesh}" scale="{a.mesh_scale} {a.mesh_scale} {a.mesh_scale}"/>')
w('  </asset>')
w('')
w('  <default>')
w('    <default class="bend">')
w(f'      <joint type="hinge" axis="0 1 0" stiffness="{k_bend:.4g}" damping="{damp_b:.4g}" armature="{arm:g}" range="{-rng_b:.5f} {rng_b:.5f}"/>')
w('    </default>')
w('    <default class="twist">')
w(f'      <joint type="hinge" axis="0 0 1" stiffness="{k_twist:.4g}" damping="{damp_t:.4g}" armature="{arm:g}" range="{-rng_t:.5f} {rng_t:.5f}"/>')
w('    </default>')
w('  </default>')
w('')
w('  <worldbody>')
w('    <light pos="0 0 0.3" dir="0 0 -1" directional="true"/>')
w('    <geom type="plane" size="0.5 0.5 0.01" rgba="0.9 0.9 0.9 1"/>')
w('    <camera name="side" pos="0.12 -0.12 0.06" xyaxes="0.707 0.707 0 -0.2 0.2 0.96"/>')
w('')
# base: hình trụ cố định, mặt trên ở z = 0.02
w('    <body name="base" pos="0 0 0.01">')
w('      <geom type="cylinder" size="0.006 0.01" rgba="0.3 0.3 0.3 1"/>')

indent = "      "
w(f'{indent}<!-- đốt đầu tiên bắt đầu ngay mặt trên base, trục chuỗi = +z -->')
depth_open = 0
for i in range(1, N + 1):
    ind = indent + "  " * (i - 1)
    z = 0.01 if i == 1 else L          # đốt 1: mặt trên base (base half-height = 0.01)
    w(f'{ind}<body name="notch_{i}" pos="0 0 {z:g}">')
    w(f'{ind}  <joint name="bend_{i}" class="bend"/>')
    w(f'{ind}  <joint name="twist_{i}" class="twist"/>')
    if a.mesh:
        mp = " ".join(f"{v:g}" for v in a.mesh_pos)
        me = " ".join(f"{v:g}" for v in a.mesh_euler)
        w(f'{ind}  <geom type="mesh" mesh="notch_mesh" pos="{mp}" euler="{me}" material="silicone" mass="{m_seg:.4g}" contype="0" conaffinity="0"/>')
    else:
        w(f'{ind}  <geom type="cylinder" fromto="0 0 0 0 0 {L:g}" size="{ro:g}" material="silicone" mass="{m_seg:.4g}" contype="0" conaffinity="0"/>')
    depth_open += 1
# site đầu tip nằm cuối đốt cuối
ind = indent + "  " * N
w(f'{ind}<site name="tip" pos="0 0 {L:g}" size="{ro/2:g}" rgba="1 0 0 1"/>')
for i in range(N, 0, -1):
    ind = indent + "  " * (i - 1)
    w(f'{ind}</body>')
w('    </body>')
w('  </worldbody>')
w('')
w('  <!-- 2 "tendon ảo" = tổng góc bend và tổng góc twist, chia đều cho mọi đốt -->')
w('  <tendon>')
w('    <fixed name="bend_total">')
for i in range(1, N + 1):
    w(f'      <joint joint="bend_{i}" coef="1"/>')
w('    </fixed>')
w('    <fixed name="twist_total">')
for i in range(1, N + 1):
    w(f'      <joint joint="twist_{i}" coef="1"/>')
w('    </fixed>')
w('  </tendon>')
w('')
w('  <actuator>')
w(f'    <position name="pull_push" tendon="bend_total"  kp="{kp_b:.4g}" ctrlrange="{-math.pi:.4f} {math.pi:.4f}"/>')
w(f'    <position name="twist"     tendon="twist_total" kp="{kp_t:.4g}" ctrlrange="-4.5 4.5"/>')
w('  </actuator>')
w('</mujoco>')

with open(a.out, "w") as f:
    f.write("\n".join(out) + "\n")
print(f"Wrote {a.out}: N={N}, k_bend={k_bend:.3g} N·m/rad, k_twist={k_twist:.3g} N·m/rad, m_seg={m_seg:.3g} kg")
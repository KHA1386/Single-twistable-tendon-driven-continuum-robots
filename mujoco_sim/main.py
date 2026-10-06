import mujoco
import mujoco.viewer
import numpy as np
import time

# ==========================================
# 1. CHUYỂN ĐỔI MÔ HÌNH TOÁN TỪ MATLAB
# ==========================================

def load_robot_params():
    """Tương đương loadRobotParams.m nhưng đổi sang m"""
    g = 2.5 * 1e-3
    r_o = (3.5 / 2) * 1e-3
    r_i = (1.8 / 2) * 1e-3
    p = 0.4375 * 1e-3
    h = 1.5 * 1e-3
    d_t = 1.04 * 1e-3

    phi_o = 2 * np.arccos((g - r_o) / r_o)
    phi_i = 2 * np.arccos((g - r_o - p) / r_i)

    A_o = 0.5 * r_o**2 * (phi_o - np.sin(phi_o))
    A_i = 0.5 * r_i**2 * (phi_i - np.sin(phi_i))

    y_o = ((2 * r_o**3) / (3 * A_o)) * (np.sin(0.5 * phi_o))**3
    y_i = ((2 * r_i**3) / (3 * A_i)) * (np.sin(0.5 * phi_i))**3 + p
    y_c = -p * np.pi * r_i**2 / (np.pi * r_o**2 - np.pi * r_i**2)
    y_nbp = (y_o * A_o - y_i * A_i) / (A_o - A_i)

    return h, d_t, y_c, y_nbp

def compute_pcc(delta_l):
    """Tương đương pcc.m - Tính toán độ cong (curvature)"""
    # Chuyển đổi đơn vị sang mét
    delta_l = delta_l * 1e-3
    
    # Material and mechanics
    E_t = 50 * 1e9    # Young's modulus of tendon [Pa]
    E_s = 6 * 1e9     # Young's modulus of continuum robot [Pa]
    k_push = 0.65     # Negative factor
    
    # Geometric params in [m]
    d_t = 1.04 * 1e-3
    r_o = 3.5 / 2 * 1e-3
    r_i = 1.8 / 2 * 1e-3
    p = 0.4375 * 1e-3
    g = 2.5 * 1e-3
    r_t = 0.15 * 1e-3
    A_t = np.pi * r_t**2
    L = 255 * 1e-3
    
    y_c = -p * np.pi * r_i**2 / (np.pi * (r_o**2 - r_i**2))
    
    # Neutral bending plane
    phi_o = 2 * np.arccos((g - r_o) / r_o)
    phi_i = 2 * np.arccos((g - r_o - p) / r_i)
    
    A_o = 0.5 * r_o**2 * (phi_o - np.sin(phi_o))
    A_i = 0.5 * r_i**2 * (phi_i - np.sin(phi_i))
    
    y_o = ((2 * r_o**3) / (3 * A_o)) * (np.sin(0.5 * phi_o))**3
    y_i = ((2 * r_i**3) / (3 * A_i)) * (np.sin(0.5 * phi_i))**3 + p
    y_bar = (y_o * A_o - y_i * A_i) / (A_o - A_i)
    
    # Area MoI
    I_s = (np.pi/4 * r_o**4 + np.pi * r_o**2 * y_c**2) - (np.pi/4 * r_i**4 + np.pi * r_i**2 * (p - y_c)**2)
    I_s = I_s + np.pi * (r_o**2 - r_i**2) * (y_bar - y_c)**2
    
    # Bending moment
    if delta_l >= 0:
        M = (d_t + y_bar) * (E_t * A_t * delta_l) / L
    else:
        M = k_push * (d_t + y_bar) * (E_t * A_t * delta_l) / L
        
    # Curvature
    kappa = M / (E_s * I_s)
    kappa_ = kappa / (1 - kappa * y_bar)
    
    return kappa_

# ==========================================
# 2. KHỞI CHẠY MÔ PHỎNG MUJOCO
# ==========================================
def main():
    # Test thử hàm lấy thông số
    h, d_t, y_c, y_nbp = load_robot_params()
    print(f"Robot Params Loaded -> h: {h}, d_t: {d_t}, y_c: {y_c:.4f}, y_nbp: {y_nbp:.4f}")

    # Nạp mô hình từ file XML
    xml_path = '''D:\LAB\1. mujoco project\single twist tendon in mujoco\assets\models\model.xml'''
    try:
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        
        print("Đã load mô hình MuJoCo thành công. Đang mở Viewer...")
        
        # Mở cửa sổ mô phỏng
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Chạy vòng lặp mô phỏng
            while viewer.is_running():
                step_start = time.time()
                
                # Ở các bước tiếp theo, bạn có thể truyền q (vị trí khớp) 
                # hoặc tín hiệu điều khiển gân (tendon) vào data.ctrl tại đây
                mujoco.mj_step(model, data)
                
                # Cập nhật UI
                viewer.sync()
                
                # Căn chỉnh thời gian thực
                time_until_next_step = model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                    
    except Exception as e:
        print(f"Lỗi khi load MuJoCo: {e}")

if __name__ == "__main__":
    main()
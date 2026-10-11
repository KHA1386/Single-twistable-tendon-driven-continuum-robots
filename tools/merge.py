import numpy as np
import trimesh

# 1. Tải các mesh gốc
notch_base = trimesh.load('assets/models/notch.stl')
distal_base = trimesh.load('assets/models/distal.stl')

# Đổi đơn vị sang mm (vì file gốc là mét)
scale_matrix = trimesh.transformations.scale_matrix(1000.0)
notch_base.apply_transform(scale_matrix)
distal_base.apply_transform(scale_matrix)

# Ma trận biến đổi cục bộ của geom bên trong mỗi đốt theo XML
# (pos="-0.00175 0.00175 -0.047" đổi ra mm là [-1.75, 1.75, -47.0])
local_pos = np.array([-1.75, 1.75, -47.0])
local_rot = trimesh.transformations.quaternion_matrix(
    [0.70710678, 0.70710678, 0, 0]
)
local_rot[:3, 3] = local_pos

meshes_to_combine = []

# Khởi tạo ma trận tích lũy ban đầu từ base (pos="0 0 0.004" -> z = 4.0 mm)
accumulated_matrix = np.eye(4)
accumulated_matrix[2, 3] = 4.0

# 2. Vòng lặp 16 đốt theo đúng cấu trúc phân cấp (parent-child chaining)
for i in range(1, 17):
  segment = notch_base.copy()

  # Mỗi đốt con cách đốt cha một đoạn pos="0 0 0.003" (3.0 mm) vàjoint pos z = 0.00075
  # Phép dịch chuyển tịnh tiến của riêng đốt này so với hệ tọa độ cha:
  step_translation = np.eye(4)
  step_translation[2, 3] = 3.0  # 0.003m = 3.0mm

  # Cập nhật ma trận tích lũy chuỗi phân cấp
  accumulated_matrix = accumulated_matrix.dot(step_translation)

  # Tổng hợp ma trận biến đổi cuối cùng cho đốt i (Tích lũy cha-con * Biến đổi hình học cục bộ)
  total_matrix = accumulated_matrix.dot(local_rot)
  segment.apply_transform(total_matrix)

  meshes_to_combine.append(segment)

# 3. Thêm phần đầu distal_tip tiếp nối sau đốt thứ 16
distal_step = np.eye(4)
distal_step[2, 3] = 3.0  # Khoảng cách body distal_tip so với notch_16
accumulated_matrix = accumulated_matrix.dot(distal_step)

distal_local_pos = np.array([-1.75, 1.75, -48.5])  # -0.0485m = -48.5mm
distal_rot = trimesh.transformations.quaternion_matrix(
    [0.70710678, 0.70710678, 0, 0]
)
distal_rot[:3, 3] = distal_local_pos

distal_total_matrix = accumulated_matrix.dot(distal_rot)
distal_base.apply_transform(distal_total_matrix)
meshes_to_combine.append(distal_base)

# 4. Gộp toàn bộ mesh thành một thể thống nhất
final_body = trimesh.util.concatenate(meshes_to_combine)

# 5. Xuất file hoàn chỉnh
output_filename = 'body_16_segments_merged.stl'
final_body.export(output_filename)
print(
    f'Đã tạo thành công file in 3D: {output_filename} với tổng số {len(meshes_to_combine)} thành phần được xếp nối tiếp!'
)
# Single-twistable-tendon-driven-continuum-robots

Mô phỏng trong **MuJoCo** một robot mềm liên tục (continuum robot) điều khiển bằng **một gân** (tendon) có thể **kéo-đẩy** và **xoắn**.

Robot thật là một thân liền khối. Trong mô phỏng, thân được **rời rạc hóa thành 16 đốt cứng** nối với nhau bằng các khớp đàn hồi (cách làm giống OpenCR-MuJoCo): độ cứng và damping của khớp thay cho tính đàn hồi của vật liệu. Khoảng hở nhìn thấy giữa các đốt chỉ là hệ quả của việc rời rạc hóa, không phải hình dạng thật.

---

## 1. Chạy thử nhanh

Cài môi trường (một lần):

```bash
python -m venv .venv
.venv\Scripts\activate          # Windows
pip install mujoco numpy
```

Sau đó chạy một trong hai lệnh dưới đây ở **thư mục gốc** của dự án.

| Lệnh | Bạn sẽ thấy gì |
|---|---|
| `python -m mujoco.viewer --mjcf=assets/models/model16n.xml` | Cửa sổ MuJoCo hiện robot màu xanh gồm 16 đốt đứng thẳng trên mặt sàn, sợi gân màu đỏ chạy dọc thân. Mở mục **Control** bên phải: kéo thanh `pull_push` để robot cong, kéo `twist` để xoắn. |


---

## 2. Cấu trúc thư mục

```
.
├── assets/
│   ├── mesh/                      mesh 3D (STL)
│   │   ├── notch.stl              [CHÍNH] một đốt (notch) của robot
│   │   ├── distal.stl             [CHÍNH] đốt đầu (distal tip)
│   │   └── merged/                [PHỤ TRỢ] mesh đã gộp, dùng cho SOFA
│   │       ├── body_16_dot.obj
│   │       ├── body_16_dot.stl
│   │       └── body_16_segments_merged.stl
│   └── models/                    mô hình MuJoCo (XML)
│       ├── model16n.xml           [CHÍNH] file mô hình để nạp vào MuJoCo
│       ├── part/                  các file con, được model16n.xml gọi bằng <include>
│       │   ├── assets.xml
│       │   ├── chain.xml
│       │   ├── tendon.xml
│       │   └── actuator.xml
│       └── test/
│           └── model2n.xml        [PHỤ TRỢ] mô hình thử 2 đốt
├── mujoco_sim/                    [CHƯA MÔ TẢ] code mô phỏng (xem mục 5)
├── tools/
│   └── merge.py                   [PHỤ TRỢ] gộp mesh các đốt
└── README.md
```

Chú thích: **[CHÍNH]** là file bạn cần đọc hoặc chạy. **[PHỤ TRỢ]** là file hỗ trợ, bạn không cần mở khi chỉ muốn chạy mô phỏng.

---

## 3. Các file chính

### `assets/models/model16n.xml`: file mô hình
Đây là file duy nhất bạn nạp vào MuJoCo. Nó chỉ chứa sàn, đèn và các dòng `<include>` gọi bốn file trong `part/`. Mỗi file con đảm nhận một việc:

| File | Nhiệm vụ |
|---|---|
| `part/assets.xml` | Khai báo mesh, vật liệu, các tùy chọn mô phỏng (bước thời gian, trọng lực) và các lớp mặc định: độ cứng, damping, giới hạn góc của khớp. |
| `part/chain.xml` | Chuỗi thân robot: đế → 16 notch → đốt distal. Mỗi notch có 3 khớp đàn hồi (xem mục 4) và một `site` đặt tại lỗ luồn gân. |
| `part/tendon.xml` | Khai báo gân: `tendon_pull` là sợi gân thật đi qua lỗ của từng notch; `bend_total` và `twist_total` là tổng góc uốn và xoắn. |
| `part/actuator.xml` | Hai bộ điều khiển: `pull_push` (kéo-đẩy gân) và `twist` (xoắn). |

---

## 4. Mô hình hoạt động thế nào

- **16 notch**, mỗi notch cao 3 mm, đường kính 3.5 mm. Các đốt không va chạm với nhau (`contype=0`, `conaffinity=0`); khi cần va chạm với vật bên ngoài thì thêm geom riêng cho việc đó.
- **3 khớp đàn hồi tại mỗi notch**, cùng đặt ở giữa phần flexure:
  - `bend_i`: uốn quanh trục y (mặt phẳng của gân kéo),
  - `bend2_i`: uốn quanh trục x (hướng vuông góc),
  - `twist_i`: xoắn quanh trục dọc z.
- **Gân thật**: `tendon_pull` đi qua các `site` đặt ở lỗ luồn gân, cách trục 1.04 mm về phía −x. Lực kéo gân tạo moment uốn robot.
- **Điều khiển**:
  - `pull_push`: `ctrl` là độ **dài thêm của gân (đơn vị mét)** so với lúc robot thẳng. Giá trị âm là kéo, dương là đẩy. Khoảng cho phép −0.0035 đến 0.0035.
  - `twist`: `ctrl` là góc xoắn tổng (rad), khoảng −4.5 đến 4.5.

---

## 5. File phụ trợ

| File | Tác dụng |
|---|---|
| `tools/merge.py` | Gộp mesh `notch.stl` và `distal.stl` thành một mesh liền. Kết quả nằm trong `assets/mesh/merged/`. Dùng cho bước chuyển sang SOFA, không cần cho MuJoCo. |
| `assets/mesh/merged/*` | Mesh đã gộp (một thân liền thay vì 16 đốt rời), do `merge.py` tạo ra. |
| `assets/models/test/model2n.xml` | Mô hình thử với 2 đốt, dùng để kiểm tra nhanh cú pháp hoặc ý tưởng mới. |
| `mujoco_sim/` | <!-- TODO: ghi mục đích thư mục này và file nào trong đó chạy được --> |

---

## 6. Lưu ý khi dùng trên máy khác

- Đường dẫn tới `notch.stl` và `distal.stl` trong `assets/models/part/assets.xml` đang là **đường dẫn tuyệt đối trên máy tác giả**. Khi chép dự án sang máy khác, bạn sửa hai dòng `<mesh ... file="...">` cho đúng đường dẫn thật tới thư mục `assets/mesh/`.
- Các file trong `part/` được `model16n.xml` nạp bằng `<include file="part/...">`. Nếu bạn đổi tên hoặc di chuyển chúng, nhớ sửa lại các dòng include đó.
- `MUJOCO_LOG.TXT` là file nhật ký MuJoCo tự sinh, không cần quan tâm.

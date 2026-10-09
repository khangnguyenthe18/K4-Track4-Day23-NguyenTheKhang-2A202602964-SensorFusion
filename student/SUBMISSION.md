# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Nguyễn Thế Khang
- MSSV: 2A202602964
- Email: thekhang2004@gmail.com
- Link repo (fork): https://github.com/khangnguyenthe18/K4-Track4-Day23-NguyenTheKhang-2A202602964-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): c956fcc35051f123955dc2d9d40ee2cd47c914d5

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: compare, [0, 198] (199 frames), training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord, seed 0
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: precision = 0.9700934579439252 (97.01%), recall = 0.7004048582995951 (70.04%), tp = 519, fp = 16, fn = 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse = 0.15027808396396983 m, matches = 502, sum_sq_err = 11.336918264980747 m², ghost_track_frames = 0, missed_gt_frames = 239, mean_confirmed_tracks = 2.522613065326633
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse = 0.13584777701442574 m, matches = 502, sum_sq_err = 9.26421849692009 m², ghost_track_frames = 0, missed_gt_frames = 239, mean_confirmed_tracks = 2.522613065326633
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss:
  1. Số cặp ghép (matches = 502), số ghost tracks (ghost_track_frames = 0) và số miss GT (missed_gt_frames = 239) là hoàn toàn giống nhau giữa hai mode LiDAR và Fused. Điều này thể hiện đúng tính chất của kiến trúc track-then-fuse: Camera chỉ cập nhật trạng thái EKF (refine state) mà không can thiệp vào vòng đời tạo hay xoá track, đạt precision_track = 502/(502+0) = 1.0 (vượt xa ngưỡng 0.75) và coverage = 502/519 = 0.967 (vượt xa ngưỡng 0.70).
  2. Về sai số vị trí (RMSE): Chế độ Fused đạt RMSE = 0.1358 m, giảm 9.6% so với LiDAR-only (0.1503 m) với tổng bình phương sai số sum_sq_err giảm từ 11.3369 m² xuống 9.2642 m². Sự cải thiện này đến từ việc bổ sung các phép đo góc phương vị chính xác cao từ camera FRONT khi xe nằm trong tầm nhìn, giúp làm mịn tọa độ ngang và vận tốc của xe.
  3. Tính nhất quán của fusion: rmse_fused - rmse_lidar = -0.01443 m <= 0.05 m, thỏa mãn xuất sắc tiêu chuẩn chất lượng và độ an toàn của hệ thống fusion.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`, `H`)?
   - Vector đo `z`: LiDAR đo trực tiếp tọa độ 3D vật lý $[x, y, z]^T$ trong không gian (mét, kích thước 3x1). Camera đo tọa độ pixel 2D $[u, v]^T$ trên mặt phẳng cảm biến ảnh (pixel, kích thước 2x1).
   - Ma trận hiệp phương sai nhiễu `R`: LiDAR có R kích thước 3x3 theo mét vuông ($\text{diag}(0.1^2, 0.1^2, 0.1^2) = \text{diag}(0.01, 0.01, 0.01)\text{ m}^2$). Camera có R kích thước 2x2 theo pixel bình phương ($\text{diag}(5.0^2, 5.0^2) = \text{diag}(25, 25)\text{ px}^2$).
   - Ma trận Jacobian quan sát `H`: Mô hình đo LiDAR là hàm tuyến tính $h(x) = R_{v2s} p + t_{v2s}$, Jacobian H kích thước 3x6 là ma trận hằng số. Ngược lại, camera tuân theo mô hình chiếu xuyên tâm pinhole phi tuyến: $u = c_i - f_i \frac{y_s}{x_s}, v = c_j - f_j \frac{z_s}{x_s}$ (với $p_s = R_{v2s} p + t_{v2s}$), do đó Jacobian H kích thước 2x6 phụ thuộc phi tuyến vào trạng thái ước lượng x tại mỗi thời điểm và tính qua quy tắc chuỗi: $H = J_{proj}(p_s) \cdot R_{v2s} \cdot [I_{3 \times 3} \mid 0_{3 \times 3}]$.

2. Vì sao cần gating Mahalanobis trước khi gán?
   - Khoảng cách Mahalanobis $d^2 = \gamma^T S^{-1} \gamma$ là khoảng cách thống kê chuẩn hóa theo ma trận hiệp phương sai đổi mới $S = H P H^T + R$. Khác với khoảng cách Euclidean thuần túy, Mahalanobis xem xét cả độ bất định phân bố trạng thái của track ($P$) lẫn độ nhiễu cảm biến ($R$).
   - Cổng kiểm định $\chi^2$ (chi-square gate) cho phép lọc bỏ ngay các phép đo ngoại lai (clutter/false alarms) hoặc đo lường thuộc về các phương tiện khác với xác suất sai số vượt mức tin cậy $p=0.995$ (ngưỡng $\approx 12.84$ cho LiDAR bậc 3 và $\approx 10.60$ cho camera bậc 2).
   - Việc gating trước khi thực hiện thuật toán gán Greedy ngăn chặn triệt để hiện tượng gán nhầm đo lường (misassociation), từ đó triệt tiêu nguy cơ trôi track (track drift) và nhảy danh tính (ID switch).

3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.
   - Pipeline thuộc kiến trúc **track-then-fuse** (cụ thể là centralized track-level fusion với các bước cập nhật EKF nối tiếp theo từng modality).
   - Hệ thống duy trì duy nhất một danh sách track toàn cục: mỗi frame chỉ thực hiện `predict` một lần duy nhất, sau đó tiến hành gán và cập nhật EKF cho LiDAR (AssocL), quản lý vòng đời (score/init/delete), rồi tiếp tục gán và cập nhật EKF cho camera (AssocC) trên chính các track đó để tinh chỉnh trạng thái.
   - Minh chứng từ log `fusion-run-lab`: Cả hai file `grade_run_lidar.log` và `grade_run_fused.log` có số lượng confirmed tracks và số lượng matches trên từng frame giống hệt nhau (tổng 502 matches, trung bình 2.52 confirmed tracks/frame). Camera hoàn toàn không sinh thêm track mới cũng như không xóa bất kỳ track nào, chỉ điều chỉnh lại `sum_sq_err` (từ 11.3369 m² xuống 9.2642 m²).

4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?
   - Khi extrinsic của camera bị lệch (ví dụ lệch góc xoay yaw/pitch/roll hoặc độ lệch tịnh tiến), phép chiếu pinhole $h(x)$ sẽ dự đoán vị trí pixel bị lệch có hệ thống (systematic non-zero mean offset).
   - Triệu chứng cụ thể:
     a) Vector sai số đổi mới $\gamma = z - h(x)$ bị lệch kỳ vọng khác 0 (biased innovation), không còn tuân theo phân phối chuẩn Gaussian trắng $\mathcal{N}(0, S)$.
     b) Khoảng cách Mahalanobis $d^2 = \gamma^T S^{-1} \gamma$ tăng vọt. Nếu độ lệch nhỏ (dưới ngưỡng $\chi^2$), EKF vẫn nhận đo nhưng cập nhật sai lệch khiến track bị giật về hướng lệch, làm RMSE tăng cao hơn LiDAR-only (vi phạm tính nhất quán của fusion).
     c) Nếu độ lệch lớn vượt ngưỡng $\chi^2$ ($d^2 > 10.60$), toàn bộ phép đo camera sẽ bị cổng gating loại bỏ, camera rơi vào trạng thái mất liên kết hoàn toàn (100% rejection).

5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng?
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.
   - Cần sensor tường minh: Ở những frame không có measurement nào (`meas_list` rỗng), `associate_and_update` vẫn phải gọi `manager.manage_tracks(unassigned_tracks, unassigned_meas, sensor)`. Nhờ tham số `sensor` tường minh, `TrackManager` phân biệt được modality của lượt quét: Nếu là `lidar`, nó sẽ duyệt các track trong FOV và phạt miss (`score -= 1/window`), đồng thời xét xoá track. Nếu là lượt `camera`, `TrackManager` sẽ bỏ qua ngay lập tức (`if sensor.name != "lidar": return`), tránh phạt nhầm điểm của track.
   - Lý do LiDAR quyết định lifecycle: LiDAR cung cấp vị trí 3D đầy đủ $(x, y, z)$ và khoảng cách độ sâu tuyệt đối, hoạt động ổn định bất kể ngày đêm hay đổ bóng, nên là nguồn tin cậy duy nhất để xác định sự tồn tại của vật thể. Trong khi đó, camera trong lab chỉ đo góc 2D trên ảnh FRONT, thiếu thông tin độ sâu trực tiếp và dễ bị che khuất; nếu cho phép camera can thiệp vào lifecycle, khi xe đi ra khỏi góc nhìn camera, track sẽ bị phạt xóa nhầm hoặc tạo ra các track ảo (ghosts) không thể định vị trong không gian 3D.

6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.
   - Xác nhận track (Confirmation): Track mới được khởi tạo ở trạng thái `"initialized"` với `score = 1/window = 1/6 ≈ 0.167`. Mỗi lần nhận được LiDAR hit, điểm số tăng `+1/window` (tối đa 1.0). Khi `score > confirmed_threshold = 0.8`, track chính thức chuyển thành `"confirmed"` (yêu cầu tối thiểu 5 lần hit liên tiếp).
   - Giữ trạng thái sau miss (Hysteresis): Khi một confirmed track bị miss trong FOV LiDAR, điểm số bị giảm `score -= 1/window`, nhưng trạng thái `"confirmed"` vẫn được giữ nguyên vẹn. Cơ chế trễ này giúp track sống sót qua các frame bị che khuất thoáng qua mà không bị rớt cấp về tentative.
   - Xóa track (Deletion): Track bị xóa ngay lập tức nếu thỏa mãn bất kỳ điều kiện nào sau đây (quan hệ logic OR):
     1. Phương sai tọa độ ngang $P[0, 0] > \text{max\_P}$ hoặc $P[1, 1] > \text{max\_P}$ (với $\text{max\_P} = 3.0^2 = 9.0\text{ m}^2$).
     2. Track đã `"confirmed"` nhưng bị miss liên tục khiến `score < delete_threshold = 0.6`.
     3. Track chưa `"confirmed"` (`"initialized"` hoặc `"tentative"`) bị miss khiến `score <= 0.0`.

## Bonus (không bắt buộc)

- Phân tích độ nhạy Calibration Camera (Mục 2 RUBRIC.md, +4 điểm):
  - File bằng chứng: `student/bonus/calibration_drift_analysis.png`
  - Đánh giá trên 3 mức lệch góc xoay Yaw của camera:
    | Mức lệch Yaw | Độ dịch pixel $\Delta u$ (tại 20m) | Khoảng cách Mahalanobis $d^2$ | Tác động Gating $\chi^2$ (Ngưỡng 10.60) | Ảnh hưởng RMSE tracking |
    |---|---|---|---|---|
    | Yaw $+0.5^\circ$ | $\approx 7.0\text{ px}$ | $1.63$ | Chấp nhận (Pass Gate) | RMSE tăng nhẹ ($0.136 \to 0.141\text{ m}$) do residual lệch |
    | Yaw $+1.0^\circ$ | $\approx 14.0\text{ px}$ | $6.53$ | Chấp nhận sát ngưỡng | RMSE tăng đáng kể ($0.158\text{ m}$), xấu hơn LiDAR-only |
    | Yaw $+2.0^\circ$ | $\approx 28.0\text{ px}$ | $26.10$ | Bị từ chối hoàn toàn (Reject) | Camera bị cô lập, hệ thống tự fallback về LiDAR-only |
  - Kết luận: Cổng Mahalanobis $\chi^2$ hoạt động như một cơ chế an toàn fail-safe tự nhiên, tự động cô lập camera khi góc lệch vượt quá $1.5^\circ \to 2.0^\circ$.

## Khai báo sử dụng AI (bắt buộc)

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Antigravity AI (Gemini 3.8 Flash)
- Dùng cho phần nào (hàm, câu hỏi, debug): Hỗ trợ phân tích công thức ma trận EKF trong `kalman.py`, tính toán ma trận Jacobian chuỗi pinhole trong `camera_fusion.py`, kiểm chứng logic gating chi-square và gán Greedy trong `association.py`, và hỗ trợ soạn thảo phân tích toán học các câu hỏi giải thích.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): Chạy toàn bộ bộ test `pytest student/tests` với 128/128 tests pass (100%), kiểm tra tính hội tụ và đơn điệu của các hàm, đối chiếu các bất biến `matches + ghosts == confirmed` và `matches + misses == valid_gt` trên log `fusion-run-lab`, và kiểm tra script `tools/check_submission.py` đạt kết quả "SẴN SÀNG NỘP".

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [x] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))

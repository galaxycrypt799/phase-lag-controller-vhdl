# Testbench

Mỗi file testbench định nghĩa một entity riêng và có thể mô phỏng độc lập:

- `phase_lag_tb`: kịch bản cơ bản với hệ số mặc định.
- `phase_lag_tb_case1`, `phase_lag_tb_case2`, `phase_lag_tb_case3`: các cấu
  hình hệ số controller thay thế.
- `multi_case_tb`: so sánh bốn cấu hình chạy song song.
- `controller_debug_tb`: kiểm tra controller co-simulation và tín hiệu monitor.

Phân tích (analyze) file testbench sau các file `src/`, rồi elaborate và chạy
entity cùng tên với testbench. Các stimulus mô phỏng không thay thế việc kiểm
tra log và dạng sóng của simulator.

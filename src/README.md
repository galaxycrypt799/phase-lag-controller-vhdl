# Mã thiết kế

Các module ở đây tạo thành hệ thống Phase-Lag:

1. `phase_lag_pkg.vhd` — kiểu dữ liệu, hệ số và khai báo dùng chung.
2. `microdata.vhd` và `microctrl.vhd` — datapath và control unit của CPU.
3. `micro.vhd` — ghép hai khối CPU.
4. `plant.vhd` — mô hình plant rời rạc.
5. `phase_lag_top.vhd` — top-level vòng kín; `phase_lag_controller_cosim.vhd`
   là top-level controller-only cho HDL co-simulation.

Biên dịch package trước, sau đó đến datapath/control, CPU/plant, top-level và
cuối cùng là testbench cần chạy. Xem hướng dẫn GHDL và thông tin cổng trong
README ở thư mục gốc.

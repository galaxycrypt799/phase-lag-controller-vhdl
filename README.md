# Bộ điều khiển Phase-Lag trên FPGA

Dự án mô phỏng một hệ thống điều khiển vòng kín bằng VHDL. Bộ điều khiển
Phase-Lag được xây dựng như một CPU nhỏ có vi chương trình; đầu ra của bộ điều
khiển được đưa vào mô hình plant bậc hai. Dữ liệu dùng số có dấu 24-bit, định
dạng fixed-point Q14.

> **Trạng thái:** mã nguồn và testbench được sắp xếp trong repository. Các test
>bench là những kịch bản mô phỏng có sẵn; cần chạy bằng simulator để xác nhận
>kết quả trên máy của bạn.

## Mục lục

- [Mục tiêu và luồng hoạt động](#mục-tiêu-và-luồng-hoạt-động)
- [Cấu trúc thư mục](#cấu-trúc-thư-mục)
- [Các khối thiết kế](#các-khối-thiết-kế)
- [Biểu diễn số Q14](#biểu-diễn-số-q14)
- [Giao tiếp khối top](#giao-tiếp-khối-top)
- [Testbench](#testbench)
- [Biên dịch và mô phỏng](#biên-dịch-và-mô-phỏng)
- [Mở báo cáo](#mở-báo-cáo)
- [Đưa dự án lên GitHub](#đưa-dự-án-lên-github)

## Mục tiêu và luồng hoạt động

Hệ thống nhận giá trị tham chiếu `ref_in`, so sánh với đầu ra plant `y_out`, rồi
tính sai số:

```text
e(k) = ref_in - y(k)
```

Bộ điều khiển dùng sai số hiện tại, sai số chu kỳ trước và đầu ra điều khiển
chu kỳ trước để tính lệnh mới:

```text
u(k) = (B1*u(k-1) + A0*e(k) + A1*e(k-1)) >> 14
```

Ở cấu hình mặc định, các hệ số Q14 là `B1 = 16382`, `A0 = 16465` và
`A1 = -16301`. Mỗi lần cập nhật, khối điều khiển thực hiện tính toán, cập nhật
`u(k)`, sau đó khối plant tính `y(k)`. Giá trị `y(k)` được dùng để tính sai số
ở lần lấy mẫu kế tiếp.

Luồng khối:

```text
ref_in ──> tính sai số ──> micro (microctrl + microdata) ──> u(k)
   ^                                                        │
   └──────────────────── plant <───────────────────────────┘
```

## Cấu trúc thư mục

```text
.
|-- src/                         # Mã VHDL của hệ thống
|   |-- phase_lag_pkg.vhd        # Hằng số, kiểu và khai báo component dùng chung
|   |-- microctrl.vhd            # Bộ điều khiển vi chương trình
|   |-- microdata.vhd            # Datapath, ALU và register bank
|   |-- micro.vhd                # Ghép microctrl với microdata thành CPU
|   |-- plant.vhd                # Mô hình plant bậc hai rời rạc
|   |-- phase_lag_top.vhd        # Top-level hệ vòng kín
|   `-- phase_lag_controller_cosim.vhd
|                                # Top-level controller-only cho co-simulation
|-- testbench/                   # Các testbench mô phỏng
|   |-- phase_lag_tb.vhd
|   |-- phase_lag_tb_case1.vhd
|   |-- phase_lag_tb_case2.vhd
|   |-- phase_lag_tb_case3.vhd
|   |-- multi_case_tb.vhd
|   `-- controller_debug_tb.vhd
|-- vendor/                      # Mã thư viện LPM của Altera/Quartus
|   |-- lpm_component_package.vhd
|   `-- lpm_synthesizable_models.vhd
|-- docs/
|   `-- FPGA_VHDL_report.pdf     # Báo cáo đồ án gốc
|-- .gitignore
`-- README.md
```

## Các khối thiết kế

| File | Vai trò |
|---|---|
| `src/phase_lag_pkg.vhd` | Định nghĩa độ rộng word/accumulator, hệ số Q14, mã thao tác ALU, các component và hàm dịch Q14. |
| `src/microctrl.vhd` | Control unit tuần tự hóa các vi lệnh trong control store. Chọn thanh ghi nguồn/đích, phép toán và điều kiện nhánh. |
| `src/microdata.vhd` | Datapath gồm register bank 8 thanh ghi, các latch đầu vào/hệ số, ALU cộng/trừ/nhân/tăng, cờ zero/negative/carry và đầu ra điều khiển. |
| `src/micro.vhd` | Nối `microctrl` với `microdata`; đây là CPU của bộ điều khiển. |
| `src/plant.vhd` | Mô hình plant bậc hai; lưu các mẫu đầu vào/đầu ra trước để tính mẫu kế tiếp. |
| `src/phase_lag_top.vhd` | Điều phối vòng kín: tạo nhịp lấy mẫu, khởi chạy CPU, cập nhật plant và đưa trạng thái ra ngoài. |
| `src/phase_lag_controller_cosim.vhd` | Chỉ chứa controller, tính `ref_in - feed` và đồng bộ tín hiệu khởi chạy từ `clk_op`; dùng cho HDL co-simulation. |
| `vendor/lpm_component_package.vhd` và `vendor/lpm_synthesizable_models.vhd` | Các khai báo/mô hình LPM cũ đi kèm dự án. Mã thiết kế hiện tại không instantiate LPM; chỉ biên dịch khi flow FPGA của bạn cần chúng. |

### Vi chương trình và datapath

Control unit có control store gồm 17 địa chỉ. Vi lệnh thao tác điều khiển
datapath; vi lệnh nhánh kiểm tra `start` hoặc cờ ALU rồi chọn địa chỉ tiếp theo.
Datapath có tám thanh ghi `R0`–`R7`; theo thiết kế, `R0` dùng làm zero,
`R1` giữ sai số hiện tại, `R6` giữ sai số trước và `R7` giữ đầu ra điều khiển
trước. Các thanh ghi còn lại phục vụ kết quả trung gian.

### Mô hình plant

Plant liên tục trong báo cáo là:

```text
G(s) = 100 / (s^2 + 10s + 100)
```

Sau rời rạc hóa Tustin với `Ts = 0.01 s`, mã dùng phương trình sai phân:

```text
y(k) = (31056*y(k-1) - 14827*y(k-2)
        + 39*u(k) + 78*u(k-1) + 39*u(k-2)) >> 14
```

## Biểu diễn số Q14

- Một mẫu được lưu bằng `SIGNED(23 DOWNTO 0)`.
- 14 bit dành cho phần lẻ; hệ số quy đổi là `2^14 = 16384`.
- Đổi số thực sang Q14: nhân với `16384` rồi lượng tử hóa về số nguyên.
- Đổi Q14 sang số thực: chia số nguyên có dấu cho `16384`.
- Ví dụ: `1.0` được lưu là `16384`; `0.5` là `8192`; `-1.0` là `-16384`.
- Tích hai số Q14 có 28 bit phần lẻ. Mã dịch phải 14 bit để đưa kết quả về
  Q14; kết quả bị giới hạn nếu vượt miền biểu diễn 24-bit.

Các hệ số controller mặc định trong package:

| Hệ số | Giá trị Q14 | Xấp xỉ số thực |
|---|---:|---:|
| `B1` | `16382` | `0.999878` |
| `A0` | `16465` | `1.004944` |
| `A1` | `-16301` | `-0.995422` |

## Giao tiếp khối top

Entity chính là `phase_lag_top` trong `src/phase_lag_top.vhd`.

| Cổng | Hướng | Ý nghĩa |
|---|---|---|
| `clk` | vào | Clock hệ thống. |
| `rst` | vào | Reset đồng bộ mức cao; xóa trạng thái vòng lặp và trạng thái plant. |
| `ref_in` | vào | Giá trị đặt, signed Q14 24-bit. |
| `start` | vào | Cho phép bắt đầu một lần tính toán tại nhịp lấy mẫu phù hợp. |
| `y_out` | ra | Đầu ra plant, signed Q14 24-bit. |
| `u_out` | ra | Lệnh điều khiển hiện tại, signed Q14 24-bit. |
| `err_now` | ra | Sai số hiện tại `ref_in - y_out`. |
| `err_old` | ra | Sai số của lần cập nhật trước. |
| `u_old` | ra | Lệnh điều khiển của lần cập nhật trước. |
| `ctrl_done` | ra | Trạng thái hoàn tất của controller. |
| `plant_done` | ra | Xung báo plant đã cập nhật một mẫu. |
| `sample_tick` | ra | Xung báo đến nhịp lấy mẫu. |

Các generic `G_COEFF_A0`, `G_COEFF_A1`, `G_COEFF_B1` cho phép thay hệ số
controller khi mô phỏng. `CLK_FREQ_HZ` và `SAMPLE_PERIOD_S` đặt tần số clock
và chu kỳ lấy mẫu. Mặc định `SIM_SHORT_DIV = TRUE` dùng bộ chia ngắn để giảm
thời gian mô phỏng; đặt `FALSE` để tính số tick theo clock và chu kỳ lấy mẫu
cấu hình.

## Testbench

Mỗi testbench là một entity độc lập; chọn entity tương ứng khi chạy simulator.

| Entity / file | Nội dung |
|---|---|
| `phase_lag_tb` (`phase_lag_tb.vhd`) | Kiểm tra cơ bản reset, controller, đáp ứng plant và vòng kín với hệ số mặc định. |
| `phase_lag_tb_case1` | So sánh cấu hình controller yếu hơn: `A0=8233`, `A1=-8151`, `B1=16382`. |
| `phase_lag_tb_case2` | So sánh cấu hình controller mạnh hơn: `A0=24698`, `A1=-24452`, `B1=16382`. |
| `phase_lag_tb_case3` | Kiểm tra cấu hình yếu với `B1=16383`. |
| `multi_case_tb` | Chạy song song bốn bộ hệ số để so sánh đáp ứng. |
| `controller_debug_tb` | Kiểm tra riêng top co-simulation, sai số và các tín hiệu monitor. |

Các file testbench chứa stimulus và phép quan sát mô phỏng; không nên hiểu việc
có testbench là bằng chứng test đã chạy thành công. Hãy kiểm tra log và dạng sóng
trong simulator sau khi chạy.

## Biên dịch và mô phỏng

Mã dùng các package Synopsys `IEEE.STD_LOGIC_ARITH` và `IEEE.STD_LOGIC_SIGNED`.
Hãy dùng simulator hỗ trợ các package này (ví dụ ModelSim/Questa, flow FPGA
tương thích, hoặc GHDL được bật Synopsys packages).

Ví dụ với GHDL trên PowerShell, chạy từ thư mục gốc repository:

```powershell
New-Item -ItemType Directory -Force build

ghdl -a --std=93 -fsynopsys --workdir=build src/phase_lag_pkg.vhd
ghdl -a --std=93 -fsynopsys --workdir=build src/microdata.vhd src/microctrl.vhd
ghdl -a --std=93 -fsynopsys --workdir=build src/micro.vhd src/plant.vhd
ghdl -a --std=93 -fsynopsys --workdir=build src/phase_lag_top.vhd src/phase_lag_controller_cosim.vhd
ghdl -a --std=93 -fsynopsys --workdir=build testbench/phase_lag_tb.vhd

ghdl -e --std=93 -fsynopsys --workdir=build phase_lag_tb
ghdl -r --std=93 -fsynopsys --workdir=build phase_lag_tb --stop-time=1ms
```

Để chạy testbench khác, thay `phase_lag_tb` bằng entity tương ứng trong bảng
trên và phân tích file `.vhd` của testbench đó trước lệnh elaborate. Nếu simulator
của bạn không nhận `-fsynopsys` hoặc không có hai package Synopsys, hãy cấu hình
library tương thích theo tài liệu của simulator.

## Mở báo cáo

Báo cáo đồ án tiếng Việt gốc nằm tại
[docs/FPGA_VHDL_report.pdf](docs/FPGA_VHDL_report.pdf). Báo cáo mô tả mục tiêu,
kiến trúc CPU, bộ điều khiển, vi chương trình và các kết quả mô phỏng được trình
bày trong tài liệu.

## Đưa dự án lên GitHub

Repository đích dự kiến:
[github.com/galaxycrypt799/phase-lag-controller-vhdl](https://github.com/galaxycrypt799/phase-lag-controller-vhdl)

Repository public này cần được tạo trên GitHub trước khi push lần đầu. Sau khi
cài Git, mở PowerShell tại thư mục dự án và chạy:

```powershell
git init
git add README.md .gitignore src testbench vendor docs
git commit -m "Add phase-lag FPGA controller project"
git branch -M main
git remote add origin https://github.com/galaxycrypt799/phase-lag-controller-vhdl.git
git push -u origin main
```

Nếu remote `origin` đã tồn tại, kiểm tra bằng `git remote -v` thay vì thêm lại.
GitHub sẽ yêu cầu bạn xác thực qua trình duyệt hoặc credential manager; không
đưa access token hay mật khẩu vào README hoặc commit.

## Ghi chú về thư viện vendor

Hai file trong `vendor/` là mã nguồn LPM của Altera, có thông báo bản quyền và
điều khoản đi kèm ở đầu file. Chúng được giữ riêng để phân biệt với mã dự án;
hãy giữ nguyên các thông báo đó nếu sao chép hoặc phân phối các file vendor.

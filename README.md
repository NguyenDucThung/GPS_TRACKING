#ESP32 Smart Motorbike Security & Tracking System
Hệ thống chống trộm, định vị và quản lý xe máy thông minh hiệu năng cao được phát triển trên vi điều khiển ESP32-C3 
sử dụng framework ESP-IDF và hệ điều hành thời gian thực FreeRTOS. Dự án tích hợp các công nghệ định vị 4G/GPS, xác 
thực BLE và cảm biến gia tốc để tối ưu hóa khả năng bảo mật toàn diện cho xe máy đồng thời đạt mức tiêu 
thụ năng lượng siêu thấp.
-------------------------------------------------------------------------------------------------------------------------
#Các Tính Năng Cốt Lõi
Chống Trộm Tự Động (Anti-Theft System): Tự động phát hiện hành vi dựng thẳng xe hoặc dắt xe trái phép thông qua cảm biến MPU6050. Khi phát hiện đột nhập, hệ thống sẽ ngắt Rơ-le hệ thống điện của xe, hú còi cảnh báo và điều khiển module SIM gọi điện cảnh báo trực tiếp tới chủ xe.

Xác Thực Không Chạm Qua BLE (Seamless BLE Authentication): Khi xe được dựng thẳng, hệ thống mở một cửa sổ thời gian 15 giây để quét tín hiệu ứng dụng Flutter trên điện thoại chủ xe qua Bluetooth Low Energy (NimBLE). Nếu xác thực thành công, hệ thống tự động nhả Rơ-le cho phép đề nổ mà không cần thao tác thủ công.

Tìm Xe Từ Xa Qua Cuộc Gọi (Remote Vehicle Finder): Hỗ trợ tìm xe nhanh trong các hầm gửi xe lớn bằng cách gọi điện (nháy máy) vào số SIM trên xe. Cú sụt áp chớp nhoáng trên chân RX UART do chuỗi lệnh RING đổ về sẽ kích hoạt ngắt phần cứng gọi chip tỉnh giấc, hú còi định vị và lập tức đẩy tọa độ GPS lên Firebase trước khi quay lại giấc ngủ sâu.

Năng Lượng Siêu Tiết Kiệm (Ultra-Low Power Standby): Sử dụng chế độ ngủ sâu esp_light_sleep_start() kết hợp cấu hình nguồn ngắt vật lý (GPIO Wakeup) cùng tính năng ngủ giữ mạng (Network Sleep Mode - AT+CSCLK=1) của module SIM. Tổng dòng chờ của toàn mạch chỉ ~2.5mA, đảm bảo an toàn tuyệt đối cho bình ắc quy xe máy trong hơn 30 ngày đỗ liên tục.
-------------------------------------------------------------------------------------------------------------------------
# Kiến Trúc Đa Luồng FreeRTOS (System State Machine)
Hệ thống được thiết kế theo mô hình State Machine đồng bộ hóa qua cơ chế Mutex và Semaphore để triệt tiêu hiện tượng tranh chấp tài nguyên phần cứng (Hardware Resource Contention):

Task 1 (mpu_monitor_task - Độ ưu tiên 5): Giám sát liên tục góc nghiêng vật lý thông qua bộ lọc nhiễu số. Đóng vai trò cấu hình đưa hệ thống vào giấc ngủ sâu và phân phối nguồn ngắt phần cứng (SIM_RX_PIN và WAKEUP_GPIO_PIN) khi tỉnh giấc.

Task 2 (central_control_task - Độ ưu tiên 4): Bộ não điều khiển trung tâm, quản lý Rơ-le bảo mật, đếm ngược cửa sổ thời gian 15 giây để xác thực Token mã hóa nhận từ luồng BLE.

Task 3 (sim_gps_network_task - Độ ưu tiên 4): Luồng độc quyền quản lý module SIM A7600E được bảo vệ bởi cổng gác xSimMutex. Chịu trách nhiệm mở kết nối mạng 4G LTE, thu thập tọa độ vệ tinh GPS và đồng bộ trực tiếp lên Firebase Realtime Database.

Task 4 (ble_auth_task - Độ ưu tiên 3): Khởi chạy tầng RF phát quảng bá gói tin định danh UUID bảo mật để bắt cặp với App Flutter của chủ xe khi xe chuyển sang trạng thái chờ xác thực (STATE_VERIFYING).

Task 5 (remote_find_task - Độ ưu tiên 5): Tiến trình xử lý độc lập kịch bản tìm xe từ xa. Phân tích chuỗi dữ liệu UART để xác nhận cuộc gọi đến, hú còi bíp bíp ngắn và mượn luồng trạng thái mạng của Task 3 để cập nhật bản đồ thời gian thực.
-------------------------------------------------------------------------------------------------------------------------
#Thành Phần Phần Cứng (Hardware Components)
Vi điều khiển: ESP32-C3 SoC (Hỗ trợ BLE 5.0, Wi-Fi, Tiết kiệm năng lượng phần cứng).

Cảm biến góc/Gia tốc: MPU6050 (Giao tiếp I2C, sử dụng chân ngắt cấu hình cứng INT).

Module Viễn thông: SIM A7600E 4G LTE & GPS (Giao tiếp UART1, sử dụng chân ngủ tiết kiệm điện DTR/SLEEP).

Cơ cấu chấp hành: Rơ-le 5V DC ngắt dòng khóa điện xe (Mạch hở chân NO), Còi chip (Buzzer) phát tần số báo động.
-------------------------------------------------------------------------------------------------------------------------
#Công Nghệ Sử Dụng (Software Stack)
Framework chính: ESP-IDF v5.x (Chính chủ Expressif, tối ưu hóa C mã nguồn thấp).

Hệ điều hành: FreeRTOS (Multitasking, Mutexes, Semaphores, Context Switching).

Bluetooth Stack: NimBLE Framework (Dòng RAM thấp, hiệu năng quét sóng cao).

Giao thức mạng: AT Commands Parser, HTTP/REST Client kết nối Cloud Firebase.
-------------------------------------------------------------------------------------------------------------------------
#Hướng Dẫn Cài Đặt & Triển Khai
1. Chuẩn bị môi trường
Cài đặt công cụ lập trình ESP-IDF (Khuyên dùng phiên bản v5.1 trở lên hoặc extension trên VS Code).

2. Nhân bản mã nguồn
Bash
git clone https://github.com/your-username/esp32-motorbike-tracking.git
cd esp32-motorbike-tracking
3. Cấu hình phần cứng
Chỉnh sửa cấu hình chân GPIO và số điện thoại chủ xe trong file main/main.c:

----------------------------------------------
#define OWNER_PHONE_NUMBER "xxxxxxxxx"
#define SIM_SLEEP_PIN      3  
// Các chân I2C cho MPU6050 và UART cho SIM được cấu hình tương ứng trong thư mục components/
4. Biên dịch và nạp code xuống mạch
Kết nối board mạch ESP32-C3 qua cổng USB-UART và chạy chuỗi lệnh:

Bash
# Dọn dẹp bản build cũ
idf.py fullclean

# Biên dịch dự án, nạp phần sụn và mở cổng giám sát log UART
idf.py build flash monitor

#License
Được phát triển bởi Nguyen Duc Thuan with Gemini

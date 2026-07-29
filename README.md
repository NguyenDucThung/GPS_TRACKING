ĐÂY LÀ ĐỀ TÀI QUẢN LÝ PHƯƠNG TIỆN CÁ NHÂN( XE MÁY) THEO THỜI GIAN THỰC
CHỨC NĂNG CHÍNH:
TÌM XE TRONG BÃI.
CẢNH BÁO KHI CÓ TRỘM.
THEO DÕI VỊ TRÍ CỦA XE.
XEM NHẬT KÝ HÀNH TRÌNH BẰNG FIRESTORE.

Cách mở:.
VS code -> mở terminal -> chuyển sang CMD thay vì PS.
Gõ: C:\esp\v6.0.1\esp-idf\export.bat (ở đây dùng IDF-v6.0.1).
   idf.py set-target esp32c3 (trong đồ án này dùng esp32c3).
   idf.py menuconfig -> config component -> Bluetooth -> Host -> Nimble -> ctrl S .
   idf.py -p COMX flash .


   

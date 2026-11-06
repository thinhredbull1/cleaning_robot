1. Quy trình Auto Calibration mới
Bước 1: Chọn bánh cần calibration

UI hiển thị robot:
text

                 FRONT
          ┌─────────────────┐
          │                 │
     ┌────┤                 ├────┐
     │ LU │                 │ RU │
     └────┤                 ├────┘
          │                 │
          │      ROBOT      │
          │                 │
     ┌────┤                 ├────┐
     │ LD │                 │ RD │
     └────┤                 ├────┘
          │                 │
          └─────────────────┘
                  REAR

Click vào LU, phần mềm sẽ:

    Gửi lệnh cho motor channel 0 chạy.
    Đọc encoder feedback của cả 4 channel.
    Xác định channel encoder nào đang thay đổi.
    Ghi nhận mapping motor và encoder.

Bước 2: Xác nhận bánh thực tế

Ví dụ bạn click LU nhưng quan sát thực tế bánh RD đang quay.

Bạn click RD trên hình robot.

Kết quả:
text

Motor channel 0  →  RD

Phần mềm tự chuyển cặp PWM/DIR của channel 0 sang index RD.
Bước 3: Tự nhận diện encoder

Trong lúc motor chạy, Python theo dõi:
text

Encoder 0:  0 → 0
Encoder 1:  0 → 0
Encoder 2:  0 → 125
Encoder 3:  0 → 0

Kết quả:
text

Motor channel 0 → Physical RD
Encoder channel 2 → Physical RD

Như vậy, Python tự biết:

    PWM/DIR lấy từ motor channel 0.
    ENCA/ENCB lấy từ encoder channel 2.

Không cần nhập mapping encoder thủ công.
Bước 4: Xác định chiều encoder

Python so sánh encoder trước và sau khi bánh quay.

    Encoder tăng: ghi nhận dấu dương.
    Encoder giảm: ghi nhận dấu âm.

Tuy nhiên, dấu này cần được đối chiếu với chiều quay chuẩn của bánh xe. Vì thế UI sẽ yêu cầu xác nhận chiều quay thực tế một lần.
Bước 5: Xuất Arduino configuration

Kết quả cuối cùng tự sinh:
cpp

const int pwm[] = {...};
const int dir[] = {...};

const int enca[] = {...};
const int encb[] = {...};

const int dir_M_L_UP = ...;
const int dir_M_L_DOWN = ...;
const int dir_M_R_UP = ...;
const int dir_M_R_DOWN = ...;

int dir_encod[] = {...};

2. Python Auto Calibration – phiên bản mới

Mình sẽ sử dụng nguyên giao thức Serial của bạn:

    Baudrate: 57600.
    Lệnh điều khiển: motor_index/speed;.
    Encoder feedback: LU/LD&RU*RD;.

Có một điểm cần lưu ý: Arduino hiện publish 4 giá trị encoder nhưng chưa thấy đoạn cập nhật publish_encoder[] từ encoder_count[] trong code bạn gửi. Mình sẽ bổ sung phần đó để Auto Calibration nhận được feedback thực tế.
Cách hoạt động

Với mỗi motor channel từ 0 đến 3:

    Python lưu encoder hiện tại làm mốc.
    Chạy motor với tốc độ thấp.
    Bạn quan sát bánh nào thực sự quay và click bánh đó trên hình robot.
    Motor tự dừng.
    Python so sánh 4 encoder trước và sau khi chạy.
    Encoder nào thay đổi lớn nhất sẽ được ghép với bánh vừa xác nhận.
    Python tính chiều dir_encod.
    Chuyển sang motor channel tiếp theo.

Sau 4 lần, chương trình sinh cấu hình hoàn chỉnh.

Lưu ý: Feedback hiện tại đủ để nhận diện encoder channel và chiều đếm, nhưng không thể phân biệt độc lập chân A với chân B. Ta sẽ giữ nguyên cặp enca[]/encb[] của từng channel và hoán đổi cả cặp. Đây là cách phù hợp với thuật toán interrupt hiện tại của bạn.
File auto_calibration.py
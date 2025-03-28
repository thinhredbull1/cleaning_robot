#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>
void sendSerial(int command) {
        if (command == 0) {
            std::cout<<"RESET"<<std::endl;
        } else {
            throw std::invalid_argument("Lệnh không hợp lệ cho sendSerial(int)");
        }
    }

    // Phiên bản 2: sendSerial(1, 12, 12) -> "12/12;"
    void sendSerial(int command, int left, int right) {
        if (command == 1) {
            std::ostringstream oss;
            oss << left << "/" << right << ";";
            std::cout<<oss.str()<<std::endl;
          
        } else {
            throw std::invalid_argument("Lệnh không hợp lệ cho sendSerial(int, int, int)");
        }
    }

    // Phiên bản 3: sendSerial(2, 0.3, 0.5) -> "0.3L0.5R;"
    void sendSerial(int command, double left, double right) {
        if (command == 2) {
            std::ostringstream oss;
            oss << left << "L" << right << "R;";
            std::cout<<oss.str()<<std::endl;
           
        } else {
            throw std::invalid_argument("Lệnh không hợp lệ cho sendSerial(int, double, double)");
        }
    }
int main() {
    std::string s = "0.12x1.255y1.5v0.5"; // Chuỗi đầu vào của bạn
    
    // Bước 1: Tìm vị trí của 'x' và 'y'
    size_t x_pos = s.find('x');
    size_t y_pos = s.find('y', x_pos + 1);
    size_t v_pos= s.find('v',y_pos+1);
    
    // Kiểm tra xem 'x' và 'y' có tồn tại không
    if (x_pos == std::string::npos || y_pos == std::string::npos) {
        throw std::invalid_argument("Chuỗi không đúng định dạng: thiếu 'x' hoặc 'y'");
    }
    
    // Bước 2: Tách chuỗi thành ba phần
    std::string number_1_str = s.substr(0, x_pos);              // Từ đầu đến 'x'
    std::string number_2_str = s.substr(x_pos + 1, y_pos - x_pos - 1); // Từ sau 'x' đến 'y'
    std::string number_3_str = s.substr(y_pos + 1,v_pos-y_pos-1);             // Từ sau 'y' đến cuối
    std::string number_4_str = s.substr(v_pos + 1);
    // Bước 3: Chuyển đổi các phần chuỗi thành số
    double number_1, number_2, number_3,number_4;
    try {
        number_1 = std::stod(number_1_str);
        number_2 = std::stod(number_2_str);
        number_3 = std::stod(number_3_str);
        number_4=std::stod(number_4_str);
    } catch (const std::invalid_argument& e) {
        throw std::invalid_argument("Một trong các phần không phải là số hợp lệ");
    } catch (const std::out_of_range& e) {
        throw std::out_of_range("Một trong các số vượt quá phạm vi của double");
    }
    
    // In kết quả
    std::cout << "number_1: " << number_1 << std::endl;
    std::cout << "number_2: " << number_2 << std::endl;
    std::cout << "number_3: " << number_3 << std::endl;
    std::cout << "number_4: " << number_4 << std::endl;
    sendSerial(0);
    sendSerial(1,10,10);
    sendSerial(2,0.2323,0.34343);
    return 0;
}
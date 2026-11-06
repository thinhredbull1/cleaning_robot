1. Tạo cổng Serial ảo
    sudo apt install socat
    socat -d -d pty,raw,echo=0 pty,raw,echo=0
2. Code Fake Arduino
    python fake_arduino.py


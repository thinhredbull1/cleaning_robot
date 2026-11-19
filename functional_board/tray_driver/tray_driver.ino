#include <Arduino.h>
#include <WiFi.h>
#include <AccelStepper.h>

// =================================================
// 1. WIFI CONFIG
// =================================================

const char* WIFI_SSID = "ESP32_MASTER";
const char* WIFI_PASS = "12345678";

const char* SERVER_IP = "192.168.4.1";
const uint16_t SERVER_PORT = 5000;
int lastButton = 0;

WiFiClient client;
bool uvState[3] = {
    true, true, true
};

// =================================================
// 2. MOTOR GPIO
// =================================================

// Motor 1
#define STEP1 33
#define DIR1  32

// Motor 2
#define STEP2 26
#define DIR2  25

// Motor 3
#define STEP3 13
#define DIR3  27

// Analog button
#define SW_PIN 36

// Limit switches
#define CTHT1 39
#define CTHT2 34
#define CTHT3 35

// =================================================
// 3. ACCELSTEPPER
// =================================================

#define DRIVER_TYPE 1

AccelStepper motor1(DRIVER_TYPE, STEP1, DIR1);
AccelStepper motor2(DRIVER_TYPE, STEP2, DIR2);
AccelStepper motor3(DRIVER_TYPE, STEP3, DIR3);

AccelStepper* motors[3] = {
    &motor1,
    &motor2,
    &motor3
};
// =================================================
// DOOR STATE MACHINE
// =================================================

enum DoorState
{
    DOOR_CLOSED,       // Cửa đóng, đang khóa
    UNLOCKING,         // Đã mở khóa, chờ CTHT OFF
    OPENING,           // Motor đang chạy đến B
    DOOR_OPEN,         // Cửa mở, chờ nhấn nút
    CLOSING,           // Motor đang chạy về HOME
    WAIT_LIMIT_ON      // Chờ CTHT ON để khóa lại
};

DoorState doorState[3] =
{
    DOOR_CLOSED,
    DOOR_CLOSED,
    DOOR_CLOSED
};
// =================================================
// SYNC HANDSHAKE
// =================================================

bool syncReady = false;
bool initialSyncCompleted = false;

unsigned long lastSyncRequest = 0;

#define SYNC_RETRY_TIME 2000
// Xác định trạng thái cửa
bool doorIsOpen[3] = {false, false, false};

// Chống kích hoạt nhiều lần
bool buttonAllowed[3] = {true, true, true};
// =================================================
// INITIAL SYSTEM SYNC
// =================================================
// =================================================
// 4. MOTOR PARAMETERS
// =================================================

float motorSpeed[3] = {
    5000, 5000, 5000
};

float motorAccel[3] = {
    5000, 5000, 5000
};

bool invertDir[3] = {
    false, false, false
};
// Danh dau motor dang trong chu trinh tu dong
bool autoMotorBusy[3] = {false, false, false};

// =================================================
// 5. BUTTON AND LIMIT SWITCH
// =================================================
// Vi tri A (HOME)
long positionA[3] = {
    0,      // Motor 1
    0,      // Motor 2
    0       // Motor 3
};

// Vi tri B
long positionB[3] = {
    10000,  // Motor 1
    10000,  // Motor 2
    10000   // Motor 3
};

// Thoi gian xac nhan cong tac
#define LIMIT_OFF_MOVE_TIME 6000
#define LIMIT_ON_HOME_TIME  3000

// Timer OFF
unsigned long motorOffStart[3] = {0, 0, 0};

// Timer ON
unsigned long motorOnStart[3] = {0, 0, 0};

// Trang thai timer
bool motorOffTiming[3] = {false, false, false};
bool motorOnTiming[3] = {false, false, false};

// Danh dau da thuc hien lenh
bool motorMovedToB[3] = {false, false, false};
bool motorReturnedHome[3] = {false, false, false};

// =================================================
// DIEU KHIEN LED RGB
// LOCK ON  -> GREEN
// LOCK OFF -> RED
// =================================================
int limitPins[3] = {
    CTHT1,
    CTHT2,
    CTHT3
};

bool limitState[3] = {
    false, false, false
};

// Quy uoc cong tac:
// LOW  = OFF
// HIGH = ON

#define LIMIT_OFF_LEVEL LOW
#define LIMIT_ON_LEVEL  HIGH

// Thoi gian OFF de tu dong dao LOCK
#define LOCK_AUTO_TIME 3000

unsigned long limitOffStart[3] = {
    0, 0, 0
};

bool limitTimerRunning[3] = {
    false, false, false
};

bool autoLockTriggered[3] = {
    false, false, false
};

// Trang thai LOCK duoc quan ly tai Slave
// false = OFF
// true  = ON

bool lockState[3] = {
    false, false, false
};

// =================================================
// 6. WIFI CONNECTION
// =================================================

unsigned long lastReconnect = 0;
bool wifiConnecting = false;
unsigned long wifiAttemptStart = 0;
void initialSync()
{
    if (initialSyncCompleted)
        return;

    Serial.println("\n========== INITIAL SYNC ==========");

    // 1. Đồng bộ UV
    for (int i = 0; i < 3; i++)
    {
        sendCommand(
            "UV " + String(i + 1) +
            " " + String(uvState[i] ? 1 : 0)
        );
    }

    // 2. Đọc trạng thái nút nhấn
    lastButton = readAnalogButton(analogRead(SW_PIN));

    Serial.printf("BUTTON STATE: %d\n", lastButton);

    // 3. Đọc công tắc hành trình
    for (int i = 0; i < 3; i++)
    {
        limitState[i] =
            (digitalRead(limitPins[i]) == LIMIT_ON_LEVEL);

        Serial.printf(
            "LIMIT %d: %s\n",
            i + 1,
            limitState[i] ? "ON" : "OFF"
        );
    }

    // 4. Khởi tạo trạng thái cửa
    for (int i = 0; i < 3; i++)
    {
        if (limitState[i])
        {
            doorState[i] = DOOR_CLOSED;
            doorIsOpen[i] = false;
        }
        else
        {
            doorState[i] = DOOR_OPEN;
            doorIsOpen[i] = true;
        }
    }

    initialSyncCompleted = true;

    Serial.println("INITIAL SYNC COMPLETED");
    Serial.println("=================================\n");
}


void processSyncResponse(String data)
{
    data.trim();
    data.toUpperCase();

    if (data == "SYNC_OK")
    {
        if (syncReady)
            return;

        Serial.println("\n[RX MASTER] SYNC_OK");

        // Xác nhận đã đồng bộ thành công
        syncReady = true;

        Serial.println("[SYNC] MASTER ACKNOWLEDGED");

        // Chỉ đồng bộ trạng thái ban đầu một lần
        if (!initialSyncCompleted)
        {
            initialSync();
        }
    }
}

void startOpening(int index)
{
    if (index < 0 || index >= 3)
        return;

    if (doorState[index] != DOOR_CLOSED)
        return;

    Serial.printf("\nDOOR %d: START OPENING\n", index + 1);

    // 1. Mở khóa
    setLock(index, false);

    // 2. LED xanh
    sendCommand(
        "LED " + String(index + 1) + " GREEN 1"
    );

    // 3. Chuyển sang trạng thái chờ CTHT OFF
    doorState[index] = UNLOCKING;

    Serial.printf(
        "DOOR %d: WAITING FOR LIMIT SWITCH OFF\n",
        index + 1
    );
}
void startClosing(int index)
{
    if (index < 0 || index >= 3)
        return;

    if (doorState[index] != DOOR_OPEN)
        return;

    Serial.printf("\nDOOR %d: START CLOSING\n", index + 1);

    // 1. Di chuyển motor về HOME
    motors[index]->moveTo(positionA[index]);

    doorState[index] = CLOSING;

    Serial.printf(
        "DOOR %d: MOTOR MOVING TO HOME\n",
        index + 1
    );
}



void connectWiFi()
{
    if (WiFi.status() == WL_CONNECTED)
        return;

    if (wifiConnecting)
        return;

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    wifiConnecting = true;
    wifiAttemptStart = millis();

    Serial.println("Connecting WiFi...");
}


// =================================================
// 7. TCP SERVER CONNECTION
// =================================================
void sendSyncRequest()
{
    if (!client.connected() || syncReady)
        return;

    if (millis() - lastSyncRequest < SYNC_RETRY_TIME &&
        lastSyncRequest != 0)
        return;

    lastSyncRequest = millis();

    client.println("SYNC");

    Serial.println("[TX MASTER] SYNC REQUEST");
}

void connectServer()
{
    if (client.connected())
        return;

    client.stop();

    syncReady = false;

    Serial.println("Connecting TCP Server...");

    if (client.connect(SERVER_IP, SERVER_PORT))
    {
        Serial.println("TCP SERVER CONNECTED!");

        lastSyncRequest = 0;
    }
    else
    {
        Serial.println("TCP connection failed!");
    }
}


// =================================================
// 8. SEND COMMAND TO MASTER
// =================================================

void sendCommand(String command)
{
    command.trim();

    if (command.length() == 0)
        return;

    if (!client.connected())
        return;

    // Chưa nhận SENSOR đầu tiên thì không gửi lệnh
    if (!syncReady)
    {
        Serial.println("SYSTEM NOT SYNCED - COMMAND BLOCKED");
        return;
    }

    client.println(command);

    Serial.print("[TX MASTER] ");
    Serial.println(command);
}


// =================================================
// 9. CONTROL LOCK
// =================================================

void updateLockLED(int index)
{
    if (index < 0 || index >= 3)
        return;

    String command;

    if (doorIsOpen[index] ||
        doorState[index] == UNLOCKING ||
        doorState[index] == OPENING)
    {
        command =
            "LED " + String(index + 1) + " GREEN 1";
    }
    else
    {
        command =
            "LED " + String(index + 1) + " RED 1";
    }

    sendCommand(command);
}


// =================================================
// DIEU KHIEN UV
// =================================================

void setUV(int index, bool state)
{
    if (index < 0 || index >= 3)
        return;

    uvState[index] = state;

    String command =
        "UV " + String(index + 1) +
        " " + String(state ? 1 : 0);

    sendCommand(command);

    Serial.printf(
        "UV %d = %s\n",
        index + 1,
        state ? "ON" : "OFF"
    );
}
void setLock(int index, bool state)
{
    if (index < 0 || index >= 3)
        return;

    lockState[index] = state;

    Serial.printf(
        "LOCK %d = %s\n",
        index + 1,
        state ? "ON" : "OFF"
    );

    updateLockLED(index);
}

// =================================================
// AUTO MOTOR POSITION CONFIGURATION
// =================================================


// Dao trang thai LOCK
void toggleLock(int index)
{
    if (index < 0 || index >= 3)
        return;

    setLock(index, !lockState[index]);
}
// =================================================
// RESET SYSTEM
// =================================================

void resetSystem()
{
    Serial.println("\n================================");
    Serial.println("       SYSTEM RESET");
    Serial.println("================================");

    // =====================================
    // 1. STOP ALL MOTORS
    // =====================================

    for (int i = 0; i < 3; i++)
    {
        motors[i]->stop();

        // Neu motor van dang giam toc,
        // danh dau dang ban de khong doc CTHT
        autoMotorBusy[i] = motors[i]->isRunning();
    }
    for (int i = 0; i < 3; i++)
    {
        // Chi reset toa do khi motor da dung hoan toan
        if (!motors[i]->isRunning())
        {
            motors[i]->setCurrentPosition(0);

            Serial.printf(
                "MOTOR %d POSITION RESET TO 0\n",
                i + 1
            );
        }
        else
        {
            Serial.printf(
                "MOTOR %d STILL MOVING - POSITION NOT RESET\n",
                i + 1
            );
        }
    }
    Serial.println("[RESET] Motors stopping");

    // =====================================
    // 2. RESET AUTO MOTOR STATE
    // =====================================

    for (int i = 0; i < 3; i++)
    {
        motorOffStart[i] = 0;
        motorOnStart[i] = 0;

        motorOffTiming[i] = false;
        motorOnTiming[i] = false;

        motorMovedToB[i] = false;
        motorReturnedHome[i] = false;

        // Khong reset autoMotorBusy o day
        // de motor giam toc xong moi kiem tra CTHT
    }

    Serial.println("[RESET] Auto motor state cleared");

    // =====================================
    // 3. RESET LIMIT SWITCH STATE
    // =====================================

    for (int i = 0; i < 3; i++)
    {
        limitState[i] =
            (digitalRead(limitPins[i]) == LIMIT_ON_LEVEL);

        limitOffStart[i] = 0;
        limitTimerRunning[i] = false;

        autoLockTriggered[i] = false;
    }
    Serial.println("[RESET] Limit switch timers cleared");

    // =====================================
    // 4. RESET LOCK AND UV
    // =====================================

    for (int i = 0; i < 3; i++)
    {
        // Tat UV
        setUV(i, false);

        // Tat LOCK va cap nhat LED
        // setLock(i, false);

        // // Gui lenh cap nhat LOCK cho Master
        // sendCommand(
        //     "LOCK " + String(i + 1) + " 0"
        // );
    }

    Serial.println("[RESET] LOCK and UV OFF");

    // =====================================
    // 6. RESET MOTOR CONFIGURATION
    // =====================================

    for (int i = 0; i < 3; i++)
    {
        motorSpeed[i] = 5000;
        motorAccel[i] = 5000;

        invertDir[i] = false;

        motors[i]->setMaxSpeed(motorSpeed[i]);

        motors[i]->setAcceleration(motorAccel[i]);

        motors[i]->setPinsInverted(false);
    }

    Serial.println("[RESET] Motor configuration restored");

    // =====================================
    // 7. RESET OTHER TIMER
    // =====================================

    // Khong reset WiFi/TCP connection
    // De tranh mat ket noi voi Master

    Serial.println("================================");
    Serial.println("       RESET COMPLETED");
    Serial.println("================================\n");
}

// =================================================
// 10. MOTOR UPDATE
// =================================================
// =================================================
// =================================================
// DOOR STATE MACHINE UPDATE
// =================================================

void updateDoorState()
{
    for (int i = 0; i < 3; i++)
    {
        bool currentLimit =
            (digitalRead(limitPins[i]) == LIMIT_ON_LEVEL);

        switch (doorState[i])
        {
            // =====================================
            // 1. DOOR CLOSED
            // =====================================

            case DOOR_CLOSED:
            {
                doorIsOpen[i] = false;
                break;
            }

            // =====================================
            // 2. UNLOCKING
            // CHO CTHT OFF
            // =====================================

            case UNLOCKING:
            {
                if (!currentLimit)
                {
                    Serial.printf(
                        "DOOR %d: LIMIT OFF -> OPEN MOTOR\n",
                        i + 1
                    );

                    motors[i]->moveTo(positionB[i]);

                    doorState[i] = OPENING;
                }

                break;
            }

            // =====================================
            // 3. OPENING
            // MOTOR -> POSITION B
            // =====================================

            case OPENING:
            {
                if (motors[i]->distanceToGo() == 0)
                {
                    Serial.printf(
                        "DOOR %d: OPENED SUCCESSFULLY\n",
                        i + 1
                    );

                    doorIsOpen[i] = true;

                    doorState[i] = DOOR_OPEN;
                }

                break;
            }

            // =====================================
            // 4. DOOR OPEN
            // CHO NUT NHAN
            // =====================================

            case DOOR_OPEN:
            {
                doorIsOpen[i] = true;

                break;
            }

            // =====================================
            // 5. CLOSING
            // MOTOR -> HOME
            // =====================================

            case CLOSING:
            {
                if (motors[i]->distanceToGo() == 0)
                {
                    Serial.printf(
                        "DOOR %d: MOTOR RETURNED HOME\n",
                        i + 1
                    );

                    // LED đỏ
                    sendCommand(
                        "LED " + String(i + 1) + " RED 1"
                    );

                    doorIsOpen[i] = false;

                    // Chờ CTHT ON
                    doorState[i] = WAIT_LIMIT_ON;
                }

                break;
            }

            // =====================================
            // 6. WAIT LIMIT ON
            // =====================================

            case WAIT_LIMIT_ON:
            {
                if (currentLimit)
                {
                    Serial.printf(
                        "DOOR %d: LIMIT ON -> LOCK ON\n",
                        i + 1
                    );

                    // Bật LOCK
                    setLock(i, true);

                    doorState[i] = DOOR_CLOSED;

                    Serial.printf(
                        "DOOR %d: CYCLE COMPLETED\n",
                        i + 1
                    );
                }

                break;
            }
        }
    }
}




void updateMotors()
{
    for (int i = 0; i < 3; i++)
    {
        motors[i]->run();
    }
}

// =================================================
// 11. MOTOR MOVE
// =================================================

void moveMotor(int index, long steps)
{
    if (index < 0 || index >= 3)
        return;

    motors[index]->move(steps);

    Serial.printf(
        "M%d MOVE: %ld steps\n",
        index + 1,
        steps
    );
}

// =================================================
// 12. MOTOR SPEED
// =================================================

void setMotorSpeed(int index, float speed)
{
    if (index < 0 || index >= 3)
        return;

    if (speed <= 0 || speed > 3000)
        return;

    motorSpeed[index] = speed;

    motors[index]->setMaxSpeed(speed);

    Serial.printf(
        "M%d SPEED = %.1f\n",
        index + 1,
        speed
    );
}

// =================================================
// 13. MOTOR ACCELERATION
// =================================================

void setMotorAcceleration(int index, float accel)
{
    if (index < 0 || index >= 3)
        return;

    if (accel <= 0 || accel > 5000)
        return;

    motorAccel[index] = accel;

    motors[index]->setAcceleration(accel);

    Serial.printf(
        "M%d ACCEL = %.1f\n",
        index + 1,
        accel
    );
}

// =================================================
// 14. MOTOR STOP
// =================================================

void stopMotor(int index)
{
    if (index < 0 || index >= 3)
        return;

    motors[index]->stop();

    Serial.printf(
        "M%d STOPPING\n",
        index + 1
    );
}

void stopAllMotors()
{
    for (int i = 0; i < 3; i++)
    {
        motors[i]->stop();
    }

    Serial.println("ALL MOTORS STOPPING");
}

// =================================================
// 15. MOTOR DIRECTION
// =================================================

void reverseMotor(int index)
{
    if (index < 0 || index >= 3)
        return;

    invertDir[index] = !invertDir[index];

    motors[index]->setPinsInverted(
        invertDir[index]
    );

    Serial.printf(
        "M%d DIRECTION: %s\n",
        index + 1,
        invertDir[index] ? "INVERTED" : "NORMAL"
    );
}

// =================================================
// 16. READ ANALOG BUTTON
// =================================================

int readAnalogButton(float adcAverage) {

  // int adcValue = analogRead(SW_PIN);
  // Serial.println(adcAverage);
  /*
    Nguong tham khao theo mang dien tro:

    R2 = 2K
    R3 = 4K
    R4 = 6K
    R1 = 10K

    Gia tri ADC thuc te can hieu chinh.
  */

  if (adcAverage > 3000 && adcAverage < 3700) {
    return 1; ///3195 upper
  }

  if (adcAverage > 1600 && adcAverage <= 2000) {
    return 2; //2539 middle
  }

  if (adcAverage > 2200 && adcAverage <= 2450) { 
    return 3; //2395 bottom
  }

  return 0;
}

// =================================================
// 17. BUTTON PRESSED
// =================================================
void onButtonPressed(int button)
{
    int index = button - 1;

    if (index < 0 || index >= 3)
        return;

    Serial.printf(
        "\nBUTTON %d PRESSED\n",
        button
    );

    switch (doorState[index])
    {
        case DOOR_CLOSED:

            startOpening(index);

            break;

        case DOOR_OPEN:

            startClosing(index);

            break;

        default:

            Serial.printf(
                "DOOR %d BUSY - BUTTON IGNORED\n",
                button
            );

            break;
    }
}



// =================================================
// 18. UPDATE BUTTONS
// =================================================

void updateButtons()
{
    static int adcSum = 0;
    static int sampleCount = 0;

    static unsigned long lastSample = 0;

    // static int lastButton = 0;

    // Lấy mẫu ADC mỗi 50ms
    if (millis() - lastSample >= 25)
    {
        lastSample = millis();
        
        int adcNew = analogRead(SW_PIN);
        if(adcNew >100)
        {
          sampleCount++;
          adcSum+=adcNew;
          int sample=10;
          // Đủ 10 mẫu thì tính trung bình
          if (sampleCount >= sample)
          {
              int adcAverage = adcSum / sample;

              int currentButton = readAnalogButton(adcAverage);

              // Chỉ xử lý khi xác định được nút nhấn mới
              if (currentButton != 0 && currentButton != lastButton)
              {
                  onButtonPressed(currentButton);
              }

              lastButton = currentButton;

              // Reset để chuẩn bị lấy mẫu tiếp
              adcSum = 0;
              sampleCount = 0;
          }
        }
        else{
          lastButton =0;
            adcSum = 0;
              sampleCount = 0;
          }
    }

    // Đọc công tắc hành trình mỗi 150ms
    static unsigned long lastLimitRead = 0;

    if (millis() - lastLimitRead >= 150)
    {
        lastLimitRead = millis();

        limitState[0] = digitalRead(CTHT1);
        limitState[1] = digitalRead(CTHT2);
        limitState[2] = digitalRead(CTHT3);
    }
}


// =================================================
// 19. READ LIMIT SWITCHES
// =================================================

void readLimitSwitches()
{
    for (int i = 0; i < 3; i++)
    {
        limitState[i] =
            (digitalRead(limitPins[i]) == LIMIT_ON_LEVEL);
    }
}

// =================================================
// 20. AUTO LOCK CONTROL
// =================================================

// =================================================
// AUTO UV + LOCK + LED
// =================================================

void updateAutoLock()
{
    unsigned long now = millis();

    for (int i = 0; i < 3; i++)
    {
        int state = digitalRead(limitPins[i]);

        // =====================================
        // CTHT ON
        // =====================================

        if (state == LIMIT_ON_LEVEL)
        {
            limitState[i] = true;

            // Reset timer
            limitTimerRunning[i] = false;

            // Bat UV neu dang tat
            if (!uvState[i])
            {
                setUV(i, true);
            }
        }

        // =====================================
        // CTHT OFF
        // =====================================

        else
        {
            limitState[i] = false;

            // Tat UV ngay lap tuc
            if (uvState[i])
            {
                setUV(i, false);
            }

            // Bat dau dem 3 giay
            if (!limitTimerRunning[i])
            {
                limitOffStart[i] = now;

                limitTimerRunning[i] = true;
            }

            // OFF lien tuc du 3 giay
            // if (limitTimerRunning[i] &&
            //     now - limitOffStart[i] >= LOCK_AUTO_TIME)
            // {
            //     // Chi tat LOCK neu dang ON
            //     if (lockState[i])
            //     {
            //         Serial.printf(
            //             "CTHT %d OFF 3S -> LOCK %d OFF\n",
            //             i + 1,
            //             i + 1
            //         );

            //         setLock(i, false);
            //     }

            //     // Khong lap lai cho den khi CTHT ON
            //     limitTimerRunning[i] = false;
            // }
        }
    }
}



// =================================================
// 21. MOTOR STATUS
// =================================================

void printMotorStatus(int index)
{
    Serial.println("-------------------------");

    Serial.printf(
        "MOTOR %d\n",
        index + 1
    );

    Serial.printf(
        "Limit: %s\n",
        limitState[index] ? "ACTIVE" : "NORMAL"
    );

    Serial.printf(
        "Position: %ld\n",
        motors[index]->currentPosition()
    );

    Serial.printf(
        "Target: %ld\n",
        motors[index]->targetPosition()
    );

    Serial.printf(
        "Speed: %.2f\n",
        motors[index]->speed()
    );

    Serial.println("-------------------------");
}

void printAllStatus()
{
    readLimitSwitches();

    Serial.println("\n======= MOTOR STATUS =======");

    for (int i = 0; i < 3; i++)
    {
        printMotorStatus(i);
    }

    Serial.println("======= LOCK STATUS ========");

    for (int i = 0; i < 3; i++)
    {
        Serial.printf(
            "LOCK %d: %s\n",
            i + 1,
            lockState[i] ? "ON" : "OFF"
        );
    }
}

// =================================================
// 22. PROCESS MOTOR COMMAND
// =================================================

bool processMotorCommand(String cmd)
{
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "ALL STOP")
    {
        stopAllMotors();
        return true;
    }

    if (cmd == "STATUS")
    {
        printAllStatus();

        // Dong thoi yeu cau Master in trang thai
        sendCommand("STATUS");

        return true;
    }

    if (cmd.startsWith("M1"))
    {
        // Motor 1
    }
    else if (cmd.startsWith("M2"))
    {
        // Motor 2
    }
    else if (cmd.startsWith("M3"))
    {
        // Motor 3
    }
    else
    {
        return false;
    }

    int index = -1;

    if (cmd.startsWith("M1")) index = 0;
    if (cmd.startsWith("M2")) index = 1;
    if (cmd.startsWith("M3")) index = 2;

    String command = cmd.substring(2);

    command.trim();

    // MOVE
    if (command.startsWith("MOVE"))
    {
        long steps = command.substring(4).toInt();

        moveMotor(index, steps);
    }

    // SPEED
    else if (command.startsWith("SPEED"))
    {
        float speed = command.substring(5).toFloat();

        setMotorSpeed(index, speed);
    }

    // ACCEL
    else if (command.startsWith("ACCEL"))
    {
        float accel = command.substring(5).toFloat();

        setMotorAcceleration(index, accel);
    }

    // STOP
    else if (command.startsWith("STOP"))
    {
        stopMotor(index);
    }

    // DIR
    else if (command.startsWith("DIR"))
    {
        reverseMotor(index);
    }

    // STATUS
    else if (command.startsWith("STATUS"))
    {
        printMotorStatus(index);
    }

    else
    {
        Serial.println("UNKNOWN MOTOR COMMAND");
    }

    return true;
}

// =================================================
// 23. PROCESS MASTER COMMAND
// =================================================

void processMasterCommand(String command)
{
    command.trim();
    command.toUpperCase();

    if (command.length() == 0)
        return;

    // LOCK: cap nhat trang thai noi bo
    if (command.startsWith("LOCK "))
    {
        int index = -1;

        if (command.startsWith("LOCK 1 ")) index = 0;
        if (command.startsWith("LOCK 2 ")) index = 1;
        if (command.startsWith("LOCK 3 ")) index = 2;

        if (index >= 0)
        {
            int state = command.substring(7).toInt();

            lockState[index] = (state == 1);

            updateLockLED(index);
        }
    }


    // ALLOFF
    if (command == "ALLOFF")
    {
        for (int i = 0; i < 3; i++)
        {
            lockState[i] = false;
        }
    }

    sendCommand(command);
}

// =================================================
// 24. PROCESS SERIAL
// =================================================

void processSerial()
{
    if (!Serial.available())
        return;

    String command = Serial.readStringUntil('\n');

    command.trim();
    command.toUpperCase();

    if (command.length() == 0)
        return;
    if (command == "RESET")
    {
        resetSystem();
        return;
    }

    // HELP
    if (command == "HELP")
    {
        Serial.println("\n========== COMMAND LIST ==========");

        Serial.println("\n--- MOTOR ---");

        Serial.println("M1 MOVE 1000");
        Serial.println("M1 SPEED 500");
        Serial.println("M1 ACCEL 300");
        Serial.println("M1 STOP");
        Serial.println("M1 DIR");
        Serial.println("M1 STATUS");

        Serial.println("M2 MOVE 1000");
        Serial.println("M3 MOVE 1000");

        Serial.println("ALL STOP");

        Serial.println("\n--- BUTTON / LIMIT ---");

        Serial.println("BUTTON 1 -> LOCK 1");
        Serial.println("BUTTON 2 -> LOCK 2");
        Serial.println("BUTTON 3 -> LOCK 3");

        Serial.println("LIMIT OFF 3s -> AUTO TOGGLE LOCK");

        Serial.println("\n--- MASTER CONTROL ---");

        Serial.println("LED 1 RED 1");
        Serial.println("LED 2 GREEN 1");
        Serial.println("UV 1 1");
        Serial.println("LOCK 1 1");
        Serial.println("BUZZER 1");

        Serial.println("READ");
        Serial.println("STATUS");
        Serial.println("ALLOFF");

        Serial.println("==================================");

        return;
    }

    // MOTOR COMMAND
    if (processMotorCommand(command))
        return;

    // MASTER COMMAND
    processMasterCommand(command);
}

// =================================================
// 25. RECEIVE SENSOR DATA
// =================================================

// =================================================
// 25. RECEIVE SENSOR DATA
// =================================================

void processSensor(String data)
{
    data.trim();

    if (!data.startsWith("SENSOR,"))
        return;

    Serial.println(data);

    // =====================================
    // FIRST SENSOR RECEIVED
    // =====================================

    if (!syncReady)
    {
        Serial.println("[SYNC] FIRST SENSOR RECEIVED");

        syncReady = true;

        // Thực hiện đồng bộ trạng thái ban đầu
        initialSync();
    }

    // =====================================
    // PROCESS SENSOR DATA
    // =====================================

    // Giữ phần xử lý SENSOR hiện tại của bạn
}



// =================================================
// 26. RECEIVE WIFI
// =================================================

void receiveWiFi()
{
    if (!client.connected())
        return;

    while (client.available())
    {
        String data = client.readStringUntil('\n');

        data.trim();

        if (data.length() > 0)
        {
            processSensor(data);
        }
    }
}

// =================================================
// 27. SETUP
// =================================================

void setup()
{
    Serial.begin(57600);

    Serial.setTimeout(100);

    analogReadResolution(12);
    for (int i = 0; i < 3; i++)
    {
        motors[i]->setCurrentPosition(0);
    }
    // Button
    pinMode(SW_PIN, INPUT);

    // Limit switches
    for (int i = 0; i < 3; i++)
    {
        pinMode(limitPins[i], INPUT);
    }

    // Motor configuration
    for (int i = 0; i < 3; i++)
    {
        motors[i]->setMaxSpeed(motorSpeed[i]);

        motors[i]->setAcceleration(motorAccel[i]);

        motors[i]->setPinsInverted(invertDir[i]);

        motors[i]->setMinPulseWidth(2);
    }

    Serial.println("\n================================");
    Serial.println(" ESP32 SLAVE SYSTEM STARTED");
    Serial.println("================================");

    Serial.println("3 STEPPER MOTORS READY");

    connectWiFi();

    if (WiFi.status() == WL_CONNECTED)
    {
        connectServer();
    }

    Serial.println("Type HELP for commands");
}

// =================================================
// 28. LOOP
// =================================================

void loop()
{
    updateMotors();

    // Chỉ đọc và xử lý nút khi đã SYNC
    if (syncReady)
    {
        updateButtons();
        updateAutoLock();
        updateDoorState();
    }

    processSerial();

    if (WiFi.status() == WL_CONNECTED)
    {
        wifiConnecting = false;

        if (!client.connected())
        {
            syncReady = false;

            if (millis() - lastReconnect >= 3000)
            {
                lastReconnect = millis();
                connectServer();
            }
        }
    }
    else
    {
        syncReady = false;

        if (wifiConnecting &&
            millis() - wifiAttemptStart >= 10000)
        {
            wifiConnecting = false;
            WiFi.disconnect();
        }

        if (!wifiConnecting &&
            millis() - lastReconnect >= 3000)
        {
            lastReconnect = millis();
            connectWiFi();
        }
    }

    receiveWiFi();
}

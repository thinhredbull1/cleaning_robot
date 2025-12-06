#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad_I2C.h>
#include <Keypad.h>

// Define I2C addresses
#define LCD_ADDR 0x27  // Common I2C address for LCD
#define KEYPAD_ADDR 0x20  // Example I2C address for PCF8574 keypad expander

// LCD setup
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);  // 16x2 LCD

// Keypad setup
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1', '4', '7', '*'},
  {'2', '5', '8', '0'},
  {'3', '6', '9', '#'},
  {'A', 'B', 'C', 'D'}
};
byte rowPins[ROWS] = {0, 1, 2, 3};  // Connect to PCF8574 P0-P3
byte colPins[COLS] = {4, 5, 6, 7};  // Connect to PCF8574 P4-P7
TwoWire *jwire = &Wire;   //test passing pointer to keypad lib
Keypad_I2C keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS, KEYPAD_ADDR);

// Pin definitions
#define LED_L_PIN 4
#define LED_R_PIN 5
#define BUZZER_PIN 3
#define RELAY_PIN 2

// State machine states
enum State {
  IDLE_,
  ENTER_PASSWORD,
  PROCESS_ACTION
};

// Variables
State currentState = IDLE_;
String uartCommand = "";
String targetPassword = "";
String enteredPassword = "";
bool passwordMode = false;

void setup() {
  // Initialize Serial
  Serial.begin(9600);
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  updateLCD("Idle", "");
  
  // Initialize Keypad
  keypad.begin(makeKeymap(keys));
  
  // Initialize pins
  pinMode(LED_L_PIN, OUTPUT);
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  
  digitalWrite(LED_L_PIN, LOW);
  digitalWrite(LED_R_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
}

void loop() {
  handleUART();
  handleKeypad();
  handleState();
}

// Function to update LCD
void updateLCD(String status, String input) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(status);
  lcd.setCursor(0, 1);
  lcd.print(input);
}

// Handle UART input
void handleUART() {
  if (Serial.available() > 0) {
    uartCommand = Serial.readStringUntil('\n');
    uartCommand.trim();
    
    if (currentState == IDLE_) {
      processCommand(uartCommand);
    }
  }
}

// Process UART command
void processCommand(String cmd) {
  if (cmd.startsWith("L_ON")) {
    digitalWrite(LED_L_PIN, HIGH);
    updateLCD("LED_L ON", "");
  } else if (cmd.startsWith("L_OFF")) {
    digitalWrite(LED_L_PIN, LOW);
    updateLCD("LED_L_OFF", "");
  }  if (cmd.startsWith("R_ON")) {
    digitalWrite(LED_R_PIN, HIGH);
    updateLCD("LED R ON", "");
  } else if (cmd.startsWith("R_OFF")) {
    digitalWrite(LED_R_PIN, LOW);
    updateLCD("LED2 OFF", "");
  } if (cmd.startsWith("B_ON")) {
    digitalWrite(BUZZER_PIN, HIGH);
    updateLCD("BUZZER ON", "");
  } else if (cmd.startsWith("B_OFF")) {
    digitalWrite(BUZZER_PIN, LOW);
    updateLCD("BUZZER OFF", "");
  }  if (cmd.startsWith("R_ON")) {
    digitalWrite(RELAY_PIN, HIGH);
    updateLCD("RELAY ON", "");
  } else if (cmd.startsWith("R_OFF")) {
    digitalWrite(RELAY_PIN, LOW);
    updateLCD("RELAY OFF", "");
  } if (cmd.startsWith("UNLOCK ")) {
    targetPassword = cmd.substring(7);
    if (targetPassword.length() == 8) {
      currentState = ENTER_PASSWORD;
      enteredPassword = "";
      updateLCD("Enter Password", "");
      passwordMode = true;
    } else {
      updateLCD("Invalid Pass Len", "");
    }
  } else {
    updateLCD("Unknown Command", "");
  }
}

// Handle keypad input
void handleKeypad() {
  if (passwordMode) {
    char key = keypad.getKey();
    // Serial.println(key);
    if (key) {
      if (key == '*') {  // Delete key
        if (enteredPassword.length() > 0) {
          enteredPassword = enteredPassword.substring(0, enteredPassword.length() - 1);
        }
      } else if (key == '#') {  // Enter key, but we check length automatically
        // Ignore, or use for something else
      } else {
        if (enteredPassword.length() < 8) {
          enteredPassword += key;
        }
      }
      updateLCD("Enter Password", enteredPassword);
      
      if (enteredPassword.length() == 8) {
        currentState = PROCESS_ACTION;
      }
    }
  }
}

// Handle state machine
void handleState() {
  switch (currentState) {
    case IDLE_:
      // Do nothing, wait for UART
      break;
      
    case ENTER_PASSWORD:
      // Handled in handleKeypad
      break;
      
    case PROCESS_ACTION:
      if (enteredPassword == targetPassword) {
        updateLCD("Access Granted", "");
        // Perform sequence of actions, e.g., buzz, LED blink, relay on
        digitalWrite(BUZZER_PIN, HIGH);
        delay(500);
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_L_PIN, HIGH);
        digitalWrite(LED_R_PIN, HIGH);
        // digitalWrite(RELAY_PIN, HIGH);
        delay(2000);
        digitalWrite(LED_L_PIN, LOW);
        digitalWrite(LED_R_PIN, LOW);
        digitalWrite(RELAY_PIN, LOW);
      } else {
        updateLCD("Access Denied", "");
        delay(2000);
      }
      passwordMode = false;
      currentState = IDLE_;
      updateLCD("Idle", "");
      break;
  }
}
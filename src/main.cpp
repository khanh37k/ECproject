#include <EEPROM.h>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ========== MOTOR L298N ==========
#define LDIR1 17    // IN1 - chiều quay motor trái
#define LDIR2 16    // IN2 - chiều quay motor trái
#define LPWM  5     // ENA - tốc độ motor trái

#define RDIR1 4     // IN3 - chiều quay motor phải
#define RDIR2 2     // IN4 - chiều quay motor phải
#define RPWM  15    // ENB - tốc độ motor phải

// PWM channels for the ESP32 LEDC peripheral
#define LPWM_CH 0
#define RPWM_CH 1

// ========== BUTTON ==========
#define BUTTON1 12  // nút nhấn 1 (trái)
#define BUTTON2 13  // nút nhấn 2 (giữa)
#define BUTTON3 22  // nút nhấn 3 (phải)

// ========== HC-SR04 ==========
#define US_TRIG 18
#define US_ECHO 19

// ========== EEPROM ==========
#define EEPROM_SIZE 40

// ========== CẢM BIẾN 8 KÊNH ==========
// Kênh 1→8 từ trái sang phải
// Kênh lẻ: D14, D27, D26, D25
// Kênh chẵn: D34, D33, D32, D35
// Thứ tự đọc từ trái sang phải: CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8
// → PIN: 14, 34, 27, 33, 26, 32, 25, 35

hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile long lastPos;
volatile unsigned char isCalib = 0;
int servoPwm;
volatile int calPID = 0;
volatile unsigned char sensor;
unsigned int sensorValue[8];
unsigned int sensorPID[8];
unsigned int black_value[8];
unsigned int white_value[8];
unsigned int compare_value[8];
int speed_run_forward;
int nut1 = 0;
int nut2 = 2800;
int nut3 = 3072;

volatile long posPID;
volatile int cnt = 0;
volatile int cnt1 = 0;
unsigned char pattern, start, trangThai;
int RememberLine = 0;
volatile float kp;
volatile int kd;
volatile int PIDfre = 0;

enum RouteAction : uint8_t {
  ACTION_STRAIGHT,
  ACTION_LEFT,
  ACTION_RIGHT,
  ACTION_UTURN,
  ACTION_STOP
};

enum MissionMode : uint8_t {
  MISSION_FOLLOW_LINE,
  MISSION_ENTER_NODE,
  MISSION_GO_STRAIGHT,
  MISSION_TURN_LEFT,
  MISSION_TURN_RIGHT,
  MISSION_TURN_BACK,
  MISSION_STOPPED
};

const unsigned long NODE_DEBOUNCE_MS = 250;
const unsigned long NODE_ENTRY_MS = 100;
const unsigned long STRAIGHT_CROSS_MS = 170;
const unsigned long TURN_MIN_MS = 140;
const int TURN_FORWARD_SPEED = 1350;
const int TURN_REVERSE_SPEED = 550;

MissionMode missionMode = MISSION_FOLLOW_LINE;
RouteAction currentAction = ACTION_STRAIGHT;
const RouteAction* activeRoute = nullptr;
int activeRouteLen = 0;
int routeIndex = 0;
unsigned long missionStampMs = 0;
unsigned long lastNodeMs = 0;

const float OBJECT_PRESENT_CM = 10.0f;
const unsigned long SCAN_PAUSE_MS = 300;

const char* BLE_DEVICE_NAME = "EC-Robot";
const char* BLE_SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
const char* BLE_CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
const int BLE_MANUAL_SPEED = 1800;

enum MainMode : uint8_t {
  MODE_MANUAL_TO_CP1,
  MODE_AUTO_TASK2,
  MODE_FINISHED
};

enum Task2State : uint8_t {
  TASK2_ROUTE_TO_MAZE,
  TASK2_SCAN_SLOT_A,
  TASK2_ROUTE_TO_SLOT_B,
  TASK2_SCAN_SLOT_B,
  TASK2_PICK_PLACEHOLDER,
  TASK2_ROUTE_TO_FINISH,
  TASK2_DROP_PLACEHOLDER,
  TASK2_FINISHED
};

// ===== Route block: keep all editable route arrays here =====
// Task 2 map assumptions from map_EC:
// - CP1 starts on the lower-left lane facing into the map.
// - Slot A is the black marker in the middle-right corridor.
// - Slot B is the white marker in the upper dead-end.
const RouteAction routeCp1ToMaze[]       = { ACTION_STRAIGHT, ACTION_LEFT, ACTION_RIGHT, ACTION_STRAIGHT };
const RouteAction routeMazeToSlotB[]     = { ACTION_UTURN, ACTION_LEFT, ACTION_STRAIGHT };
const RouteAction routeSlotAToFinish[]   = { ACTION_UTURN, ACTION_RIGHT, ACTION_RIGHT, ACTION_STRAIGHT, ACTION_STOP };
const RouteAction routeSlotBToFinish[]   = { ACTION_UTURN, ACTION_RIGHT, ACTION_STRAIGHT, ACTION_RIGHT, ACTION_STRAIGHT, ACTION_STOP };

MainMode mainMode = MODE_MANUAL_TO_CP1;
Task2State task2State = TASK2_ROUTE_TO_MAZE;
int task2SlotIndex = 0;
bool task2ObjectFound = false;
char bleCommand = 'S';

struct Task2Slot {
  const char* name;
};

const Task2Slot task2Slots[] = {
  { "maze_black_slot" },
  { "maze_white_slot" }
};

void readEeprom();
void timer_init();
void PID();
void read_sensor();
void ARDUINO_ISR_ATTR onTimer();
void updateLine();
void runforwardline(int tocdo);
void handleAndSpeed(int angle, int speed1);
void speed_run(int speedDC_left, int speedDC_right);
unsigned char sensorMask(unsigned char mask);
bool isNodeCandidate();
bool isCenteredOnLine();
uint8_t countActiveBits(unsigned char value);
float readUltrasonicCm();
void startMissionAction(RouteAction action);
void finishMissionAction();
void startRoute(const RouteAction* route, int len);
bool isRouteIdle();
void runRouteMission(int tocdo);
void initBleControl();
void runManualBleControl();
void resetMissionState();
void runHybridMission(int tocdo);
void runTask2Mission(int tocdo);
void pickTask2Placeholder();
void dropTask2Placeholder();

void setup() {
  // Motor pins
  pinMode(LDIR1, OUTPUT);
  pinMode(LDIR2, OUTPUT);
  pinMode(RDIR1, OUTPUT);
  pinMode(RDIR2, OUTPUT);

  // Button pins
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);

  // Dừng motor ban đầu
  digitalWrite(LDIR1, LOW);
  digitalWrite(LDIR2, LOW);
  digitalWrite(RDIR1, LOW);
  digitalWrite(RDIR2, LOW);

  // PWM cho L298N (tần số 2000Hz, độ phân giải 12-bit)
  ledcSetup(LPWM_CH, 2000, 12);
  ledcSetup(RPWM_CH, 2000, 12);
  ledcAttachPin(LPWM, LPWM_CH);
  ledcAttachPin(RPWM, RPWM_CH);

  speed_run(0, 0);
  trangThai = 9;
  pattern = 11;
  start = 0;
  activeRoute = nullptr;
  activeRouteLen = 0;
  routeIndex = 0;
  missionMode = MISSION_FOLLOW_LINE;
  currentAction = ACTION_STRAIGHT;
  missionStampMs = 0;
  lastNodeMs = 0;

  EEPROM.begin(EEPROM_SIZE);
  readEeprom();
  Serial.begin(115200);
  initBleControl();

  timer_init();
  isCalib = 0;
}

void timer_init() {
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);
}

class BleControlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    std::string value = characteristic->getValue();
    if (!value.empty()) {
      bleCommand = static_cast<char>(toupper(value[0]));
    }
  }
};

void initBleControl() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer* server = BLEDevice::createServer();
  BLEService* service = server->createService(BLE_SERVICE_UUID);

  BLECharacteristic* controlCharacteristic = service->createCharacteristic(
      BLE_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_NOTIFY);

  controlCharacteristic->addDescriptor(new BLE2902());
  controlCharacteristic->setCallbacks(new BleControlCallbacks());
  controlCharacteristic->setValue("S");

  service->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->start();
}

void resetMissionState() {
  activeRoute = nullptr;
  activeRouteLen = 0;
  routeIndex = 0;
  missionMode = MISSION_FOLLOW_LINE;
  currentAction = ACTION_STRAIGHT;
  missionStampMs = millis();
  lastNodeMs = 0;
  task2State = TASK2_ROUTE_TO_MAZE;
  task2SlotIndex = 0;
  task2ObjectFound = false;
}

void PID() {
  if (calPID == 1) {
    long sum = 0;
    long avg = 0;
    long i, iP, iD;
    long iRet;

    for (int j = 0; j < 8; j++) {
      avg = avg + (sensorPID[j]) * (j * 3000);
      sum = sum + sensorPID[j];
    }
    i = ((avg / sum) - 10500);
    posPID = i;
    kp = 1;
    kd = 15;
    iP = kp * i;
    iD = kd * (lastPos - i);
    iRet = (iP - iD);
    if (iRet < -12000) iRet = 0;
    servoPwm = iRet / 5;
    lastPos = i;
    calPID = 0;
  }
}

void read_sensor() {
  unsigned char temp = 0;
  PIDfre++;

  // Đọc 8 kênh theo thứ tự trái→phải: CH1→CH8
  // CH1=D14, CH2=D34, CH3=D27, CH4=D33
  // CH5=D26, CH6=D32, CH7=D25, CH8=D35
  sensorValue[0] = 4095 - analogRead(14);   // CH1 - ngoài cùng trái
  sensorValue[1] = 4095 - analogRead(34);   // CH2
  sensorValue[2] = 4095 - analogRead(27);   // CH3
  sensorValue[3] = 4095 - analogRead(33);   // CH4
  sensorValue[4] = 4095 - analogRead(26);   // CH5
  sensorValue[5] = 4095 - analogRead(32);   // CH6
  sensorValue[6] = 4095 - analogRead(25);   // CH7
  sensorValue[7] = 4095 - analogRead(35);   // CH8 - ngoài cùng phải

  for (int j = 0; j < 8; j++) {
    if (isCalib == 0) {
      if (sensorValue[j] < black_value[j]) sensorValue[j] = black_value[j];
      if (sensorValue[j] > white_value[j]) sensorValue[j] = white_value[j];
      sensorPID[j] = map(sensorValue[j], black_value[j], white_value[j], 1, 3000);
    }
    temp = temp << 1;
    if (sensorValue[j] > compare_value[j]) {
      temp |= 0x01;
    } else {
      temp &= 0xfe;
    }
    sensor = temp;
  }

  if (PIDfre == 2) {
    PIDfre = 0;
    calPID = 1;
  }
}

void ARDUINO_ISR_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  cnt++;
  cnt1++;
  read_sensor();
  portEXIT_CRITICAL_ISR(&timerMux);
}

void loop() {
  PID();

  while (start == 0) {
    PID();
    if (isCalib == 0) {
      if (digitalRead(BUTTON1) == 0) {
        for (int i = 0; i < 8; i++) black_value[i] = 5000;
        for (int i = 0; i < 8; i++) white_value[i] = 0;
        for (int i = 0; i < 8; i++) compare_value[i] = 5000;
        // beep bỏ vì không có buzzer
        while (digitalRead(BUTTON1) == 0) {}
        isCalib = 1;
        break;
      }
      if (digitalRead(BUTTON2) == 0) {
        start = 1;
        trangThai = 10;
        cnt = 0;
        cnt1 = 0;
        speed_run_forward = nut2;
        mainMode = MODE_MANUAL_TO_CP1;
        bleCommand = 'S';
        resetMissionState();
        break;
      }
      if (digitalRead(BUTTON3) == 0) {
        start = 1;
        trangThai = 10;
        cnt = 0;
        cnt1 = 0;
        speed_run_forward = nut3;
        mainMode = MODE_MANUAL_TO_CP1;
        bleCommand = 'S';
        resetMissionState();
        break;
      }
    } else {
      updateLine();
      if (digitalRead(BUTTON1) == 0) {
        for (int i = 0; i < 8; i++) {
          EEPROM.write(2 * i,       black_value[i] / 16);
          EEPROM.write((2 * i) + 1, black_value[i] % 16);
          delay(1);
        }
        for (int i = 0; i < 8; i++) {
          EEPROM.write(16 + (2 * i), white_value[i] / 16);
          EEPROM.write(17 + (2 * i), white_value[i] % 16);
          delay(1);
        }
        EEPROM.commit();
        while (digitalRead(BUTTON1) == 0) {}
        start = 1;
        isCalib = 0;
        speed_run_forward = 0;
      }
    }
  }

  switch (trangThai) {
    case 9:
      if (digitalRead(BUTTON1) == 0) {
        trangThai = 91;
        cnt = 0;
        speed_run_forward = nut1;
      }
      if (digitalRead(BUTTON2) == 0) {
        trangThai = 10;
        cnt = 0;
        cnt1 = 0;
        speed_run_forward = nut2;
        mainMode = MODE_MANUAL_TO_CP1;
        bleCommand = 'S';
        resetMissionState();
      }
      if (digitalRead(BUTTON3) == 0) {
        trangThai = 10;
        cnt = 0;
        cnt1 = 0;
        speed_run_forward = nut3;
        mainMode = MODE_MANUAL_TO_CP1;
        bleCommand = 'S';
        resetMissionState();
      }
      break;
    case 91:
      handleAndSpeed(servoPwm, 0);
      break;
    case 10:
      trangThai = 11;
      break;
    case 11:
      PID();
      runHybridMission(speed_run_forward);
      break;
    case 100:
      speed_run(0, 0);
      break;
    default:
      trangThai = 11;
      break;
  }
}

void runforwardline(int tocdo) {
  PID();
  switch (pattern) {
    case 10:
      pattern = 11;
      break;
    case 11:
      if (sensorMask(0x01) == 0x01) RememberLine = 1;
      else if (sensorMask(0x80) == 0x80) RememberLine = -1;

      if (sensor == 0b00000000) {
        if (cnt1 > 40000) {
          trangThai = 100;
          pattern = 100;
        } else if (RememberLine != 0) {
          if (RememberLine == 1) {
            speed_run(1200, -500);
            pattern = 12;
          } else if (RememberLine == -1) {
            speed_run(-500, 1200);
            pattern = 12;
          }
        } else {
          speed_run(0, 0);
        }
        break;
      } else {
        switch (sensor) {
          case 0b00011000:
          case 0b00111100:
          case 0b00011100:
          case 0b00001000:
          case 0b00001100:
          case 0b00011110:
          case 0b00000100:
          case 0b00001110:
          case 0b00000110:
          case 0b00001111:
          case 0b00011111:
          case 0b00111111:
          case 0b01111111:
          case 0b01111100:
          case 0b01111110:
          case 0b00111110:
          case 0b00000010:
          case 0b00000111:
          case 0b00000011:
          case 0b00000001:
          case 0b00000000:
          case 0b00010000:
          case 0b00111000:
          case 0b00110000:
          case 0b01111000:
          case 0b00100000:
          case 0b01110000:
          case 0b01100000:
          case 0b11110000:
          case 0b11111000:
          case 0b11111100:
          case 0b11111110:
          case 0b01000000:
          case 0b11100000:
          case 0b11000000:
          case 0b10000000:
            handleAndSpeed(servoPwm, tocdo);
            break;
          default:
            handleAndSpeed(servoPwm / 4, tocdo);
            break;
        }
      }
      break;
    case 12:
      if (RememberLine == 1) {
        speed_run(1200, -500);
        pattern = 21;
      } else if (RememberLine == -1) {
        speed_run(-500, 1200);
        pattern = 31;
      } else {
        pattern = 11;
      }
      break;
    case 21:
      speed_run(1200, -500);
      if (sensorMask(0x03) != 0) {
        speed_run(1200, -500);
        pattern = 22;
      }
      break;
    case 22:
      speed_run(1200, -500);
      if (sensorMask(0xfc) != 0) {
        pattern = 11;
        RememberLine = 0;
        cnt = 0;
      }
      break;
    case 31:
      speed_run(-500, 1200);
      if (sensorMask(0xc0) != 0) {
        speed_run(-500, 1200);
        pattern = 32;
      }
      break;
    case 32:
      speed_run(-500, 1200);
      if (sensorMask(0x3f) != 0) {
        pattern = 11;
        RememberLine = 0;
        cnt = 0;
      }
      break;
    case 100:
      speed_run(0, 0);
      break;
    default:
      break;
  }
}

uint8_t countActiveBits(unsigned char value) {
  uint8_t count = 0;
  while (value != 0) {
    count += (value & 0x01);
    value >>= 1;
  }
  return count;
}

bool isCenteredOnLine() {
  return (sensorMask(0x18) != 0) || (sensorMask(0x3c) == 0x3c);
}

bool isNodeCandidate() {
  const bool leftBranch = sensorMask(0xe0) != 0;
  const bool rightBranch = sensorMask(0x07) != 0;
  const bool centerLine = isCenteredOnLine();
  const uint8_t activeBits = countActiveBits(sensor);

  return centerLine && activeBits >= 4 && (leftBranch || rightBranch);
}

void startMissionAction(RouteAction action) {
  currentAction = action;
  missionStampMs = millis();
  lastNodeMs = missionStampMs;

  switch (action) {
    case ACTION_STRAIGHT:
      missionMode = MISSION_GO_STRAIGHT;
      break;
    case ACTION_LEFT:
      missionMode = MISSION_ENTER_NODE;
      break;
    case ACTION_RIGHT:
      missionMode = MISSION_ENTER_NODE;
      break;
    case ACTION_UTURN:
      missionMode = MISSION_ENTER_NODE;
      break;
    case ACTION_STOP:
      missionMode = MISSION_STOPPED;
      trangThai = 100;
      break;
    default:
      missionMode = MISSION_FOLLOW_LINE;
      break;
  }
}

void finishMissionAction() {
  missionMode = MISSION_FOLLOW_LINE;
  currentAction = ACTION_STRAIGHT;
  missionStampMs = millis();
}

void startRoute(const RouteAction* route, int len) {
  activeRoute = route;
  activeRouteLen = len;
  routeIndex = 0;
  missionMode = MISSION_FOLLOW_LINE;
  currentAction = ACTION_STRAIGHT;
  missionStampMs = millis();
  lastNodeMs = 0;
}

bool isRouteIdle() {
  return (activeRoute == nullptr) || (routeIndex >= activeRouteLen);
}

float readUltrasonicCm() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  unsigned long duration = pulseIn(US_ECHO, HIGH, 30000UL);
  if (duration == 0) return 999.0f;
  return duration * 0.0343f * 0.5f;
}

void runRouteMission(int tocdo) {
  const unsigned long now = millis();

  switch (missionMode) {
    case MISSION_FOLLOW_LINE:
      if ((activeRoute != nullptr) &&
          (routeIndex < activeRouteLen) &&
          (pattern == 11) &&
          (sensor != 0) &&
          (now - lastNodeMs > NODE_DEBOUNCE_MS) &&
          isNodeCandidate()) {
        startMissionAction(activeRoute[routeIndex]);
        routeIndex++;
        break;
      }
      runforwardline(tocdo);
      break;

    case MISSION_ENTER_NODE:
      speed_run(tocdo, tocdo);
      if (now - missionStampMs >= NODE_ENTRY_MS) {
        missionStampMs = now;
        if (currentAction == ACTION_LEFT) missionMode = MISSION_TURN_LEFT;
        else if (currentAction == ACTION_RIGHT) missionMode = MISSION_TURN_RIGHT;
        else missionMode = MISSION_TURN_BACK;
      }
      break;

    case MISSION_GO_STRAIGHT:
      speed_run(tocdo, tocdo);
      if ((now - missionStampMs >= STRAIGHT_CROSS_MS) && !isNodeCandidate()) {
        finishMissionAction();
      }
      break;

    case MISSION_TURN_LEFT:
      speed_run(-TURN_REVERSE_SPEED, TURN_FORWARD_SPEED);
      if ((now - missionStampMs >= TURN_MIN_MS) && isCenteredOnLine() && !isNodeCandidate()) {
        finishMissionAction();
      }
      break;

    case MISSION_TURN_RIGHT:
      speed_run(TURN_FORWARD_SPEED, -TURN_REVERSE_SPEED);
      if ((now - missionStampMs >= TURN_MIN_MS) && isCenteredOnLine() && !isNodeCandidate()) {
        finishMissionAction();
      }
      break;

    case MISSION_TURN_BACK:
      speed_run(TURN_FORWARD_SPEED, -TURN_FORWARD_SPEED);
      if ((now - missionStampMs >= (TURN_MIN_MS * 2)) && isCenteredOnLine()) {
        finishMissionAction();
      }
      break;

    case MISSION_STOPPED:
      speed_run(0, 0);
      break;

    default:
      missionMode = MISSION_FOLLOW_LINE;
      break;
  }

  if ((activeRoute != nullptr) &&
      (routeIndex >= activeRouteLen) &&
      (missionMode == MISSION_FOLLOW_LINE)) {
    activeRoute = nullptr;
    activeRouteLen = 0;
  }
}

void runManualBleControl() {
  switch (bleCommand) {
    case 'F':
      speed_run(BLE_MANUAL_SPEED, BLE_MANUAL_SPEED);
      break;
    case 'B':
      speed_run(-BLE_MANUAL_SPEED, -BLE_MANUAL_SPEED);
      break;
    case 'L':
      speed_run(-BLE_MANUAL_SPEED, BLE_MANUAL_SPEED);
      break;
    case 'R':
      speed_run(BLE_MANUAL_SPEED, -BLE_MANUAL_SPEED);
      break;
    case 'S':
    default:
      speed_run(0, 0);
      break;
  }
}

void pickTask2Placeholder() {
  Serial.println("task2 pick placeholder");
}

void dropTask2Placeholder() {
  Serial.println("task2 drop placeholder at finish");
}

void runTask2Mission(int tocdo) {
  const float distanceCm = readUltrasonicCm();

  if ((task2State == TASK2_ROUTE_TO_MAZE) && (activeRoute == nullptr) && (routeIndex == 0)) {
    startRoute(routeCp1ToMaze, sizeof(routeCp1ToMaze) / sizeof(routeCp1ToMaze[0]));
  }

  if ((task2State == TASK2_ROUTE_TO_SLOT_B) && (activeRoute == nullptr) && (routeIndex == 0)) {
    startRoute(routeMazeToSlotB, sizeof(routeMazeToSlotB) / sizeof(routeMazeToSlotB[0]));
  }

  if ((task2State == TASK2_ROUTE_TO_FINISH) && (activeRoute == nullptr) && (routeIndex == 0)) {
    if (task2SlotIndex == 0) {
      startRoute(routeSlotAToFinish, sizeof(routeSlotAToFinish) / sizeof(routeSlotAToFinish[0]));
    } else {
      startRoute(routeSlotBToFinish, sizeof(routeSlotBToFinish) / sizeof(routeSlotBToFinish[0]));
    }
  }

  switch (task2State) {
    case TASK2_ROUTE_TO_MAZE:
      runRouteMission(tocdo);
      if (isRouteIdle()) {
        missionStampMs = millis();
        task2State = TASK2_SCAN_SLOT_A;
      }
      break;

    case TASK2_SCAN_SLOT_A:
      speed_run(0, 0);
      if (millis() - missionStampMs >= SCAN_PAUSE_MS) {
        Serial.print(task2Slots[0].name);
        Serial.print(" distance(cm): ");
        Serial.println(distanceCm);
        if (distanceCm <= OBJECT_PRESENT_CM) {
          task2SlotIndex = 0;
          task2ObjectFound = true;
          task2State = TASK2_PICK_PLACEHOLDER;
        } else {
          routeIndex = 0;
          task2State = TASK2_ROUTE_TO_SLOT_B;
        }
      }
      break;

    case TASK2_ROUTE_TO_SLOT_B:
      runRouteMission(tocdo);
      if (isRouteIdle()) {
        missionStampMs = millis();
        task2State = TASK2_SCAN_SLOT_B;
      }
      break;

    case TASK2_SCAN_SLOT_B:
      speed_run(0, 0);
      if (millis() - missionStampMs >= SCAN_PAUSE_MS) {
        Serial.print(task2Slots[1].name);
        Serial.print(" distance(cm): ");
        Serial.println(distanceCm);
        task2SlotIndex = 1;
        task2ObjectFound = (distanceCm <= OBJECT_PRESENT_CM);
        task2State = TASK2_PICK_PLACEHOLDER;
      }
      break;

    case TASK2_PICK_PLACEHOLDER:
      if (task2ObjectFound) {
        Serial.print("task2 object selected at ");
        Serial.println(task2Slots[task2SlotIndex].name);
        pickTask2Placeholder();
      } else {
        Serial.println("task2 object not found in both slots");
      }
      routeIndex = 0;
      task2State = TASK2_ROUTE_TO_FINISH;
      break;

    case TASK2_ROUTE_TO_FINISH:
      runRouteMission(tocdo);
      if (isRouteIdle()) {
        task2State = TASK2_DROP_PLACEHOLDER;
      }
      break;

    case TASK2_DROP_PLACEHOLDER:
      if (task2ObjectFound) {
        dropTask2Placeholder();
      }
      task2State = TASK2_FINISHED;
      break;

    case TASK2_FINISHED:
      speed_run(0, 0);
      mainMode = MODE_FINISHED;
      trangThai = 100;
      break;

    default:
      task2State = TASK2_ROUTE_TO_MAZE;
      break;
  }
}

void runHybridMission(int tocdo) {
  switch (mainMode) {
    case MODE_MANUAL_TO_CP1:
      runManualBleControl();
      if (bleCommand == 'A') {
        bleCommand = 'S';
        speed_run(0, 0);
        mainMode = MODE_AUTO_TASK2;
        resetMissionState();
      }
      break;

    case MODE_AUTO_TASK2:
      runTask2Mission(tocdo);
      break;

    case MODE_FINISHED:
      speed_run(0, 0);
      break;

    default:
      mainMode = MODE_MANUAL_TO_CP1;
      break;
  }
}

void updateLine() {
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorValue[i]);
    Serial.print("  ");
    delay(1);
    if (black_value[i] == 0) black_value[i] = 5000;
    if (sensorValue[i] < black_value[i]) black_value[i] = sensorValue[i];
    if (sensorValue[i] > white_value[i]) white_value[i] = sensorValue[i];
    compare_value[i] = (black_value[i] + white_value[i]) / 2;
  }
  Serial.println();
}

void readEeprom() {
  for (int i = 0; i < 8; i++) {
    black_value[i] = EEPROM.read(i * 2) * 16 + EEPROM.read((i * 2) + 1);
  }
  for (int i = 0; i < 8; i++) {
    white_value[i] = EEPROM.read(16 + (2 * i)) * 16 + EEPROM.read(17 + (2 * i));
  }
  for (int i = 0; i < 8; i++) {
    compare_value[i] = (black_value[i] + white_value[i]) / 2;
  }
}

void handleAndSpeed(int angle, int speed1) {
  int speedLeft, speedRight;
  if ((speed1 + angle) > 4095) speed1 = 4095 - angle;
  if ((speed1 - angle) > 4095) speed1 = 4095 + angle;
  speedLeft  = speed1 + angle;
  speedRight = speed1 - angle;
  speed_run(speedLeft, speedRight);
}

// =====================================================
// Điều khiển motor qua L298N (có 2 chân DIR mỗi bên)
// speedDC > 0: tiến, < 0: lùi, = 0: dừng
// =====================================================
void speed_run(int speedDC_left, int speedDC_right) {
  // Giới hạn tốc độ
  speedDC_left  = constrain(speedDC_left,  -4095, 4095);
  speedDC_right = constrain(speedDC_right, -4095, 4095);

  // Motor TRÁI (IN1/IN2 + ENA)
  if (speedDC_left > 0) {
    digitalWrite(LDIR1, HIGH);
    digitalWrite(LDIR2, LOW);
    ledcWrite(LPWM_CH, speedDC_left);
  } else if (speedDC_left < 0) {
    digitalWrite(LDIR1, LOW);
    digitalWrite(LDIR2, HIGH);
    ledcWrite(LPWM_CH, -speedDC_left);
  } else {
    digitalWrite(LDIR1, LOW);
    digitalWrite(LDIR2, LOW);
    ledcWrite(LPWM_CH, 0);
  }

  // Motor PHẢI (IN3/IN4 + ENB)
  if (speedDC_right > 0) {
    digitalWrite(RDIR1, HIGH);
    digitalWrite(RDIR2, LOW);
    ledcWrite(RPWM_CH, speedDC_right);
  } else if (speedDC_right < 0) {
    digitalWrite(RDIR1, LOW);
    digitalWrite(RDIR2, HIGH);
    ledcWrite(RPWM_CH, -speedDC_right);
  } else {
    digitalWrite(RDIR1, LOW);
    digitalWrite(RDIR2, LOW);
    ledcWrite(RPWM_CH, 0);
  }
}

unsigned char sensorMask(unsigned char mask) {
  return (sensor & mask);
}

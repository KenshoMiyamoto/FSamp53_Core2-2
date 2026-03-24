#include <Arduino.h>
#include "M5Unified.h"

#define PIN_INA 35
#define PIN_INB 36
#define PIN_INC 34

#define COLOR_A TFT_RED
#define COLOR_B TFT_GREEN
#define COLOR_C TFT_YELLOW

// ===== 設定 =====
const uint32_t SAMPLE_RATE_HZ = 600;                    // 目標サンプリング周波数
const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ;
const int DRAW_EVERY_N_SAMPLES = 3;                    // 3 -> 200Hz描画
const int SERIAL_EVERY_N_SAMPLES = 3;                  // 3 -> 200Hz送信

// マトリクス
const float MATRIX[3][3] = {
    {0.01178285, -0.3731538 , 0.3773194},
    {0.42417336, -0.18295395, -0.22511071},
    {-0.62513834, -0.37187675, -0.49920508}
};

const int BUFFER_SIZE = 320;
int bufferA[BUFFER_SIZE];
int bufferB[BUFFER_SIZE];
int bufferC[BUFFER_SIZE];

// simple moving average LPF
const int LPF_SIZE = 10;
int lpfBufA[LPF_SIZE];
int lpfBufB[LPF_SIZE];
int lpfBufC[LPF_SIZE];

float accelLpfBufX[LPF_SIZE];
float accelLpfBufY[LPF_SIZE];
float accelLpfBufZ[LPF_SIZE];

float gyroLpfBufX[LPF_SIZE];
float gyroLpfBufY[LPF_SIZE];
float gyroLpfBufZ[LPF_SIZE];

float forceLpfBufX[LPF_SIZE];
float forceLpfBufY[LPF_SIZE];
float forceLpfBufZ[LPF_SIZE];

int lpfIndex = 0;
bool lpfFull = false;

long lpfSumA = 0;
long lpfSumB = 0;
long lpfSumC = 0;

float accelLpfSumX = 0;
float accelLpfSumY = 0;
float accelLpfSumZ = 0;

float gyroLpfSumX = 0;
float gyroLpfSumY = 0;
float gyroLpfSumZ = 0;

float forceLpfSumX = 0;
float forceLpfSumY = 0;
float forceLpfSumZ = 0;

int displayWidth;
int displayHeight;

// 描画用の現在X位置
const int LEFT_MARGIN = 25;
int plotX = LEFT_MARGIN;

// prev
int prevYA = -1;
int prevYB = -1;
int prevYC = -1;

int prevYA_accel = -1;
int prevYB_accel = -1;
int prevYC_accel = -1;

int prevYA_gyro = -1;
int prevYB_gyro = -1;
int prevYC_gyro = -1;

int prevYA_force = -1;
int prevYB_force = -1;
int prevYC_force = -1;

// サンプリングレート表示用
float currentSampleRate = 0.0f;
volatile uint32_t isrCount = 0;
unsigned long rateMeasureMillis = 0;
uint32_t lastIsrCount = 0;

// オフセット関連
bool offsetMode = false;
float offsetA = 0.0;
float offsetB = 0.0;
float offsetC = 0.0;
bool collectingOffsets = false;
int sampleCount = 0;
long sumA = 0;
long sumB = 0;
long sumC = 0;

bool displayOn = true;

// モード表示：0: 生の電圧, 1: オフセット済み電圧, 2: IMU, 3: 力
int displayMode = 0;

// ロック状態
bool displayModeLocked = false;

// Bボタン長押し1回判定用
bool btnBLongPressHandled = false;

// 生データ
int rawA = 0;
int rawB = 0;
int rawC = 0;

float accelX = 0.0f;
float accelY = 0.0f;
float accelZ = 0.0f;

float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;

// LPF後の値
float valA = 0.0f;
float valB = 0.0f;
float valC = 0.0f;

float accelValX = 0.0f;
float accelValY = 0.0f;
float accelValZ = 0.0f;

float gyroValX = 0.0f;
float gyroValY = 0.0f;
float gyroValZ = 0.0f;

// 力
float forceX = 0.0f;
float forceY = 0.0f;
float forceZ = 0.0f;

float forceValX = 0.0f;
float forceValY = 0.0f;
float forceValZ = 0.0f;

// タイマー割り込み
hw_timer_t *timer = nullptr;
volatile uint32_t pendingSamples = 0;

// 間引き用
uint32_t drawCounter = 0;
uint32_t serialCounter = 0;

// ===== タイマーISR =====
void IRAM_ATTR onSampleTimer() {
  pendingSamples++;
  isrCount++;
}

// ===== 共通関数 =====
void drawModeLabel();
void drawLeftScale();
void drawIMUSeparator();

void startOffsetCollection() {
  collectingOffsets = true;
  sampleCount = 0;
  sumA = 0;
  sumB = 0;
  sumC = 0;
  offsetMode = true;
}

void resetPrevPoints() {
  prevYA = -1;
  prevYB = -1;
  prevYC = -1;

  prevYA_accel = -1;
  prevYB_accel = -1;
  prevYC_accel = -1;

  prevYA_gyro = -1;
  prevYB_gyro = -1;
  prevYC_gyro = -1;

  prevYA_force = -1;
  prevYB_force = -1;
  prevYC_force = -1;
}

void refreshModeScreen() {
  plotX = LEFT_MARGIN;
  resetPrevPoints();
  M5.Display.fillScreen(TFT_BLACK);
  drawModeLabel();
  drawLeftScale();
  drawIMUSeparator();
}

void handleSerialCommand() {
  while (Serial.available() > 0) {
    char cmd = (char)Serial.read();

    // 改行などは無視
    if (cmd == '\n' || cmd == '\r' || cmd == ' ') continue;

    if (cmd == 'v') {
      displayMode = 0;
      refreshModeScreen();
      Serial.println("# command: Raw Volt");
    } else if (cmd == 'o') {
      displayMode = 1;
      refreshModeScreen();
      Serial.println("# command: Off Volt");
    } else if (cmd == 'I') {
      displayMode = 2;
      refreshModeScreen();
      Serial.println("# command: IMU");
    } else if (cmd == 'f') {
      displayMode = 3;
      refreshModeScreen();
      Serial.println("# command: Force");
    } else if (cmd == 'O') {
      startOffsetCollection();
      Serial.println("# command: Offset start");
    }
  }
}

// 画面上部にモード・凡例・バッテリー残量を表示
void drawModeLabel() {
  M5.Display.fillRect(0, 0, displayWidth, 20, TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(10, 5);

  const char* modeStr[] = {"Raw Volt", "Off Volt", "IMU", "Force"};
  M5.Display.printf("Mode: %s", modeStr[displayMode]);

  // ロック状態表示（中央）
  if (displayModeLocked) {
    M5.Display.setCursor(displayWidth / 2 - 15, 5);
    M5.Display.printf("LOCKED");
  }

  // バッテリー残量表示
  int batteryPercent = M5.Power.getBatteryLevel();
  M5.Display.setCursor(displayWidth - 42, 5);
  M5.Display.printf("%d%%", batteryPercent);

  // 凡例は少し左へ
  int startX = displayWidth - 130;
  int y = 10;
  int radius = 4;

  const char* labelA = (displayMode == 2 || displayMode == 3) ? "X" : "A";
  const char* labelB = (displayMode == 2 || displayMode == 3) ? "Y" : "B";
  const char* labelC = (displayMode == 2 || displayMode == 3) ? "Z" : "C";

  M5.Display.drawCircle(startX, y, radius, COLOR_A);
  M5.Display.fillCircle(startX, y, radius, COLOR_A);
  M5.Display.setCursor(startX + 10, 5);
  M5.Display.printf("%s", labelA);

  startX += 30;
  M5.Display.drawCircle(startX, y, radius, COLOR_B);
  M5.Display.fillCircle(startX, y, radius, COLOR_B);
  M5.Display.setCursor(startX + 10, 5);
  M5.Display.printf("%s", labelB);

  startX += 30;
  M5.Display.drawCircle(startX, y, radius, COLOR_C);
  M5.Display.fillCircle(startX, y, radius, COLOR_C);
  M5.Display.setCursor(startX + 10, 5);
  M5.Display.printf("%s", labelC);
}

void drawLeftScale() {
  M5.Display.fillRect(0, 20, LEFT_MARGIN, displayHeight - 20, TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(1);

  if (displayMode == 0) {
    // Raw Volt: 0 ～ 3.3 V
    float voltages[] = {0, 0.8, 1.65, 2.5, 3.3};
    for (int i = 0; i < 5; i++) {
      float voltage = voltages[i];
      int y = map((int)(voltage * 1000), 0, 3300, displayHeight - 1, 20);
      M5.Display.drawLine(LEFT_MARGIN - 3, y, LEFT_MARGIN - 1, y, TFT_WHITE);
      M5.Display.setCursor(1, y - 3);
      M5.Display.printf("%.1f", voltage);
    }
    M5.Display.setCursor(2, 22);
    M5.Display.printf("V");

  } else if (displayMode == 1) {
    // Off Volt: -2 ～ +2 V
    float voltages[] = {-2.0, -1.0, 0.0, 1.0, 2.0};
    for (int i = 0; i < 5; i++) {
      float voltage = voltages[i];
      int y = map((int)(voltage * 1000), -2000, 2000, displayHeight - 1, 20);
      M5.Display.drawLine(LEFT_MARGIN - 3, y, LEFT_MARGIN - 1, y, TFT_WHITE);
      M5.Display.setCursor(1, y - 3);
      M5.Display.printf("%+.1f", voltage);
    }
    M5.Display.setCursor(2, 22);
    M5.Display.printf("V");

  } else if (displayMode == 2) {
    int midY = (displayHeight + 20) / 2;

    float accelMarks[] = {-1.0, -0.5, 0.0, 0.5, 1.0};
    for (int i = 0; i < 5; i++) {
      float accel = accelMarks[i];
      int y = constrain(map((long)(accel * 1000), -1000, 1000, midY - 1, 20), 20, midY - 1);
      M5.Display.drawLine(LEFT_MARGIN - 3, y, LEFT_MARGIN - 1, y, TFT_WHITE);
      M5.Display.setCursor(1, y - 3);
      M5.Display.printf("%+.1f", accel);
    }

    M5.Display.setCursor(2, 22);
    M5.Display.print("m");
    M5.Display.setCursor(2, 30);
    M5.Display.print("s2");

    float gyroMarks[] = {-250, -125, 0, 125, 250};
    for (int i = 0; i < 5; i++) {
      float gyro = gyroMarks[i];
      int y = constrain(map((long)gyro, -250, 250, displayHeight - 1, midY), midY, displayHeight - 1);
      M5.Display.drawLine(LEFT_MARGIN - 3, y, LEFT_MARGIN - 1, y, TFT_WHITE);
      M5.Display.setCursor(1, y - 3);
      M5.Display.printf("%+d", (int)gyro);
    }

    M5.Display.setCursor(2, midY + 2);
    M5.Display.print("d");
    M5.Display.setCursor(2, midY + 10);
    M5.Display.print("ps");

  } else if (displayMode == 3) {
    float forceMarks[] = {-2.0, -1.0, 0.0, 1.0, 2.0};
    for (int i = 0; i < 5; i++) {
      float force = forceMarks[i];
      int y = map((int)(force * 1000), -2000, 2000, displayHeight - 1, 20);
      M5.Display.drawLine(LEFT_MARGIN - 3, y, LEFT_MARGIN - 1, y, TFT_WHITE);
      M5.Display.setCursor(1, y - 3);
      M5.Display.printf("%+.1f", force);
    }
    M5.Display.setCursor(2, 22);
    M5.Display.printf("N");
  }
}

void drawIMUSeparator() {
  if (displayMode == 2) {
    int midY = (displayHeight + 20) / 2;
    M5.Display.drawLine(LEFT_MARGIN, midY, displayWidth - 1, midY, TFT_WHITE);
  }
}

void clearPlotArea() {
  M5.Display.fillRect(LEFT_MARGIN, 20, displayWidth - LEFT_MARGIN, displayHeight - 20, TFT_BLACK);
  drawModeLabel();
  drawLeftScale();
  drawIMUSeparator();
}

// ===== 1サンプル分の取得・計算 =====
void processOneSample() {
  rawA = analogReadMilliVolts(PIN_INA);
  rawB = analogReadMilliVolts(PIN_INB);
  rawC = analogReadMilliVolts(PIN_INC);

  M5.Imu.getAccel(&accelX, &accelY, &accelZ);
  M5.Imu.getGyro(&gyroX, &gyroY, &gyroZ);

  // LPF voltage
  lpfSumA += rawA;
  lpfSumB += rawB;
  lpfSumC += rawC;

  if (lpfFull) {
    lpfSumA -= lpfBufA[lpfIndex];
    lpfSumB -= lpfBufB[lpfIndex];
    lpfSumC -= lpfBufC[lpfIndex];
  }

  lpfBufA[lpfIndex] = rawA;
  lpfBufB[lpfIndex] = rawB;
  lpfBufC[lpfIndex] = rawC;

  // LPF accel
  accelLpfSumX += accelX;
  accelLpfSumY += accelY;
  accelLpfSumZ += accelZ;

  if (lpfFull) {
    accelLpfSumX -= accelLpfBufX[lpfIndex];
    accelLpfSumY -= accelLpfBufY[lpfIndex];
    accelLpfSumZ -= accelLpfBufZ[lpfIndex];
  }

  accelLpfBufX[lpfIndex] = accelX;
  accelLpfBufY[lpfIndex] = accelY;
  accelLpfBufZ[lpfIndex] = accelZ;

  // LPF gyro
  gyroLpfSumX += gyroX;
  gyroLpfSumY += gyroY;
  gyroLpfSumZ += gyroZ;

  if (lpfFull) {
    gyroLpfSumX -= gyroLpfBufX[lpfIndex];
    gyroLpfSumY -= gyroLpfBufY[lpfIndex];
    gyroLpfSumZ -= gyroLpfBufZ[lpfIndex];
  }

  gyroLpfBufX[lpfIndex] = gyroX;
  gyroLpfBufY[lpfIndex] = gyroY;
  gyroLpfBufZ[lpfIndex] = gyroZ;

  int denom = lpfFull ? LPF_SIZE : (lpfIndex + 1);
  if (denom <= 0) denom = 1;

  valA = lpfSumA / (float)denom;
  valB = lpfSumB / (float)denom;
  valC = lpfSumC / (float)denom;

  accelValX = accelLpfSumX / (float)denom;
  accelValY = accelLpfSumY / (float)denom;
  accelValZ = accelLpfSumZ / (float)denom;

  gyroValX = gyroLpfSumX / (float)denom;
  gyroValY = gyroLpfSumY / (float)denom;
  gyroValZ = gyroLpfSumZ / (float)denom;

  if (collectingOffsets) {
    sumA += rawA;
    sumB += rawB;
    sumC += rawC;
    sampleCount++;

    if (sampleCount >= 10) {
      offsetA = sumA / 10.0f;
      offsetB = sumB / 10.0f;
      offsetC = sumC / 10.0f;
      collectingOffsets = false;
    }
  }

  float volA_off = (valA - offsetA) / 1000.0f;
  float volB_off = (valB - offsetB) / 1000.0f;
  float volC_off = (valC - offsetC) / 1000.0f;

  forceX = MATRIX[0][0] * volA_off + MATRIX[0][1] * volB_off + MATRIX[0][2] * volC_off;
  forceY = MATRIX[1][0] * volA_off + MATRIX[1][1] * volB_off + MATRIX[1][2] * volC_off;
  forceZ = MATRIX[2][0] * volA_off + MATRIX[2][1] * volB_off + MATRIX[2][2] * volC_off;

  // LPF force
  forceLpfSumX += forceX;
  forceLpfSumY += forceY;
  forceLpfSumZ += forceZ;

  if (lpfFull) {
    forceLpfSumX -= forceLpfBufX[lpfIndex];
    forceLpfSumY -= forceLpfBufY[lpfIndex];
    forceLpfSumZ -= forceLpfBufZ[lpfIndex];
  }

  forceLpfBufX[lpfIndex] = forceX;
  forceLpfBufY[lpfIndex] = forceY;
  forceLpfBufZ[lpfIndex] = forceZ;

  forceValX = forceLpfSumX / (float)denom;
  forceValY = forceLpfSumY / (float)denom;
  forceValZ = forceLpfSumZ / (float)denom;

  lpfIndex++;
  if (lpfIndex >= LPF_SIZE) {
    lpfIndex = 0;
    lpfFull = true;
  }
}

void updateDisplayedSampleRate() {
  unsigned long nowMillis = millis();
  if (nowMillis - rateMeasureMillis >= 1000) {
    noInterrupts();
    uint32_t countNow = isrCount;
    interrupts();

    currentSampleRate = (float)(countNow - lastIsrCount);

    lastIsrCount = countNow;
    rateMeasureMillis = nowMillis;
  }
}

void outputSerial() {
  if (displayMode == 0) {
    Serial.print(rawA / 1000.0f);
    Serial.print(",");
    Serial.print(rawB / 1000.0f);
    Serial.print(",");
    Serial.print(rawC / 1000.0f);
    Serial.print(",");
    Serial.println(currentSampleRate);

  } else if (displayMode == 1) {
    float outA = (valA - offsetA) / 1000.0f;
    float outB = (valB - offsetB) / 1000.0f;
    float outC = (valC - offsetC) / 1000.0f;

    Serial.print(outA);
    Serial.print(",");
    Serial.print(outB);
    Serial.print(",");
    Serial.print(outC);
    Serial.print(",");
    Serial.println(currentSampleRate);

  } else if (displayMode == 2) {
    Serial.print(accelValX);
    Serial.print(",");
    Serial.print(accelValY);
    Serial.print(",");
    Serial.print(accelValZ);
    Serial.print(",");
    Serial.print(gyroValX);
    Serial.print(",");
    Serial.print(gyroValY);
    Serial.print(",");
    Serial.print(gyroValZ);
    Serial.print(",");
    Serial.println(currentSampleRate);

  } else if (displayMode == 3) {
    Serial.print(forceValX);
    Serial.print(",");
    Serial.print(forceValY);
    Serial.print(",");
    Serial.print(forceValZ);
    Serial.print(",");
    Serial.println(currentSampleRate);
  }
}

void drawGraph() {
  float displayValA, displayValB, displayValC;
  int yA_accel, yB_accel, yC_accel;
  int yA_vel, yB_vel, yC_vel;

  if (displayMode == 0) {
    displayValA = rawA;
    displayValB = rawB;
    displayValC = rawC;
  } else if (displayMode == 1) {
    displayValA = offsetMode ? (valA - offsetA) : valA;
    displayValB = offsetMode ? (valB - offsetB) : valB;
    displayValC = offsetMode ? (valC - offsetC) : valC;
  } else if (displayMode == 2) {
    displayValA = accelValX;
    displayValB = accelValY;
    displayValC = accelValZ;
  } else {
    displayValA = forceValX;
    displayValB = forceValY;
    displayValC = forceValZ;
  }

  if (displayMode == 0) {
    yA_accel = constrain(map((long)displayValA, 0, 3300, displayHeight - 1, 20), 20, displayHeight - 1);
    yB_accel = constrain(map((long)displayValB, 0, 3300, displayHeight - 1, 20), 20, displayHeight - 1);
    yC_accel = constrain(map((long)displayValC, 0, 3300, displayHeight - 1, 20), 20, displayHeight - 1);
    yA_vel = yA_accel;
    yB_vel = yB_accel;
    yC_vel = yC_accel;

  } else if (displayMode == 1) {
    yA_accel = constrain(map((long)displayValA, -2000, 2000, displayHeight - 1, 20), 20, displayHeight - 1);
    yB_accel = constrain(map((long)displayValB, -2000, 2000, displayHeight - 1, 20), 20, displayHeight - 1);
    yC_accel = constrain(map((long)displayValC, -2000, 2000, displayHeight - 1, 20), 20, displayHeight - 1);
    yA_vel = yA_accel;
    yB_vel = yB_accel;
    yC_vel = yC_accel;

  } else if (displayMode == 2) {
    int midY = (displayHeight + 20) / 2;

    yA_accel = constrain(map((long)(displayValA * 1000), -1000, 1000, midY - 1, 20), 20, midY - 1);
    yB_accel = constrain(map((long)(displayValB * 1000), -1000, 1000, midY - 1, 20), 20, midY - 1);
    yC_accel = constrain(map((long)(displayValC * 1000), -1000, 1000, midY - 1, 20), 20, midY - 1);

    yA_vel = constrain(map((long)gyroValX, -250, 250, displayHeight - 1, midY), midY, displayHeight - 1);
    yB_vel = constrain(map((long)gyroValY, -250, 250, displayHeight - 1, midY), midY, displayHeight - 1);
    yC_vel = constrain(map((long)gyroValZ, -250, 250, displayHeight - 1, midY), midY, displayHeight - 1);

  } else {
    yA_accel = constrain(map((long)(displayValA * 1000), -2000, 2000, displayHeight - 1, 20), 20, displayHeight - 1);
    yB_accel = constrain(map((long)(displayValB * 1000), -2000, 2000, displayHeight - 1, 20), 20, displayHeight - 1);
    yC_accel = constrain(map((long)(displayValC * 1000), -2000, 2000, displayHeight - 1, 20), 20, displayHeight - 1);
    yA_vel = yA_accel;
    yB_vel = yB_accel;
    yC_vel = yC_accel;
  }

  if (plotX >= displayWidth) {
    plotX = LEFT_MARGIN;
    resetPrevPoints();
    clearPlotArea();
  }

  if (displayOn) {
    int x = plotX;

    if (displayMode == 2) {
      if (prevYA_accel != -1) M5.Display.drawLine(x - 1, prevYA_accel, x, yA_accel, COLOR_A);
      else M5.Display.drawPixel(x, yA_accel, COLOR_A);

      if (prevYB_accel != -1) M5.Display.drawLine(x - 1, prevYB_accel, x, yB_accel, COLOR_B);
      else M5.Display.drawPixel(x, yB_accel, COLOR_B);

      if (prevYC_accel != -1) M5.Display.drawLine(x - 1, prevYC_accel, x, yC_accel, COLOR_C);
      else M5.Display.drawPixel(x, yC_accel, COLOR_C);

      if (prevYA_gyro != -1) M5.Display.drawLine(x - 1, prevYA_gyro, x, yA_vel, COLOR_A);
      else M5.Display.drawPixel(x, yA_vel, COLOR_A);

      if (prevYB_gyro != -1) M5.Display.drawLine(x - 1, prevYB_gyro, x, yB_vel, COLOR_B);
      else M5.Display.drawPixel(x, yB_vel, COLOR_B);

      if (prevYC_gyro != -1) M5.Display.drawLine(x - 1, prevYC_gyro, x, yC_vel, COLOR_C);
      else M5.Display.drawPixel(x, yC_vel, COLOR_C);

    } else if (displayMode == 3) {
      if (prevYA_force != -1) M5.Display.drawLine(x - 1, prevYA_force, x, yA_accel, COLOR_A);
      else M5.Display.drawPixel(x, yA_accel, COLOR_A);

      if (prevYB_force != -1) M5.Display.drawLine(x - 1, prevYB_force, x, yB_accel, COLOR_B);
      else M5.Display.drawPixel(x, yB_accel, COLOR_B);

      if (prevYC_force != -1) M5.Display.drawLine(x - 1, prevYC_force, x, yC_accel, COLOR_C);
      else M5.Display.drawPixel(x, yC_accel, COLOR_C);

    } else {
      if (prevYA != -1) M5.Display.drawLine(x - 1, prevYA, x, yA_accel, COLOR_A);
      else M5.Display.drawPixel(x, yA_accel, COLOR_A);

      if (prevYB != -1) M5.Display.drawLine(x - 1, prevYB, x, yB_accel, COLOR_B);
      else M5.Display.drawPixel(x, yB_accel, COLOR_B);

      if (prevYC != -1) M5.Display.drawLine(x - 1, prevYC, x, yC_accel, COLOR_C);
      else M5.Display.drawPixel(x, yC_accel, COLOR_C);
    }
  }

  if (displayMode == 2) {
    prevYA_accel = yA_accel;
    prevYB_accel = yB_accel;
    prevYC_accel = yC_accel;

    prevYA_gyro = yA_vel;
    prevYB_gyro = yB_vel;
    prevYC_gyro = yC_vel;

  } else if (displayMode == 3) {
    prevYA_force = yA_accel;
    prevYB_force = yB_accel;
    prevYC_force = yC_accel;

  } else {
    prevYA = yA_accel;
    prevYB = yB_accel;
    prevYC = yC_accel;
  }

  plotX++;
}

void setup() {
  M5.begin();
  M5.Imu.begin();

  displayWidth = M5.Display.width();
  displayHeight = M5.Display.height();
  plotX = LEFT_MARGIN;

  M5.Display.fillScreen(TFT_BLACK);

  Serial.begin(115200);
  delay(1000);
  Serial.println("=== M5Stack Serial Start ===");
  Serial.println("# commands: v=RawVolt, o=OffVolt, I=IMU, f=Force, O=Offset");

  for (int i = 0; i < LPF_SIZE; i++) {
    lpfBufA[i] = 0;
    lpfBufB[i] = 0;
    lpfBufC[i] = 0;

    accelLpfBufX[i] = 0;
    accelLpfBufY[i] = 0;
    accelLpfBufZ[i] = 0;

    gyroLpfBufX[i] = 0;
    gyroLpfBufY[i] = 0;
    gyroLpfBufZ[i] = 0;

    forceLpfBufX[i] = 0;
    forceLpfBufY[i] = 0;
    forceLpfBufZ[i] = 0;
  }

  refreshModeScreen();

  rateMeasureMillis = millis();
  currentSampleRate = 0.0f;

  // 起動時に1回だけ自動オフセット取得
  startOffsetCollection();

  // ===== 600Hz タイマー開始 =====
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onSampleTimer);
  timerAlarm(timer, SAMPLE_PERIOD_US, true, 0);
#else
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onSampleTimer, true);
  timerAlarmWrite(timer, SAMPLE_PERIOD_US, true);
  timerAlarmEnable(timer);
#endif
}

void loop() {
  M5.update();

  // ボタンA: オフセット再取得（ロックされていない場合のみ）
  if (M5.BtnA.wasPressed() && !displayModeLocked) {
    startOffsetCollection();
  }

  // ボタンB: 3秒長押しでロック/解除（押している間に1回だけ）
  if (M5.BtnB.isPressed()) {
    if (M5.BtnB.pressedFor(3000) && !btnBLongPressHandled) {
      displayModeLocked = !displayModeLocked;
      btnBLongPressHandled = true;
      refreshModeScreen();
    }
  } else {
    // 指を離したら次の長押しを受け付ける
    btnBLongPressHandled = false;
  }

  // ボタンC: モード切り替え（ロックされていない場合のみ）
  if (M5.BtnC.wasPressed() && !displayModeLocked) {
    displayMode = (displayMode + 1) % 4;
    refreshModeScreen();
  }

  // シリアルコマンド
  handleSerialCommand();

  updateDisplayedSampleRate();

  if (pendingSamples == 0) {
    return;
  }

  noInterrupts();
  pendingSamples--;
  interrupts();

  processOneSample();

  serialCounter++;
  if (serialCounter >= SERIAL_EVERY_N_SAMPLES) {
    serialCounter = 0;
    outputSerial();
  }

  drawCounter++;
  if (drawCounter >= DRAW_EVERY_N_SAMPLES) {
    drawCounter = 0;
    drawGraph();
  }
}
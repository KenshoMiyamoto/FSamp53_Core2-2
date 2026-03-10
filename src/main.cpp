#include <Arduino.h>
#include "M5Unified.h"

#define PIN_INA 35
#define PIN_INB 36
#define PIN_INC 34

#define COLOR_A TFT_RED
#define COLOR_B TFT_GREEN
#define COLOR_C TFT_YELLOW

const int BUFFER_SIZE = 320;
int bufferA[BUFFER_SIZE];
int bufferB[BUFFER_SIZE];
int bufferC[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

// simple moving average LPF
const int LPF_SIZE = 10;
int lpfBufA[LPF_SIZE];
int lpfBufB[LPF_SIZE];
int lpfBufC[LPF_SIZE];
int lpfIndex = 0;
bool lpfFull = false;
long lpfSumA = 0;
long lpfSumB = 0;
long lpfSumC = 0;

int displayWidth;
int displayHeight;
int prevYA = -1;
int prevYB = -1;
int prevYC = -1;

float currentSampleRate = 0;

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

// 加速度モード関連
bool voltageMode = true;   // true: 電圧表示, false: 加速度表示
bool prevVoltageMode = true;

float accelX, accelY, accelZ;
float accelBufferX[BUFFER_SIZE];
float accelBufferY[BUFFER_SIZE];
float accelBufferZ[BUFFER_SIZE];

float accelLpfBufX[LPF_SIZE];
float accelLpfBufY[LPF_SIZE];
float accelLpfBufZ[LPF_SIZE];
float accelLpfSumX = 0;
float accelLpfSumY = 0;
float accelLpfSumZ = 0;

// モード表示ラベルを描く関数 (上部固定領域)
void drawModeLabel() {
  M5.Display.fillRect(0, 0, displayWidth, 20, TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setCursor(10, 5);
  M5.Display.printf("Mode: %s", voltageMode ? "Voltage" : "Accel");

  // 右側に色付きラベルを表示
  int startX = displayWidth - 120;
  int y = 10;
  int radius = 4;

  // チャンネルA
  M5.Display.drawCircle(startX, y, radius, COLOR_A);
  M5.Display.fillCircle(startX, y, radius, COLOR_A);
  M5.Display.setCursor(startX + 10, 5);
  M5.Display.printf("%s", voltageMode ? "A" : "X");

  // チャンネルB
  startX += 30;
  M5.Display.drawCircle(startX, y, radius, COLOR_B);
  M5.Display.fillCircle(startX, y, radius, COLOR_B);
  M5.Display.setCursor(startX + 10, 5);
  M5.Display.printf("%s", voltageMode ? "B" : "Y");

  // チャンネルC
  startX += 30;
  M5.Display.drawCircle(startX, y, radius, COLOR_C);
  M5.Display.fillCircle(startX, y, radius, COLOR_C);
  M5.Display.setCursor(startX + 10, 5);
  M5.Display.printf("%s", voltageMode ? "C" : "Z");
}

void setup() {
  M5.begin();
  M5.Imu.begin();

  displayWidth = M5.Display.width();
  displayHeight = M5.Display.height();

  M5.Display.fillScreen(TFT_BLACK);

  Serial.begin(115200);
  delay(1000);
  Serial.println("=== M5Stack Serial Start ===");

  drawModeLabel();
  prevVoltageMode = voltageMode;
}

void loop() {
  unsigned long loopStart = micros();
  M5.update();

  // ボタンAでオフセットモード切り替え
  if (M5.BtnA.wasPressed()) {
    offsetMode = !offsetMode;
    if (offsetMode) {
      collectingOffsets = true;
      sampleCount = 0;
      sumA = 0;
      sumB = 0;
      sumC = 0;
    }
  }

  // ボタンBで画面表示ON/OFF
  if (M5.BtnB.wasPressed()) {
    displayOn = !displayOn;
  }

  // ボタンCで電圧/加速度表示切り替え
  if (M5.BtnC.wasPressed()) {
    voltageMode = !voltageMode;
  }

  int rawA = analogReadMilliVolts(PIN_INA);
  int rawB = analogReadMilliVolts(PIN_INB);
  int rawC = analogReadMilliVolts(PIN_INC);

  // 加速度取得
  M5.Imu.getAccel(&accelX, &accelY, &accelZ);

  // moving average LPF update for voltage
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

  // moving average LPF update for acceleration
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

  lpfIndex++;
  if (lpfIndex >= LPF_SIZE) {
    lpfIndex = 0;
    lpfFull = true;
  }

  float valA = lpfSumA / (lpfFull ? LPF_SIZE : lpfIndex);
  float valB = lpfSumB / (lpfFull ? LPF_SIZE : lpfIndex);
  float valC = lpfSumC / (lpfFull ? LPF_SIZE : lpfIndex);

  float accelValX = accelLpfSumX / (lpfFull ? LPF_SIZE : lpfIndex);
  float accelValY = accelLpfSumY / (lpfFull ? LPF_SIZE : lpfIndex);
  float accelValZ = accelLpfSumZ / (lpfFull ? LPF_SIZE : lpfIndex);

  // オフセット収集
  if (collectingOffsets) {
    sumA += rawA;
    sumB += rawB;
    sumC += rawC;
    sampleCount++;

    if (sampleCount >= 10) {
      offsetA = sumA / 10.0;
      offsetB = sumB / 10.0;
      offsetC = sumC / 10.0;
      collectingOffsets = false;
    }
  }

  // シリアル出力
  if (voltageMode) {
    Serial.print(valA);
    Serial.print(",");
    Serial.print(valB);
    Serial.print(",");
    Serial.print(valC);
    Serial.print(",");
    Serial.println(currentSampleRate);
  } else {
    Serial.print(accelValX);
    Serial.print(",");
    Serial.print(accelValY);
    Serial.print(",");
    Serial.print(accelValZ);
    Serial.print(",");
    Serial.println(currentSampleRate);
  }

  // バッファ保存
  if (voltageMode) {
    bufferA[bufferIndex] = (int)valA;
    bufferB[bufferIndex] = (int)valB;
    bufferC[bufferIndex] = (int)valC;
  } else {
    accelBufferX[bufferIndex] = accelValX;
    accelBufferY[bufferIndex] = accelValY;
    accelBufferZ[bufferIndex] = accelValZ;
  }

  // 表示用値
  float displayValA, displayValB, displayValC;
  if (voltageMode) {
    displayValA = offsetMode ? (valA - offsetA) : valA;
    displayValB = offsetMode ? (valB - offsetB) : valB;
    displayValC = offsetMode ? (valC - offsetC) : valC;
  } else {
    displayValA = accelValX;
    displayValB = accelValY;
    displayValC = accelValZ;
  }

  // y座標計算 (上部20pxを固定領域とする)
  int yA, yB, yC;
  if (voltageMode) {
    yA = constrain(
      map((long)displayValA, offsetMode ? -1650 : 0, offsetMode ? 1650 : 3300, displayHeight - 1, 20),
      20, displayHeight - 1
    );
    yB = constrain(
      map((long)displayValB, offsetMode ? -1650 : 0, offsetMode ? 1650 : 3300, displayHeight - 1, 20),
      20, displayHeight - 1
    );
    yC = constrain(
      map((long)displayValC, offsetMode ? -1650 : 0, offsetMode ? 1650 : 3300, displayHeight - 1, 20),
      20, displayHeight - 1
    );
  } else {
    yA = constrain(map((long)(displayValA * 1000), -1000, 1000, displayHeight - 1, 20), 20, displayHeight - 1);
    yB = constrain(map((long)(displayValB * 1000), -1000, 1000, displayHeight - 1, 20), 20, displayHeight - 1);
    yC = constrain(map((long)(displayValC * 1000), -1000, 1000, displayHeight - 1, 20), 20, displayHeight - 1);
  }

  if (displayOn) {
    if (bufferFull) {
      // スクロール前に上部固定領域を再描画
      drawModeLabel();

      // 画面を左にスクロール
      M5.Display.scroll(-5, 0);

      // 新しい点を右端に描画
      int x = displayWidth - 1;

      if (prevYA != -1) {
        M5.Display.drawLine(x - 4, prevYA, x, yA, COLOR_A);
      } else {
        M5.Display.drawPixel(x, yA, COLOR_A);
      }

      if (prevYB != -1) {
        M5.Display.drawLine(x - 4, prevYB, x, yB, COLOR_B);
      } else {
        M5.Display.drawPixel(x, yB, COLOR_B);
      }

      if (prevYC != -1) {
        M5.Display.drawLine(x - 4, prevYC, x, yC, COLOR_C);
      } else {
        M5.Display.drawPixel(x, yC, COLOR_C);
      }
    } else {
      // バッファが満杯でない場合、順に描画
      int x = bufferIndex;

      if (prevYA != -1) {
        M5.Display.drawLine(x - 1, prevYA, x, yA, COLOR_A);
      } else {
        M5.Display.drawPixel(x, yA, COLOR_A);
      }

      if (prevYB != -1) {
        M5.Display.drawLine(x - 1, prevYB, x, yB, COLOR_B);
      } else {
        M5.Display.drawPixel(x, yB, COLOR_B);
      }

      if (prevYC != -1) {
        M5.Display.drawLine(x - 1, prevYC, x, yC, COLOR_C);
      } else {
        M5.Display.drawPixel(x, yC, COLOR_C);
      }

      // 上部固定領域を描画
      drawModeLabel();
    }
  }

  prevYA = yA;
  prevYB = yB;
  prevYC = yC;

  bufferIndex++;
  if (bufferIndex >= BUFFER_SIZE) {
    bufferFull = true;
    bufferIndex = 0;
  }

  unsigned long loopEnd = micros();
  currentSampleRate = 1000000.0 / (loopEnd - loopStart);

  delay(2);
}
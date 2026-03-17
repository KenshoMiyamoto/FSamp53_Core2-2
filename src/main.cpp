#include <Arduino.h>
#include "M5Unified.h"

#define PIN_INA 35
#define PIN_INB 36
#define PIN_INC 34

#define COLOR_A TFT_RED
#define COLOR_B TFT_GREEN
#define COLOR_C TFT_YELLOW

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
int lpfIndex = 0;
bool lpfFull = false;
long lpfSumA = 0;
long lpfSumB = 0;
long lpfSumC = 0;

int displayWidth;
int displayHeight;

// 描画用の現在X位置
int plotX = 25;

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

float currentSampleRate = 0;

// 左側スケール領域
const int LEFT_MARGIN = 25;

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

float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

float accelBufferX[BUFFER_SIZE];
float accelBufferY[BUFFER_SIZE];
float accelBufferZ[BUFFER_SIZE];

float accelLpfBufX[LPF_SIZE];
float accelLpfBufY[LPF_SIZE];
float accelLpfBufZ[LPF_SIZE];
float gyroLpfBufX[LPF_SIZE];
float gyroLpfBufY[LPF_SIZE];
float gyroLpfBufZ[LPF_SIZE];

float accelLpfSumX = 0;
float accelLpfSumY = 0;
float accelLpfSumZ = 0;
float gyroLpfSumX = 0;
float gyroLpfSumY = 0;
float gyroLpfSumZ = 0;

// 力計算用変数
float forceX, forceY, forceZ;
float forceBufferX[BUFFER_SIZE];
float forceBufferY[BUFFER_SIZE];
float forceBufferZ[BUFFER_SIZE];

float forceLpfBufX[LPF_SIZE];
float forceLpfBufY[LPF_SIZE];
float forceLpfBufZ[LPF_SIZE];
float forceLpfSumX = 0;
float forceLpfSumY = 0;
float forceLpfSumZ = 0;

// 速度計算用変数
float velocityX = 0.0;
float velocityY = 0.0;
float velocityZ = 0.0;

float velocityLpfBufX[LPF_SIZE];
float velocityLpfBufY[LPF_SIZE];
float velocityLpfBufZ[LPF_SIZE];
float velocityLpfSumX = 0;
float velocityLpfSumY = 0;
float velocityLpfSumZ = 0;

// ---------- 共通関数 ----------

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

void drawModeLabel() {
  M5.Display.fillRect(0, 0, displayWidth, 20, TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(10, 5);

  const char* modeStr[] = {"Raw Volt", "Off Volt", "IMU", "Force"};
  M5.Display.printf("Mode: %s", modeStr[displayMode]);

  int startX = displayWidth - 120;
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

  if (displayMode == 0 || displayMode == 1) {
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
  // 左スケールと上部ラベル以外の波形領域だけ消す
  M5.Display.fillRect(LEFT_MARGIN, 20, displayWidth - LEFT_MARGIN, displayHeight - 20, TFT_BLACK);
  drawModeLabel();
  drawLeftScale();
  drawIMUSeparator();
}

void setup() {
  // M5Stackの初期化
  M5.begin();
  // IMUセンサーの初期化
  M5.Imu.begin();

  // ディスプレイサイズの取得
  displayWidth = M5.Display.width();
  displayHeight = M5.Display.height();
  // プロット開始位置の設定
  plotX = LEFT_MARGIN;

  // 画面を黒で塗りつぶす
  M5.Display.fillScreen(TFT_BLACK);

  // シリアル通信の開始
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== M5Stack Serial Start ===");

  // LPFバッファの初期化
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

    velocityLpfBufX[i] = 0;
    velocityLpfBufY[i] = 0;
    velocityLpfBufZ[i] = 0;

    forceLpfBufX[i] = 0;
    forceLpfBufY[i] = 0;
    forceLpfBufZ[i] = 0;
  }

  // 初期表示の描画
  drawModeLabel();
  drawLeftScale();
  drawIMUSeparator();
}

void loop() {
  // ループ開始時間の記録
  unsigned long loopStart = micros();
  M5.update();

  // ボタンA: オフセットモードの切り替え
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

  // ボタンB: 表示のオン/オフ切り替え
  if (M5.BtnB.wasPressed()) {
    displayOn = !displayOn;
  }

  // ボタンC: 表示モードの切り替え (0: Raw Volt, 1: Off Volt, 2: IMU, 3: Force)
  if (M5.BtnC.wasPressed()) {
    displayMode = (displayMode + 1) % 4;
    plotX = LEFT_MARGIN;
    resetPrevPoints();
    clearPlotArea();
  }

  // アナログ入力の読み取り (ミリボルト単位)
  int rawA = analogReadMilliVolts(PIN_INA);
  int rawB = analogReadMilliVolts(PIN_INB);
  int rawC = analogReadMilliVolts(PIN_INC);

  // IMUデータの取得
  M5.Imu.getAccel(&accelX, &accelY, &accelZ);
  M5.Imu.getGyro(&gyroX, &gyroY, &gyroZ);

  // LPF voltage (電圧データのローパスフィルタ処理)
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

  // LPF accel (加速度データのローパスフィルタ処理)
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

  // LPF gyro (角速度データのローパスフィルタ処理)
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

  // LPFインデックスの更新
  lpfIndex++;
  if (lpfIndex >= LPF_SIZE) {
    lpfIndex = 0;
    lpfFull = true;
  }

  // LPFで平均化した値の計算
  int denom = lpfFull ? LPF_SIZE : lpfIndex;
  if (denom <= 0) denom = 1;

  float valA = lpfSumA / (float)denom;
  float valB = lpfSumB / (float)denom;
  float valC = lpfSumC / (float)denom;

  float accelValX = accelLpfSumX / (float)denom;
  float accelValY = accelLpfSumY / (float)denom;
  float accelValZ = accelLpfSumZ / (float)denom;

  float gyroValX = gyroLpfSumX / (float)denom;
  float gyroValY = gyroLpfSumY / (float)denom;
  float gyroValZ = gyroLpfSumZ / (float)denom;

  // 速度計算 (積分による速度推定)
  if (currentSampleRate > 0) {
    float dt = 1.0f / currentSampleRate;
    velocityX += accelValX * dt;
    velocityY += accelValY * dt;
    velocityZ += accelValZ * dt;
  }

  // LPF velocity (速度データのローパスフィルタ処理)
  velocityLpfSumX += velocityX;
  velocityLpfSumY += velocityY;
  velocityLpfSumZ += velocityZ;

  if (lpfFull) {
    velocityLpfSumX -= velocityLpfBufX[lpfIndex];
    velocityLpfSumY -= velocityLpfBufY[lpfIndex];
    velocityLpfSumZ -= velocityLpfBufZ[lpfIndex];
  }

  velocityLpfBufX[lpfIndex] = velocityX;
  velocityLpfBufY[lpfIndex] = velocityY;
  velocityLpfBufZ[lpfIndex] = velocityZ;

  float velocityValX = velocityLpfSumX / (float)denom;
  float velocityValY = velocityLpfSumY / (float)denom;
  float velocityValZ = velocityLpfSumZ / (float)denom;

  // オフセット収集 (ボタンA押下時に基準値を取得)
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

  // 力計算 (オフセット済み電圧をマトリックス変換して力に変換)
  float volA_off = (valA - offsetA) / 1000.0;
  float volB_off = (valB - offsetB) / 1000.0;
  float volC_off = (valC - offsetC) / 1000.0;

  forceX = MATRIX[0][0] * volA_off + MATRIX[0][1] * volB_off + MATRIX[0][2] * volC_off;
  forceY = MATRIX[1][0] * volA_off + MATRIX[1][1] * volB_off + MATRIX[1][2] * volC_off;
  forceZ = MATRIX[2][0] * volA_off + MATRIX[2][1] * volB_off + MATRIX[2][2] * volC_off;

  // LPF force (力データのローパスフィルタ処理)
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

  float forceValX = forceLpfSumX / (float)denom;
  float forceValY = forceLpfSumY / (float)denom;
  float forceValZ = forceLpfSumZ / (float)denom;

  // シリアル出力 (現在のモードに応じたデータをCSV形式で出力)
  if (displayMode == 0) {
    // モード0: 生の電圧値 (V単位) + サンプリングレート
    Serial.print(rawA / 1000.0);
    Serial.print(",");
    Serial.print(rawB / 1000.0);
    Serial.print(",");
    Serial.print(rawC / 1000.0);
    Serial.print(",");
    Serial.println(currentSampleRate);

  } else if (displayMode == 1) {
    // モード1: LPF済み電圧値 (mV単位) + サンプリングレート
    Serial.print(valA);
    Serial.print(",");
    Serial.print(valB);
    Serial.print(",");
    Serial.print(valC);
    Serial.print(",");
    Serial.println(currentSampleRate);

  } else if (displayMode == 2) {
    // モード2: 加速度 + 角速度 + サンプリングレート
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
    // モード3: 力 + サンプリングレート
    Serial.print(forceValX);
    Serial.print(",");
    Serial.print(forceValY);
    Serial.print(",");
    Serial.print(forceValZ);
    Serial.print(",");
    Serial.println(currentSampleRate);
  }

  // 表示用値の設定 (モードに応じて表示する値を選択)
  float displayValA, displayValB, displayValC;

  if (displayMode == 0) {
    // 生の電圧値
    displayValA = rawA;
    displayValB = rawB;
    displayValC = rawC;
  } else if (displayMode == 1) {
    // オフセットモード時はオフセット済み値、そうでなければLPF値
    displayValA = offsetMode ? (valA - offsetA) : valA;
    displayValB = offsetMode ? (valB - offsetB) : valB;
    displayValC = offsetMode ? (valC - offsetC) : valC;
  } else if (displayMode == 2) {
    // IMUモード: 加速度値
    displayValA = accelValX;
    displayValB = accelValY;
    displayValC = accelValZ;
  } else {
    // フォースモード: 力の値
    displayValA = forceValX;
    displayValB = forceValY;
    displayValC = forceValZ;
  }

  // y座標計算 (値を画面上のピクセル座標に変換)
  int yA_accel, yB_accel, yC_accel;
  int yA_vel, yB_vel, yC_vel;

  if (displayMode == 0) {
    // 電圧モード: 0-3.3Vを画面高さにマッピング
    yA_accel = constrain(map((long)displayValA, 0, 3300, displayHeight - 1, 20), 20, displayHeight - 1);
    yB_accel = constrain(map((long)displayValB, 0, 3300, displayHeight - 1, 20), 20, displayHeight - 1);
    yC_accel = constrain(map((long)displayValC, 0, 3300, displayHeight - 1, 20), 20, displayHeight - 1);
    yA_vel = yA_accel;
    yB_vel = yB_accel;
    yC_vel = yC_accel;

  } else if (displayMode == 1) {
    // オフセット電圧モード: -1.65V～+1.65Vを画面高さにマッピング
    yA_accel = constrain(map((long)displayValA, -1650, 1650, displayHeight - 1, 20), 20, displayHeight - 1);
    yB_accel = constrain(map((long)displayValB, -1650, 1650, displayHeight - 1, 20), 20, displayHeight - 1);
    yC_accel = constrain(map((long)displayValC, -1650, 1650, displayHeight - 1, 20), 20, displayHeight - 1);
    yA_vel = yA_accel;
    yB_vel = yB_accel;
    yC_vel = yC_accel;

  } else if (displayMode == 2) {
    // IMUモード: 上半分に加速度、下半分に角速度を表示
    int midY = (displayHeight + 20) / 2;

    // 加速度: -1.0～+1.0 m/s² を上半分にマッピング
    yA_accel = constrain(map((long)(displayValA * 1000), -1000, 1000, midY - 1, 20), 20, midY - 1);
    yB_accel = constrain(map((long)(displayValB * 1000), -1000, 1000, midY - 1, 20), 20, midY - 1);
    yC_accel = constrain(map((long)(displayValC * 1000), -1000, 1000, midY - 1, 20), 20, midY - 1);

    // 角速度: -250～+250 dps を下半分にマッピング
    yA_vel = constrain(map((long)gyroValX, -250, 250, displayHeight - 1, midY), midY, displayHeight - 1);
    yB_vel = constrain(map((long)gyroValY, -250, 250, displayHeight - 1, midY), midY, displayHeight - 1);
    yC_vel = constrain(map((long)gyroValZ, -250, 250, displayHeight - 1, midY), midY, displayHeight - 1);

  } else {
    // フォースモード: -2.0～+2.0 N を画面高さにマッピング
    yA_accel = constrain(map((long)(displayValA * 1000), -2000, 2000, displayHeight - 1, 20), 20, displayHeight - 1);
    yB_accel = constrain(map((long)(displayValB * 1000), -2000, 2000, displayHeight - 1, 20), 20, displayHeight - 1);
    yC_accel = constrain(map((long)(displayValC * 1000), -2000, 2000, displayHeight - 1, 20), 20, displayHeight - 1);
    yA_vel = yA_accel;
    yB_vel = yB_accel;
    yC_vel = yC_accel;
  }

  // 右端に来たら左へ戻る (スクロール表示)
  if (plotX >= displayWidth) {
    plotX = LEFT_MARGIN;
    resetPrevPoints();
    clearPlotArea();
  }

  // グラフ描画 (表示がオンの場合のみ)
  if (displayOn) {
    int x = plotX;

    if (displayMode == 2) {
      // IMUモード: 上半分に加速度、下半分に角速度を描画
      // 上半分: 加速度
      if (prevYA_accel != -1) M5.Display.drawLine(x - 1, prevYA_accel, x, yA_accel, COLOR_A);
      else M5.Display.drawPixel(x, yA_accel, COLOR_A);

      if (prevYB_accel != -1) M5.Display.drawLine(x - 1, prevYB_accel, x, yB_accel, COLOR_B);
      else M5.Display.drawPixel(x, yB_accel, COLOR_B);

      if (prevYC_accel != -1) M5.Display.drawLine(x - 1, prevYC_accel, x, yC_accel, COLOR_C);
      else M5.Display.drawPixel(x, yC_accel, COLOR_C);

      // 下半分: 角速度
      if (prevYA_gyro != -1) M5.Display.drawLine(x - 1, prevYA_gyro, x, yA_vel, COLOR_A);
      else M5.Display.drawPixel(x, yA_vel, COLOR_A);

      if (prevYB_gyro != -1) M5.Display.drawLine(x - 1, prevYB_gyro, x, yB_vel, COLOR_B);
      else M5.Display.drawPixel(x, yB_vel, COLOR_B);

      if (prevYC_gyro != -1) M5.Display.drawLine(x - 1, prevYC_gyro, x, yC_vel, COLOR_C);
      else M5.Display.drawPixel(x, yC_vel, COLOR_C);

    } else if (displayMode == 3) {
      // フォースモード: XYZ軸の力を描画
      if (prevYA_force != -1) M5.Display.drawLine(x - 1, prevYA_force, x, yA_accel, COLOR_A);
      else M5.Display.drawPixel(x, yA_accel, COLOR_A);

      if (prevYB_force != -1) M5.Display.drawLine(x - 1, prevYB_force, x, yB_accel, COLOR_B);
      else M5.Display.drawPixel(x, yB_accel, COLOR_B);

      if (prevYC_force != -1) M5.Display.drawLine(x - 1, prevYC_force, x, yC_accel, COLOR_C);
      else M5.Display.drawPixel(x, yC_accel, COLOR_C);

    } else {
      // 電圧モード: 3つの電圧チャンネルを描画
      if (prevYA != -1) M5.Display.drawLine(x - 1, prevYA, x, yA_accel, COLOR_A);
      else M5.Display.drawPixel(x, yA_accel, COLOR_A);

      if (prevYB != -1) M5.Display.drawLine(x - 1, prevYB, x, yB_accel, COLOR_B);
      else M5.Display.drawPixel(x, yB_accel, COLOR_B);

      if (prevYC != -1) M5.Display.drawLine(x - 1, prevYC, x, yC_accel, COLOR_C);
      else M5.Display.drawPixel(x, yC_accel, COLOR_C);
    }
  }

  // prev座標の更新 (次の描画で線を引くために使用)
  if (displayMode == 2) {
    // IMUモード: 加速度と角速度のprev座標を更新
    prevYA_accel = yA_accel;
    prevYB_accel = yB_accel;
    prevYC_accel = yC_accel;

    prevYA_gyro = yA_vel;
    prevYB_gyro = yB_vel;
    prevYC_gyro = yC_vel;

  } else if (displayMode == 3) {
    // フォースモード: 力のprev座標を更新
    prevYA_force = yA_accel;
    prevYB_force = yB_accel;
    prevYC_force = yC_accel;

  } else {
    // 電圧モード: 電圧のprev座標を更新
    prevYA = yA_accel;
    prevYB = yB_accel;
    prevYC = yC_accel;
  }

  // プロット位置を進める
  plotX++;

  // サンプリングレートの計算
  unsigned long loopEnd = micros();
  unsigned long dtMicros = loopEnd - loopStart;
  if (dtMicros == 0) dtMicros = 1;
  currentSampleRate = 1000000.0f / dtMicros;

  delay(4);
}
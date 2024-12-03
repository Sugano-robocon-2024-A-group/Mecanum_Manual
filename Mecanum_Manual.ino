#include <Arduino.h>
#include <PS4Controller.h>

// PID制御のゲイン
float kp[4] = {2.0, 2.0, 2.0, 2.0};
float ki[4] = {0.1, 0.1, 0.1, 0.1};
float kd[4] = {0.1, 0.1, 0.1, 0.3};

// 1回の入力で移動する距離(mm)
float travelDistance = 50.0;

//許容誤差5mm
const int allowableError = 5;

// 移動距離の目標設定（mm単位）
float targetDistance[4]; // 各ホイールの移動距離目標

// エンコーダ関連の設定
const float wheelDiameter = 80.0; // ホイールの直径（mm）
const float encoderPulsesPerRevolution = 750; // エンコーダの1回転あたりのパルス数
const float distancePerCount = (PI * wheelDiameter) / encoderPulsesPerRevolution; // 1カウントあたりの距離（mm）

// モーターとエンコーダのピン設定 （左前　左後　右前　右後）
const int motor_pwm[4] = {21, 13, 16, 4}; // PWM出力ピン
const int motor_dir1[4] = {18, 27, 22, 32}; // 方向ピン1
const int motor_dir2[4] = {19, 14, 23, 33}; // 方向ピン2
const int encoderA[4] = {34, 39, 36, 39}; // エンコーダA相ピン
const int encoderB[4] = {18, 34, 21, 17}; // エンコーダB相ピン

// エンコーダ関連の変数
volatile long encoderCount[4] = {0, 0, 0, 0}; // 各ホイールのエンコーダカウント

// PID制御に関する変数
float integral[4] = {0, 0, 0, 0};
float previous_error[4] = {0, 0, 0, 0};

void handleEncoder(int index)
{
  if (index < 2)  // 左側のモーターの場合
  {
    if (digitalRead(encoderA[index]) != digitalRead(encoderB[index]))
    {
    encoderCount[index]++;
    }
    else
    {
      encoderCount[index]--;
    }
  }
  else  // 右側のモーターの場合
  {
    if (digitalRead(encoderA[index]) == digitalRead(encoderB[index]))
    {
      encoderCount[index]++;
    }
    else
    {
      encoderCount[index]--;
    }    
  }
}

void init_pwm_setup()
{
  for (int i = 0; i < 4; i++)
  {
    // モーターピンを出力に設定
    pinMode(motor_pwm[i], OUTPUT);
    pinMode(motor_dir1[i], OUTPUT);
    pinMode(motor_dir2[i], OUTPUT);
  }

  // PWM設定
  for (int i = 0; i < 4; i++)
  {
    ledcAttach(motor_pwm[i], 1000, 8); // PWM出力ピン、周波数5000Hz、8ビット解像度
  }
}

void init_encoder_setup()
{
  for (int i = 0; i < 4; i++)
  {
    // モーターピンを出力に設定
    pinMode(encoderA[i], INPUT_PULLUP);
    pinMode(encoderB[i], INPUT_PULLUP);
  }
}

// PID制御計算
float pidCompute(int index, float target, float current)
{
  float error = target - current;
  integral[index] += error;
  float derivative = error - previous_error[index];
  float output = kp[index] * error + ki[index] * integral[index] + kd[index] * derivative;
  previous_error[index] = error;
  return constrain(output, -256, 256);
}

// モーターの駆動方向を制御
void driveMotor(int index, float controlSignal)
{
  if (controlSignal >= 0)
  {
    digitalWrite(motor_dir1[index], HIGH);
    digitalWrite(motor_dir2[index], LOW);
  }
  else
  {
    digitalWrite(motor_dir1[index], LOW);
    digitalWrite(motor_dir2[index], HIGH);
    controlSignal = -controlSignal;
  }
  ledcWrite(motor_pwm[index], (int)controlSignal);
}

// モーターを停止
void stopMotors()
{
  for (int i = 0; i < 4; i++)
  {
    ledcWrite(motor_pwm[i], 0);
    digitalWrite(motor_dir1[i], LOW);
    digitalWrite(motor_dir2[i], LOW);
    targetDistance[i] = 0;
  }
}

void setup()
{
  init_pwm_setup();
  init_encoder_setup();
  
  stopMotors();
  
  attachInterrupt(digitalPinToInterrupt(encoderA[0]), []() { handleEncoder(0); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA[1]), []() { handleEncoder(1); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA[2]), []() { handleEncoder(2); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA[3]), []() { handleEncoder(3); }, CHANGE);

  Serial.begin(115200);
  PS4.begin("1c:69:20:e6:20:d2");
  Serial.println("Ready.");
}

void loop()
{
  bool reachedTarget = true;

  for (int i = 0; i < 4; i++)
  {
    float currentDistance = encoderCount[0] * distancePerCount; // 現在の移動距離
    float controlSignal = pidCompute(i, targetDistance[i], currentDistance);

    // 制御信号に基づいてモーターを駆動
    driveMotor(i, controlSignal);

    // シリアルモニタでデバッグ情報出力
    Serial.print("Wheel ");
    Serial.print(i);
    Serial.print(" Target Distance: ");
    Serial.print(targetDistance[i]);
    Serial.print(" Current Distance: ");
    Serial.print(currentDistance);
    Serial.print(" Control Signal: ");
    Serial.println(controlSignal);

    // 全ホイールが目標距離に到達したかを確認
    if (abs(currentDistance - targetDistance[i]) > allowableError)
    {
      reachedTarget = false;
    }
  }

  if (reachedTarget)
  {
    stopMotors(); // 現在の移動を停止

    // エンコーダカウントとPID制御に関する変数のリセット
    for (int i = 0; i < 4; i++)
    {
      encoderCount[i] = 0;

      integral[i] = 0.0;
      previous_error[i] = 0.0;
    }
  
    if (PS4.isConnected())
    {
      if (PS4.Right())
      {
        Serial.println("Right");
        targetDistance[0] = travelDistance;
        targetDistance[1] = -travelDistance;
        targetDistance[2] = -travelDistance;
        targetDistance[3] = travelDistance;
      }
      if (PS4.Down())
      {
        Serial.println("Down");
        targetDistance[0] = -travelDistance;
        targetDistance[1] = -travelDistance;
        targetDistance[2] = -travelDistance;
        targetDistance[3] = -travelDistance;
      }
      if (PS4.Up())
      {
        Serial.println("Up");
        targetDistance[0] = travelDistance;
        targetDistance[1] = travelDistance;
        targetDistance[2] = travelDistance;
        targetDistance[3] = travelDistance;
      }
      if (PS4.Left())
      {
        Serial.println("Left");
        targetDistance[0] = -travelDistance;
        targetDistance[1] = travelDistance;
        targetDistance[2] = travelDistance;
        targetDistance[3] = -travelDistance;
      }

      if (PS4.UpRight())
      {
        Serial.println("Up Right");
        targetDistance[0] = travelDistance;
        targetDistance[1] = 0.0;
        targetDistance[2] = 0.0;
        targetDistance[3] = travelDistance;
      }
      if (PS4.DownRight())
      {
        Serial.println("Down Right");
        targetDistance[0] = 0.0;
        targetDistance[1] = -travelDistance;
        targetDistance[2] = -travelDistance;
        targetDistance[3] = 0.0;
      }
      if (PS4.UpLeft())
      {
        Serial.println("Up Left");
        targetDistance[0] = 0.0;
        targetDistance[1] = travelDistance;
        targetDistance[2] = travelDistance;
        targetDistance[3] = 0.0;
      }
      if (PS4.DownLeft())
      {
        Serial.println("Down Left");
        targetDistance[0] = -travelDistance;
        targetDistance[1] = 0.0;
        targetDistance[2] = 0.0;
        targetDistance[3] = -travelDistance;
      }
    }
  }
}

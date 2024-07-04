#include "MPU9250.h"

MPU9250 mpu;

// 放校正的值
float acc_b[3] = {-33.18, -15.28, 4.39};
float gyro_b[3] = {0.96, 1.16, -0.53};
float mag_b[3] = {258.66, 100.51, -176.88};
float mag_s[3] = {1.04, 1.00, 0.96};

// 單位轉換用的常數
float acc_s = (float)MPU9250::CALIB_ACCEL_SENSITIVITY / 1000.f;
float gyro_s = (float)MPU9250::CALIB_GYRO_SENSITIVITY;

unsigned long timer = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    // 確認有正確連接到MPU9250
    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // calibrate anytime you want to
    // Serial.println("Accel Gyro calibration will start in 5sec.");
    // Serial.println("Please leave the device still on the flat plane.");
    // mpu.verbose(true);
    // delay(5000);
    // mpu.calibrateAccelGyro();

    // Serial.println("Mag calibration will start in 5sec.");
    // Serial.println("Please Wave device in a figure eight until done.");
    // delay(5000);
    // mpu.calibrateMag();
    
    // 將校正值應用上去
    mpu.setAccBias(acc_b[0]*acc_s, acc_b[1]*acc_s, acc_b[2]*acc_s);

    mpu.setGyroBias(gyro_b[0]*gyro_s, gyro_b[1]*gyro_s, gyro_b[2]*gyro_s);

    mpu.setMagBias(mag_b[0], mag_b[1], mag_b[2]);

    mpu.setMagScale(mag_s[0], mag_s[1], mag_s[2]);

    mpu.setMagneticDeclination(-5.08); // Taiwan, 9th May
    // print_calibration();

    // 告訴程式sensor以校正
    mpu.verbose(false);
}

void loop() {
    timer = millis();

    // 這邊是預計如果突然想校正一下的時候可以用，所以分成兩個mode
    // 但目前還只能在Arduino上使用
    // python目前不知道怎麼輸入值給Arduino接收
    int mode = 1;
    if (Serial.available() && Serial.parseInt() == 1) {
      mode = 0;
    }
    // calibrate anytime you want to
    if (mode == 0) {
      Serial.println("Accel Gyro calibration will start in 5sec.");
      Serial.println("Please leave the device still on the flat plane.");
      mpu.verbose(true);
      delay(5000);
      mpu.calibrateAccelGyro();

      Serial.println("Mag calibration will start in 5sec.");
      Serial.println("Please Wave device in a figure eight until done.");
      delay(5000);
      mpu.calibrateMag();
    }

    // 如果可以正常update數據，就將update的結果印出來
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        // float mx = mpu.getMag(0);
        // float my = mpu.getMag(1);
        // float yaw = -1 * (atan2(my, mx) * 180 / M_PI);
        if (millis() > prev_ms + 25) {
            // print_roll_pitch_yaw();
            // Serial.println(yaw);
            // Serial.print(mpu.getMag(0)); Serial.print(",");
            // Serial.print(mpu.getMag(1)); Serial.print(",");
            // Serial.print(mpu.getMag(2)); Serial.println("");
            prev_ms = millis();
        }
        print_rot_mat();
    }
    //delay(10 - (millis() - timer));
    delay(10);
}

// 用來print歐拉角的函式
void print_roll_pitch_yaw() {
    // Serial.print("Pitch, Roll, Yaw: ");
    Serial.print(mpu.getEulerY(), 2);
    Serial.print(", ");
    Serial.print(mpu.getEulerX(), 2);
    Serial.print(", ");
    Serial.println(mpu.getEulerZ(), 2);
    // delay(40);
}

// 用來print旋轉矩陣的函式
void print_rot_mat() {
  Serial.print(mpu.getR11(), 2);
  Serial.print(", ");
  Serial.print(mpu.getR12(), 2);
  Serial.print(", ");
  Serial.print(mpu.getR13(), 2);
  Serial.print(", ");
  Serial.print(mpu.getR21(), 2);
  Serial.print(", ");
  Serial.print(mpu.getR22(), 2);
  Serial.print(", ");
  Serial.print(mpu.getR23(), 2);
  Serial.print(", ");
  Serial.print(mpu.getR31(), 2);
  Serial.print(", ");
  Serial.print(mpu.getR32(), 2);
  Serial.print(", ");
  Serial.println(mpu.getR33(), 2);
}

// 用來print校正值的函式
void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / gyro_s);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / gyro_s);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / gyro_s);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
    Serial.println("Mag Factory Calibration Values:");
    Serial.print(mpu.getMagOffsetX());
    Serial.print(", ");
    Serial.print(mpu.getMagOffsetY());
    Serial.print(", ");
    Serial.print(mpu.getMagOffsetZ());
    Serial.println();
}

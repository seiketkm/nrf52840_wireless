#include <bluefruit.h>
#include "LSM6DS3.h"
#include "Wire.h"

#define CLEAR_STEP      true
#define NOT_CLEAR_STEP  false

//Create a instance of class LSM6DS3
LSM6DS3 lsm6ds3(I2C_MODE, 0x6A);    //I2C device address 0x6A
uint16_t detectCount = 0;

// Nordic UART Service (NUS) の UUID
static BLEUart  bleuart;

// デバイス情報サービス（Manufacturer / Model）
static BLEDis   bledis;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  if (lsm6ds3.begin() != 0) {
      Serial.println("Device error");
  } else {
      Serial.println("Device OK!");
  }
  if (0 != config_free_fall_detect()) {
      Serial.println("Fail to configure!");
  } else {
      Serial.println("Success to Configure!");
  }
  // BLE スタック初期化
  Bluefruit.begin();
  Bluefruit.setName("XIAO52840");         // 端末に表示される名前
  Bluefruit.setTxPower(4);                // 送信出力（0–4 dBm）

  // UART サービス開始
  bleuart.begin();

  // Device Info Service 設定（任意）
  bledis.setManufacturer("Seeed");
  bledis.setModel("XIAO nRF52840 Sense");
  bledis.begin();

  // アドバタイズ設定
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.start(0);         // 無限にアドバタイズ

  Serial.println("BLE UART Advertising...");

  
}

void loop() {
  // BLE 経由で受信したら、そのままエコーバック
  if (bleuart.available()) {
    uint8_t buf[64];
    int len = bleuart.read(buf, sizeof(buf));
    Serial.print("受信: ");
    Serial.write(buf, len);
    Serial.println();
    bleuart.write(buf, len);
  }
  uint8_t readDataByte = 0;
  //Read the wake-up source register
  lsm6ds3.readRegister(&readDataByte, LSM6DS3_ACC_GYRO_WAKE_UP_SRC);
  //Mask off the FF_IA bit for free-fall detection
  readDataByte &= 0x20;
  if (readDataByte) {
      detectCount ++;
      Serial.print("Free fall detected!  ");
      Serial.println(detectCount);
      bleuart.write("free falled");
  }
  delay(10);
}

int config_free_fall_detect(void) {
    uint8_t error = 0;
    uint8_t dataToWrite = 0;

    dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
    dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
    dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;

    error += lsm6ds3.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);
    error += lsm6ds3.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00);
    error += lsm6ds3.writeRegister(LSM6DS3_ACC_GYRO_FREE_FALL, 0x33);
    error += lsm6ds3.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x10);
    error += lsm6ds3.writeRegister(LSM6DS3_ACC_GYRO_MD2_CFG, 0x10);
    error += lsm6ds3.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x81);

    return error;
}

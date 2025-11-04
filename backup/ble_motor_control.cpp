// motor_control_ble.ino - 使用 BLE (Bluetooth Low Energy) 控制 DRV8833 馬達驅動器
// 適用於 ESP32-C3 SuperMini 搭配 DRV8833 (根據 netlist_rev8.txt 的連線)

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// 定義馬達控制針腳 (來自 netlist_rev8.txt)
// 馬達 A (M1)
#define M1_IN1_PIN 3  // ESP_IO3
#define M1_IN2_PIN 2  // ESP_IO2
// 馬達 B (M2)
#define M2_IN1_PIN 10 // ESP_IO10
#define M2_IN2_PIN 7  // ESP_IO7

// BLE 服務 UUIDs (使用隨機生成的 UUID)
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// BLE 伺服器、服務和特性物件
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;

// 接收來自 BLE 客戶端指令的回調函數
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();

        if (rxValue.length() > 0) {
            // 預期接收格式: "M1:<speed>,M2:<speed>"
            // speed 範圍應為 -255 (反轉全速) 到 255 (正轉全速)。0 為停止。
            // 範例: "M1:150,M2:-100"

            Serial.print("Received Value: ");
            Serial.println(rxValue.c_str());
            
            // 解析指令
            int m1_speed = 0;
            int m2_speed = 0;

            // 尋找 M1:
            size_t m1_pos = rxValue.find("M1:");
            if (m1_pos != std::string::npos) {
                // 尋找逗號或字串結尾
                size_t end_pos = rxValue.find(",", m1_pos);
                std::string m1_str = (end_pos == std::string::npos) 
                                     ? rxValue.substr(m1_pos + 3)
                                     : rxValue.substr(m1_pos + 3, end_pos - (m1_pos + 3));
                try {
                    m1_speed = std::stoi(m1_str);
                } catch (...) {
                    Serial.println("Error parsing M1 speed.");
                }
            }

            // 尋找 M2:
            size_t m2_pos = rxValue.find("M2:");
            if (m2_pos != std::string::npos) {
                std::string m2_str = rxValue.substr(m2_pos + 3);
                 try {
                    m2_speed = std::stoi(m2_str);
                } catch (...) {
                    Serial.println("Error parsing M2 speed.");
                }
            }

            // 執行馬達控制
            setMotorSpeed(M1_IN1_PIN, M1_IN2_PIN, m1_speed);
            setMotorSpeed(M2_IN1_PIN, M2_IN2_PIN, m2_speed);
        }
    }
};

// 處理連線/斷開事件
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE Client Connected.");
      // 在連線時停止馬達
      setMotorSpeed(M1_IN1_PIN, M1_IN2_PIN, 0);
      setMotorSpeed(M2_IN1_PIN, M2_IN2_PIN, 0);
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE Client Disconnected.");
      // 在斷開連線時停止馬達
      setMotorSpeed(M1_IN1_PIN, M1_IN2_PIN, 0);
      // 重新開始廣播
      pServer->startAdvertising();
    }
};

/**
 * @brief 設定馬達速度和方向
 * * @param in1_pin 馬達驅動器 IN1 針腳
 * @param in2_pin 馬達驅動器 IN2 針腳
 * @param speed 速度值 (-255 到 255)。正值為正轉，負值為反轉，0 為停止。
 */
void setMotorSpeed(int in1_pin, int in2_pin, int speed) {
    // 確保速度在 -255 到 255 之間
    speed = constrain(speed, -255, 255);
    int absSpeed = abs(speed);
    
    if (speed > 0) { // 正轉
        // IN1 (正轉) 使用 PWM
        ledcWrite(in1_pin, absSpeed);
        // IN2 (反轉) 設為 Low
        ledcWrite(in2_pin, 0);
    } else if (speed < 0) { // 反轉
        // IN1 (正轉) 設為 Low
        ledcWrite(in1_pin, 0);
        // IN2 (反轉) 使用 PWM
        ledcWrite(in2_pin, absSpeed);
    } else { // 停止
        ledcWrite(in1_pin, 0);
        ledcWrite(in2_pin, 0);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE Motor Control (DRV8833)...");

    // ----------------------------------------------------
    // 1. 初始化 PWM 輸出 (DRV8833 支援 PWM 速度控制)
    // ----------------------------------------------------
    // 使用 8-bit 解析度 (0-255)
    const int freq = 5000;
    const int resolution = 8; 

    // LEDC 通道分配 (確保使用未被 ESP32-C3 內部使用的 GPIO 針腳)
    // ESP32-C3 的 GPIO 針腳可以直接作為 LEDC (PWM) 輸出
    
    // 設置馬達 A 的 PWM 輸出
    ledcSetup(M1_IN1_PIN, freq, resolution);
    ledcAttachPin(M1_IN1_PIN, M1_IN1_PIN); // 使用針腳編號作為通道編號
    ledcSetup(M1_IN2_PIN, freq, resolution);
    ledcAttachPin(M1_IN2_PIN, M1_IN2_PIN);

    // 設置馬達 B 的 PWM 輸出
    ledcSetup(M2_IN1_PIN, freq, resolution);
    ledcAttachPin(M2_IN1_PIN, M2_IN1_PIN);
    ledcSetup(M2_IN2_PIN, freq, resolution);
    ledcAttachPin(M2_IN2_PIN, M2_IN2_PIN);

    // 確保馬達啟動時是停止的
    setMotorSpeed(M1_IN1_PIN, M1_IN2_PIN, 0);
    setMotorSpeed(M2_IN1_PIN, M2_IN2_PIN, 0);
    
    // ----------------------------------------------------
    // 2. 初始化 BLE
    // ----------------------------------------------------
    // 初始化 BLE 裝置並設定名稱
    BLEDevice::init("ESP32C3-Motor-BLE"); 

    // 建立 BLE 伺服器
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // 建立 BLE 服務
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // 建立 BLE 特性 (Characteristic)
    pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

    // 設定特性寫入時的回調函數
    pCharacteristic->setCallbacks(new MyCallbacks());

    // 啟動服務
    pService->start();

    // 開始廣播 (讓其他裝置可以發現 ESP32)
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // 幫助快速連線
    BLEDevice::startAdvertising();
    Serial.println("BLE Advertising started. Device is discoverable.");
}

void loop() {
    // 檢查是否有客戶端連線
    if (deviceConnected) {
        // 在連線狀態下，可以執行其他任務或保持待機
        delay(100); 
    } else {
        // 斷開連線時，程式碼會進入 MyServerCallbacks::onDisconnect 重新廣播
        delay(500);
    }
}

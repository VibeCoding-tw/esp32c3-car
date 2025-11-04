#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <esp_ota_ops.h>      // For OTA partition functions
#include <esp_partition.h>    // For finding partitions
#include "gpio_pins.h"

// === 全域設定與連線狀態 ===
// OTA & Web Services
WebSocketsServer webSocket(81);
AsyncWebServer server(80);

// LEDC Channel for PWM
const int CH_A_FWD = 0;
const int CH_A_REV = 1;
const int CH_B_LEFT = 2;
const int CH_B_RIGHT = 3;
const int PWM_FREQ = 20000; // 20 kHz
const int PWM_RES = 8;      // 8-bit, 0-255 duty cycle

// 馬達控制變數
const int MAX_DUTY = 255; // 最大 PWM Duty Cycle (0~255)
// targetA/B 現在將儲存縮放後的 Duty Cycle 值 (0~MAX_DUTY)
volatile int targetA = 0; 
volatile int targetB = 0;
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 300; // 300ms 沒收到命令則停止

// === 應用程式模式 ===
enum DriveMode { AUTO, MANUAL };
DriveMode currentMode = MANUAL;

// ----------------------------------------------------------------------
// I. 遠端日誌 (Remote Logging)
// ----------------------------------------------------------------------

// 提供統一的日誌輸出通道 (Serial & WebSocket)
void sendLogMessage(const String& message) {
  Serial.println(message);
  // 將日誌訊息廣播給所有已連線的瀏覽器客戶端
  // 瀏覽器端的 JavaScript 會將此訊息輸出到 Console
  webSocket.broadcastTXT(message.c_str(), message.length());
}

// ----------------------------------------------------------------------
// II. 連線失敗機制 (Connection Fallback)
// ----------------------------------------------------------------------

// 連線超時時，跳轉回 Factory 分區的 Launcher App
void jumpToFactory() {
  sendLogMessage("--- WiFi connection failed. JUMPING TO FACTORY PARTITION (Launcher App) ---");

  // 1. 尋找 Factory 分區
  const esp_partition_t* factory = esp_partition_find_first(
      ESP_PARTITION_TYPE_APP,
      ESP_PARTITION_SUBTYPE_APP_FACTORY,
      NULL);
  
  if (factory != NULL) {
      // 2. 設定 Factory 分區為下一次啟動的目標
      esp_err_t err = esp_ota_set_boot_partition(factory);
      if (err == ESP_OK) {
          sendLogMessage("Successfully set Factory partition as next boot target. Rebooting...");
          delay(500); 
          ESP.restart(); // 重新啟動
      } else {
          sendLogMessage("Error setting boot partition! (" + String(esp_err_to_name(err)) + ") Rebooting anyway...");
          delay(2000);
          ESP.restart();
      }
  } else {
      sendLogMessage("FATAL: Factory partition not found! Rebooting...");
      delay(2000);
      ESP.restart();
  }
}

// ----------------------------------------------------------------------
// III. 網路連線 (Network Connection)
// ----------------------------------------------------------------------

// 嘗試連線到由 Launcher App 儲存的 Wi-Fi 網路
void connectToWiFi() {
  const unsigned long CONNECT_TIMEOUT_MS = 15000; // 15秒超時
  unsigned long connectStart = millis();

  sendLogMessage("Setting WiFi mode to Station and connecting with stored credentials...");
  WiFi.mode(WIFI_STA);
  // WiFi.begin() 會使用 NVS 中儲存的憑證 (由 Launcher App 配網成功後儲存)
  WiFi.begin();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    
    if (millis() - connectStart > CONNECT_TIMEOUT_MS) {
      sendLogMessage("WiFi connection timed out.");
      jumpToFactory(); // 超時，回退到 Launcher App
      return; 
    }
    
    // 輸出等待日誌
    if (WiFi.status() == WL_DISCONNECTED) {
        Serial.println("...Waiting for WiFi connection (Status: Disconnected)");
    } else {
        Serial.println("...Waiting for WiFi connection (Status: " + String(WiFi.status()) + ")");
    }
  }
  
  // 連線成功
  sendLogMessage("WiFi Connected! IP Address: " + WiFi.localIP().toString());
}


// ----------------------------------------------------------------------
// IV. 網路事件處理 (Network Event Handling)
// ----------------------------------------------------------------------

void emergencyStopNow() {
  targetA = targetB = 0;
  // immediately stop PWM and disable STBY pin
  digitalWrite(motor_stby, LOW);
  ledcWrite(CH_A_FWD, 0); ledcWrite(CH_A_REV, 0);
  ledcWrite(CH_B_LEFT, 0); ledcWrite(CH_B_RIGHT, 0);
  sendLogMessage("!!! EMERGENCY STOP Triggered !!!");
}

// 處理來自 WebSocket 客戶端的命令 (單字元或 JSON)
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: { 
        IPAddress ip = webSocket.remoteIP(num);
        sendLogMessage("--- WS Client Connected from " + ip.toString() + " ---");
      }
      break;
    case WStype_DISCONNECTED:
      sendLogMessage("--- WS Client Disconnected ---");
      // 斷線時立即停止馬達
      emergencyStopNow();
      break;
    case WStype_TEXT:
      {
        String msg = String((char*)payload);
        // V. 命令解析 (Command Parsing) - 單字元命令
        if (msg.length() == 1) {
          char cmd = msg.charAt(0);
          switch (cmd) {
            case 'A': 
              currentMode = AUTO; 
              sendLogMessage("Mode Switched: AUTO"); 
              break;
            case 'M': 
              currentMode = MANUAL; 
              sendLogMessage("Mode Switched: MANUAL"); 
              break;
            case 'S':
              emergencyStopNow();
              break;
          }
          lastCommandTime = millis(); 
        } else {
          // V. 命令解析 (Command Parsing) - JSON 遙控命令
          JsonDocument doc;
          DeserializationError err = deserializeJson(doc, (const char*)payload); 
          if (!err) {
            int steer = doc["steer"] | 0;    // 搖桿輸入 (-100 ~ 100)
            int throttle = doc["throttle"] | 0; // 搖桿輸入 (-100 ~ 100)
            
            // VI. 馬達控制 (Motor Control)
            if (currentMode == MANUAL) {
                
                // --- [修復 MAX_DUTY 問題] ---
                // 將搖桿輸入 (-100~100) 縮放至 Duty Cycle 範圍 (-MAX_DUTY~MAX_DUTY)
                // 例如：搖桿 100 應對應 MAX_DUTY 200
                int scaledThrottle = (throttle * MAX_DUTY) / 100;
                int scaledSteer = (steer * MAX_DUTY) / 100;

                // 更新目標速度 (targetA/B 現在儲存的是實際的 duty cycle)
                targetA = constrain(scaledThrottle, -MAX_DUTY, MAX_DUTY); // 前後
                targetB = constrain(scaledSteer, -MAX_DUTY, MAX_DUTY);    // 左右
                
                // 立即應用速度
                digitalWrite(motor_stby, HIGH);
                
                // 將 Duty Cycle 寫入 PWM 通道
                int speedA = targetA;
                int speedB = targetB;

                if (speedA > 0) { ledcWrite(CH_A_FWD, speedA); ledcWrite(CH_A_REV, 0); } 
                else if (speedA < 0) { ledcWrite(CH_A_FWD, 0); ledcWrite(CH_A_REV, abs(speedA)); } 
                else { ledcWrite(CH_A_FWD, 0); ledcWrite(CH_A_REV, 0); }

                if (speedB > 0) { ledcWrite(CH_B_RIGHT, speedB); ledcWrite(CH_B_LEFT, 0); } 
                else if (speedB < 0) { ledcWrite(CH_B_RIGHT, 0); ledcWrite(CH_B_LEFT, abs(speedB)); } 
                else { ledcWrite(CH_B_RIGHT, 0); ledcWrite(CH_B_LEFT, 0); }
                // -----------------------------

                // Reset timeout on every joystick command
                lastCommandTime = millis(); 
            }

            // 發送實時狀態回瀏覽器 (Console log)
            JsonDocument status;
            status["motorA"] = targetA;
            status["motorB"] = targetB;
            // 優化 debug 訊息，顯示原始輸入與實際 Duty Cycle
            status["debug"] = String("JSTK_Raw:") + throttle + "/" + steer + " | DutyA:" + targetA + "/DutyB:" + targetB + " | Mode:" + String(currentMode == AUTO ? "AUTO" : "MANUAL");
            
            size_t json_len = measureJson(status);
            char buffer[json_len + 1];
            size_t len = serializeJson(status, buffer, json_len + 1);
            webSocket.broadcastTXT(buffer, len);
            
          } else {
            sendLogMessage("WS Error: JSON parse failed: " + String(err.c_str()));
          }
        }
      }
      break;
    default:
      break;
  }
}

// ----------------------------------------------------------------------
// VII. 網頁服務 (Web Services)
// ----------------------------------------------------------------------

// 網頁前端 HTML 內容 (日誌已移至 Console)
const char index_html[] = R"rawliteral(
<!doctype html>
<html lang="zh-TW">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>ESP32 Car Remote Control (User App)</title>
  <style>
    :root{--bg:#0b0d11;--card:#0f1720;--accent:#3b82f6;--muted:#98a2b3}
    html,body{height:100%;margin:0;background:linear-gradient(180deg,var(--bg),#071022);color:#e6eef6;font-family:Inter,system-ui,Segoe UI,Roboto,"Noto Sans TC",sans-serif}
    .app{display:grid;grid-template-columns:1fr;grid-template-rows:1fr;height:100vh;padding:12px;box-sizing:border-box;position:relative}
    .viewer{background:rgba(255,255,255,0.02);border-radius:12px;padding:0;position:relative;overflow:hidden;}
    .videoFrame{width:100%;height:100%;object-fit:cover;background:#000}
    .overlay{position:absolute;left:12px;top:12px;background:rgba(0,0,0,0.45);padding:6px 8px;border-radius:8px;font-size:13px;color:var(--muted);z-index:5}
    .controls{position:absolute;top:0;left:0;width:100%;height:100%;display:flex;justify-content:space-between;align-items:flex-end;pointer-events:none}
    .stick{width:120px;height:120px;border-radius:50%;background:rgba(255,255,255,0.15);display:grid;place-items:center;position:relative;pointer-events:auto; touch-action: none;}
    .base{width:70px;height:70px;border-radius:50%;background:rgba(255,255,255,0.05);border:2px dashed rgba(255,255,255,0.03);display:grid;place-items:center}
    .knob{width:40px;height:40px;border-radius:50%;background:linear-gradient(180deg,#fff,#cbd5e1);transform:translate(-50%,-50%);position:absolute;left:50%;top:50%;box-shadow:0 6px 18px rgba(2,6,23,0.6)}
    .value{font-size:12px;color:var(--muted);text-align:center;margin-top:4px}
    
    /* 新增: 主導控制輸入顯示樣式 */
    .dominant-display {
        position: absolute;
        top: 50%;
        left: 50%;
        transform: translate(-50%, -50%);
        background: rgba(15, 23, 32, 0.95); /* Semi-transparent card background */
        padding: 20px 30px;
        border-radius: 12px;
        box-shadow: 0 4px 15px rgba(0, 0, 0, 0.5);
        z-index: 10;
        transition: opacity 0.3s ease-in-out, visibility 0.3s;
        opacity: 0; /* 預設隱藏 */
        pointer-events: none; /* 讓搖桿可以點擊穿透 */
        display: flex;
        align-items: center;
        gap: 20px;
    }
    .dominant-name { 
        font-size: 1.2rem; 
        color: #cbd5e1;
        font-weight: 500;
        min-width: 80px; /* 確保名稱區域穩定 */
        text-align: left;
    }
    .dominant-value { 
        font-size: 2.5rem; 
        font-weight: 800; 
        min-width: 120px; /* 確保數值區域穩定 */
        text-align: right; 
        font-variant-numeric: tabular-nums;
        transition: color 0.3s;
    }
    /* 顏色反饋 */
    .c-fwd { color: #22c55e; } /* 綠色 (前進) */
    .c-rev { color: #f97316; } /* 橙色 (倒車) */
    .c-left { color: #ef4444; } /* 紅色 (左轉) */
    .c-right { color: #3b82f6; } /* 藍色 (右轉) */

  </style>
</head>
<body>
  <div class="app">
    <div class="viewer">
      <img id="video" class="videoFrame" alt="遠端影像" src="" />
      <div class="overlay">IP: <span id="imgSource">N/A</span> | WS: <span id="wsStatus">未連線</span></div>
      
      <!-- 新增: 主導控制輸入顯示 (Dominant Input Display) -->
      <div id="dominant-display" class="dominant-display">
          <span id="domName" class="dominant-name"></span>
          <span id="domValue" class="dominant-value"></span>
      </div>

      <div class="controls">
        <div style="margin:12px; display:flex; flex-direction:column; gap:8px;">
          <div class="stick" id="stickLeft" data-role="steer"><div class="base"></div><div class="knob" id="knobLeft"></div></div>
          <div class="value">方向: <span id="valSteer">0</span></div>
        </div>
        <div style="margin:12px; display:flex; flex-direction:column; gap:8px;">
          <div class="stick" id="stickRight" data-role="throttle"><div class="base"></div><div class="knob" id="knobRight"></div></div>
          <div class="value">油門: <span id="valThrottle">0</span></div>
        </div>
      </div>
    </div>
  </div>

  <script>
    class VirtualStick {
      constructor(stickEl, knobEl, onChange){
        this.el = stickEl; this.knob = knobEl; this.cb = onChange; this.max = Math.min(stickEl.clientWidth, stickEl.clientHeight)/2 - 8;
        this.center = {x: this.el.clientWidth/2, y: this.el.clientHeight/2};
        this.pointerId = null; this.pos = {x:0,y:0}; this.deadzone = 6;
        this._bind();
      }
      _bind(){
        this.el.style.touchAction = 'none';
        this.el.addEventListener('pointerdown', e=>this._start(e));
        window.addEventListener('pointermove', e=>this._move(e));
        window.addEventListener('pointerup', e=>this._end(e));
        window.addEventListener('pointercancel', e=>this._end(e));
        window.addEventListener('resize', ()=>{this.center = {x:this.el.clientWidth/2,y:this.el.clientHeight/2};this.max = Math.min(this.el.clientWidth,this.el.clientHeight)/2 - 8});
      }
      _start(e){ if(this.pointerId!==null) return; this.pointerId = e.pointerId; this.el.setPointerCapture?.(e.pointerId); this._move(e); }
      _move(e){ if(this.pointerId===null || e.pointerId!==this.pointerId) return; const rect = this.el.getBoundingClientRect(); let x = e.clientX - rect.left - rect.width/2; let y = e.clientY - rect.top - rect.height/2; const d = Math.hypot(x,y); if(d>this.max){ const r = this.max/d; x*=r; y*=r; } this.pos = {x,y}; this.knob.style.left = (50 + (x/rect.width*100))+'%'; this.knob.style.top = (50 + (y/rect.height*100))+'%'; this._fire(); }
      _end(e){ if(this.pointerId===null || e.pointerId!==e.pointerId) return; this.pointerId=null; this.pos={x:0,y:0}; this.knob.style.left='50%'; this.knob.style.top='50%'; this._fire(); }
      // 此處 n.x, n.y 介於 -1 到 1 之間
      _fire(){ const norm = {x: Math.abs(this.pos.x) < this.deadzone ? 0 : this.pos.x/this.max, y: Math.abs(this.pos.y) < this.deadzone ? 0 : this.pos.y/this.max}; if(this.cb) this.cb(norm); }
    }

    const wsStatusEl = document.getElementById('wsStatus');
    const valSteer = document.getElementById('valSteer');
    const valThrottle = document.getElementById('valThrottle');
    const stickL = document.getElementById('stickLeft');
    const stickR = document.getElementById('stickRight');
    
    // 取得新的主導顯示元素
    const domDisplayEl = document.getElementById('dominant-display');
    const domNameEl = document.getElementById('domName');
    const domValueEl = document.getElementById('domValue');


    // config.sendRate: 50ms (20Hz)
    const state = {steer:0, throttle:0, ws:null, sendInterval:null, videoInterval:null, config:{videoUrl:'',videoFps:10,wsUrl:'',sendRate:50}};

    // n.x*100 或 -n.y*100 確保輸出在 -100 到 100 之間
    const left = new VirtualStick(stickL, document.getElementById('knobLeft'), n=>{ 
        state.steer = Math.round(n.x*100); 
        valSteer.textContent=state.steer; 
        updateDominantDisplay(); // 搖桿移動時也立即更新顯示
    });
    const right = new VirtualStick(stickR, document.getElementById('knobRight'), n=>{ 
        state.throttle = Math.round(-n.y*100); 
        valThrottle.textContent=state.throttle; 
        updateDominantDisplay(); // 搖桿移動時也立即更新顯示
    });
    
    // --- 新增: 更新主導控制輸入顯示 ---
    function updateDominantDisplay() {
        const steer = state.steer;
        const throttle = state.throttle;
        const absSteer = Math.abs(steer);
        const absThrottle = Math.abs(throttle);

        // 重設數值顏色類別
        domValueEl.className = 'dominant-value';

        if (absSteer === 0 && absThrottle === 0) {
            // 隱藏顯示 (不操作時)
            domDisplayEl.style.opacity = '0';
            return;
        }

        // 顯示面板
        domDisplayEl.style.opacity = '1';

        let name = '';
        let value = 0;
        let colorClass = '';

        if (absSteer >= absThrottle) {
            // 轉向 (Steer) 為主導 (或兩者相等，優先顯示 Steer)
            value = steer;
            if (value > 0) {
                name = '右轉 (STEER)';
                colorClass = 'c-right';
            } else if (value < 0) {
                name = '左轉 (STEER)';
                colorClass = 'c-left';
            } else {
                // 如果 Steer=0 且 Throttle!=0, 則轉向顯示 Throttle
                if (absThrottle > 0) {
                    value = throttle;
                    if (value > 0) { name = '前進 (THROTTLE)'; colorClass = 'c-fwd'; }
                    else { name = '倒車 (REVERSE)'; colorClass = 'c-rev'; }
                } else {
                    // 兩者皆為 0，但在開頭已處理
                    name = '靜止 (IDLE)';
                    colorClass = '';
                }
            }
        } 
        
        if (absThrottle > absSteer) {
            // 油門 (Throttle) 為主導
            value = throttle;
            if (value > 0) {
                name = '前進 (THROTTLE)';
                colorClass = 'c-fwd';
            } else {
                name = '倒車 (REVERSE)';
                colorClass = 'c-rev';
            }
        }
        
        domNameEl.textContent = name;
        domValueEl.textContent = `${Math.abs(value)}%`;
        
        if (colorClass) {
            domValueEl.classList.add(colorClass);
        }
    }
    // ----------------------

    // --- 日誌輔助函式: 輸出到瀏覽器 Console ---
    function appendLog(message) {
        const timestamp = new Date().toLocaleTimeString('en-US', {hour12: false});
        console.log(`[ESP32 LOG] [${timestamp}] ${message}`); 
    }
    // ----------------------

    function connectWs(){ 
        if(state.ws){ try{state.ws.close()}catch(e){} state.ws=null; } 
        const wsUrl = `ws://${window.location.hostname}:81`;
        
        appendLog(`嘗試連線到 WebSocket: ${wsUrl}`);
        wsStatusEl.textContent = 'Connecting...';

        try{ 
            state.ws = new WebSocket(wsUrl); 
            state.ws.binaryType='arraybuffer'; 
            
            state.ws.onopen=()=>{
                wsStatusEl.textContent = 'OPEN';
                appendLog('WebSocket 連線成功。');
            }; 
            
            state.ws.onclose=()=>{
                wsStatusEl.textContent = 'CLOSED';
                appendLog('WebSocket 已斷線，3秒後重試連線...');
                setTimeout(connectWs, 3000); // 重試連線
            }; 
            
            state.ws.onerror=()=>{
                wsStatusEl.textContent = 'ERROR';
                appendLog('WebSocket 連線錯誤。');
            }; 
            
            state.ws.onmessage = (event) => {
                const data = event.data;
                
                // 嘗試解析 JSON (控制狀態/遠端日誌)
                try {
                    const json = JSON.parse(data);
                    if (json.debug) {
                        // 這是來自 ESP32 的遠端日誌 (JSON 格式)
                        appendLog(json.debug);
                    } else if (json.motorA !== undefined) {
                        // 馬達狀態更新 (可選)
                        // appendLog(`Motor A:${json.motorA}, B:${json.motorB}`);
                    }
                } catch(e) {
                    // 如果不是 JSON，則視為遠端日誌文本
                    appendLog(data);
                }
            };
        }catch(e){ 
            wsStatusEl.textContent = 'ERROR'; 
            appendLog(`WebSocket 建立失敗: ${e.message}`);
        } 
    }

    // 發送搖桿命令到 WebSocket
    function startSending(rate){ 
      if(state.sendInterval) clearInterval(state.sendInterval); 
      state.sendInterval=setInterval(()=>{ 
        
        // **優化: 如果搖桿處於中心位置 (0, 0)，則不發送消息**
        if(state.steer === 0 && state.throttle === 0) {
            updateDominantDisplay(); // 確保歸零時顯示隱藏
            return; // 停止執行，不發送 WS 消息
        }
        
        if(state.ws && state.ws.readyState===WebSocket.OPEN){ 
          // t: timestamp, steer: 轉向 (-100 to 100), throttle: 油門 (-100 to 100)
          // 這裡發送的是原始的 -100 ~ 100 搖桿百分比
          state.ws.send(JSON.stringify({t:Date.now(),steer:state.steer,throttle:state.throttle})); 
          
          // 定時器觸發時也更新顯示，確保顯示與發送同步
          updateDominantDisplay();
        } 
      }, rate); 
    }
    
    // 影像輪詢邏輯 (僅為範例，需配合 ESP32 影像串流伺服器)
    async function fetchFrame(){ const url=state.config.videoUrl; if(!url) return; try{ const res=await fetch(url+(url.includes('?')?'&':'?')+'t='+Date.now(),{cache:'no-store'}); if(!res.ok) throw new Error('bad'); const blob=await res.blob(); const img=document.getElementById('video'); const old=img.src; img.src=URL.createObjectURL(blob); if(old&&old.startsWith('blob:')) URL.revokeObjectURL(old); }catch(e){ console.warn(e); } }
    function startVideoPoll(){ stopVideoPoll(); const fps=Math.max(1,parseInt(state.config.videoFps||10)); state.videoInterval=setInterval(fetchFrame, Math.round(1000/fps)); document.getElementById('imgSource').textContent=state.config.videoUrl||'N/A'; }
    function stopVideoPoll(){ if(state.videoInterval) clearInterval(state.videoInterval); state.videoInterval=null; }

    window.addEventListener('beforeunload', ()=>{ if(state.ws) state.ws.close(); stopSending(); stopVideoPoll(); });
    
    window.onload = () => {
        connectWs();
        startSending(50); // 每 50ms 發送一次控制命令
        updateDominantDisplay(); // 初始檢查並隱藏顯示
        // 設定影像串流 URL 範例 (如果您的 ESP32 提供影像串流)
        // state.config.videoUrl = 'http://' + window.location.hostname + '/stream';
        // startVideoPoll(); 
    };
  </script>
</body>
</html>
)rawliteral";

// 設置 HTTP Server 和 WebSocket
void setupWebServer() {
  String hostname = "esp32c3-" + String(WiFi.macAddress());
  hostname.replace(":", ""); // remove colons for clean name
  
  if (MDNS.begin(hostname.c_str())) {
    Serial.printf("mDNS responder started: %s.local\n", hostname.c_str());
  } else {
    sendLogMessage("Error setting up mDNS!");
  }

  // Handle favicon.ico request (防止 404 錯誤)
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(204); 
  });

  // 根目錄提供遙控網頁
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", index_html);
  });

  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  sendLogMessage("Web UI Ready on port 80. Remote Control Active at http://" + WiFi.localIP().toString());
}

// ----------------------------------------------------------------------
// VIII. OTA 服務 (Over-The-Air Update)
// ----------------------------------------------------------------------

void setupOTA() {
  //String hostname = "esp32car-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  String hostname = "esp32c3-" + String(WiFi.macAddress());
  hostname.replace(":", ""); // remove colons for clean name

  // 設定 OTA 參數
  ArduinoOTA.setHostname(hostname.c_str());
  ArduinoOTA.setPassword("mysecurepassword"); // 替換為您的密碼
  
  // OTA 事件處理
  ArduinoOTA.onStart([]() { sendLogMessage("OTA: Start updating " + String(ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "filesystem")); });
  ArduinoOTA.onEnd([]() { sendLogMessage("OTA: Update Finished. Rebooting..."); });
  ArduinoOTA.onError([](ota_error_t error) { sendLogMessage("OTA Error: " + String(error)); });

  ArduinoOTA.begin();
  sendLogMessage("OTA Ready. Hostname: " + hostname + ".local");
}

// ----------------------------------------------------------------------
// IX. 馬達初始化 (Motor Initialization)
// ----------------------------------------------------------------------

void setupPWM() {
  // 設置 LEDC 通道頻率與解析度
  ledcSetup(CH_A_FWD, PWM_FREQ, PWM_RES);
  ledcSetup(CH_A_REV, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B_LEFT, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B_RIGHT, PWM_FREQ, PWM_RES);

  // 將 LEDC 通道連接到 GPIO 引腳
  ledcAttachPin(motorA_pwm_fwd, CH_A_FWD);
  ledcAttachPin(motorA_pwm_rev, CH_A_REV);
  ledcAttachPin(motorB_pwm_left, CH_B_LEFT);
  ledcAttachPin(motorB_pwm_right, CH_B_RIGHT);
}

// ----------------------------------------------------------------------
// 程式進入點
// ----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(100);

  // X. 馬達開關 (Motor Enable)
  pinMode(motor_stby, OUTPUT);
  digitalWrite(motor_stby, LOW); // 預設禁用馬達

  // I. 馬達初始化 (Motor Initialization)
  setupPWM();

  // II. 網路連線 (Network Connection) - 需有 Launcher App 儲存的憑證
  connectToWiFi();
  
  // III. OTA 服務 (Over-The-Air Update)
  setupOTA();

  // IV. 網頁服務 (Web Services)
  setupWebServer();
  
  sendLogMessage("User App setup complete. Ready to receive commands.");
}


void loop() {
  // 保持 OTA 服務運行
  ArduinoOTA.handle();
  // 保持 WebSocket 服務運行
  webSocket.loop();
  
  // 馬達命令超時邏輯：若超過 COMMAND_TIMEOUT 且馬達正在運行，則停止所有馬達
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    // targetA/B 儲存的是 Duty Cycle，所以只要不為 0，就表示馬達正在轉動
    if (targetA != 0 || targetB != 0) { 
      sendLogMessage("Motors stopped due to command timeout.");
      targetA = targetB = 0; // 重設目標速度
      digitalWrite(motor_stby, LOW); // 禁用馬達
      // 確保 PWM 也停止
      ledcWrite(CH_A_FWD, 0); ledcWrite(CH_A_REV, 0);
      ledcWrite(CH_B_LEFT, 0); ledcWrite(CH_B_RIGHT, 0);
    }
  }

  // 心跳日誌
  static unsigned long lastLogMillis = 0;
  if (millis() - lastLogMillis > 5000) { 
    sendLogMessage("Heartbeat: Car system active, Mode=" + String(currentMode == AUTO ? "AUTO" : "MANUAL"));
    lastLogMillis = millis();
  }
}

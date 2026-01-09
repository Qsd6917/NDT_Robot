#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>

// ================= 1. 硬件引脚定义 (通用GPIO编号) =================
#define PIN_M0  16   // 对应 NodeMCU 的 D0
#define PIN_M1  5    // 对应 NodeMCU 的 D1
#define PIN_AUX 4    // 对应 NodeMCU 的 D2

// ================= 2. 配置参数 =================
const char* ssid = "MagClimb_Robot_WIFI";  // 机器人热点名称
const char* password = "";                 // 热点密码（留空则不设密码）
const int SERIAL_BAUD = 9600;              // E48 默认波特率
const long SEND_INTERVAL = 100;            // 自动发送频率 (10Hz)

// ================= 3. 全局变量 =================
ESP8266WebServer server(80);

int currentSpeed = 128; // 0-255, 128为静止
int modeState = 0;      // 0: 直行(0°), 1: 转向(15°)
int liftState = 0;      // 0: 关, 1: 开
unsigned long lastSendTime = 0;

// ================= 4. 网页控制面板 (HTML) =================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=0">
  <meta charset="UTF-8">
  <title>NDT Robot Control</title>
  <style>
    body { font-family: Arial; text-align: center; background: #222; color: white; margin:0; padding:20px; user-select:none; }
    h2 { color: #007bff; }
    .slider { width: 90%; height: 60px; margin: 30px 0; }
    .btn { width: 45%; height: 60px; font-size: 18px; margin: 5px; border-radius: 10px; border: none; cursor: pointer; color: white; }
    .btn-mode { background: #28a745; }
    .btn-lift { background: #dc3545; }
    .status { background: #000; padding: 15px; border-radius: 8px; font-family: monospace; color: #0f0; margin-top: 20px; font-size: 14px; word-break: break-all; }
    .label { font-size: 14px; color: #aaa; margin-top: 10px; }
  </style>
</head>
<body>
  <h2>机器人网页遥控器</h2>
  
  <p>速度控制 (松手归位)</p>
  <input type="range" min="0" max="255" value="128" class="slider" id="spd" oninput="update()">
  
  <div style="display: flex; justify-content: center;">
    <button class="btn btn-mode" id="mBtn" onclick="toggleMode()">模式: 直行</button>
    <button class="btn btn-lift" id="lBtn" onclick="toggleLift()">升降: 关</button>
  </div>

  <div class="label">实时发送 HEX 指令:</div>
  <div class="status" id="hex">AA 80 80 80 80 00 00 00 00 00 AA 55</div>

<script>
  let mode = 0; let lift = 0;
  function update() {
    let s = document.getElementById('spd').value;
    fetch(`/ctrl?s=${s}&m=${mode}&l=${lift}`).then(r => r.text()).then(t => {
      document.getElementById('hex').innerText = t;
    });
  }
  function toggleMode() { 
    mode = (mode == 0) ? 1 : 0; 
    document.getElementById('mBtn').innerText = mode ? "模式: 转向" : "模式: 直行";
    update(); 
  }
  function toggleLift() { 
    lift = (lift == 0) ? 1 : 0; 
    document.getElementById('lBtn').innerText = lift ? "升降: 开" : "升降: 关";
    update(); 
  }
  // 滑块松手归位逻辑
  const sld = document.getElementById('spd');
  const reset = () => { sld.value = 128; update(); };
  sld.onmouseup = reset;
  sld.ontouchend = reset;
</script>
</body>
</html>
)rawliteral";

// ================= 5. HEX 协议打包与发送 =================
String sendHexPacket(bool doSerial) {
  uint8_t buf[12];
  buf[0] = 0xAA; // 帧头
  
  // 4字节速度
  buf[1] = buf[2] = buf[3] = buf[4] = (uint8_t)currentSpeed;
  
  // 4字节角度 (0或15)
  uint8_t ang = (modeState == 0) ? 0 : 15;
  buf[5] = buf[6] = buf[7] = buf[8] = ang;
  
  // 1字节升降
  buf[9] = (uint8_t)liftState;
  
  // 1字节 XOR 校验 (校验 0-9 字节)
  uint8_t xor_sum = 0;
  for(int i=0; i<10; i++) xor_sum ^= buf[i];
  buf[10] = xor_sum;
  
  buf[11] = 0x55; // 帧尾

  // 串口发送给 E48
  if (doSerial) {
    Serial.write(buf, 12);
  }

  // 转换为网页显示的字符串
  String out = "";
  for(int i=0; i<12; i++) {
    if(buf[i] < 0x10) out += "0";
    out += String(buf[i], HEX) + " ";
  }
  out.toUpperCase(); // 修正后的写法
  return out;
}

// ================= 6. Web Server 路由 =================
void handleRoot() {
  server.send(200, "text/html", index_html);
}

void handleControl() {
  if (server.hasArg("s")) currentSpeed = server.arg("s").toInt();
  if (server.hasArg("m")) modeState = server.arg("m").toInt();
  if (server.hasArg("l")) liftState = server.arg("l").toInt();
  
  // 只返回字符串，不重复触发 Serial.write (由 loop 中的定时器负责发送)
  String hexStr = sendHexPacket(false);
  server.send(200, "text/plain", hexStr);
}

// ================= 7. 初始化与主循环 =================
void setup() {
  // 初始化串口
  Serial.begin(SERIAL_BAUD);
  
  // 设置 E48 进入模式 0 (透传)
  pinMode(PIN_M0, OUTPUT);
  pinMode(PIN_M1, OUTPUT);
  digitalWrite(PIN_M0, LOW);
  digitalWrite(PIN_M1, LOW);

  // 启动 WiFi 热点
  WiFi.softAP(ssid, password);

  // 启动 OTA 无线升级服务
  ArduinoOTA.setHostname("NDT-Robot-System");
  ArduinoOTA.setPassword("admin"); // OTA 升级密码
  ArduinoOTA.begin();

  // 绑定路由
  server.on("/", handleRoot);
  server.on("/ctrl", handleControl);
  server.begin();
}

void loop() {
  // 处理 OTA 请求
  ArduinoOTA.handle();
  
  // 处理网页请求
  server.handleClient();

  // 定时发送控制包 (10Hz)
  unsigned long currentMillis = millis();
  if (currentMillis - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentMillis;
    sendHexPacket(true);
  }
}
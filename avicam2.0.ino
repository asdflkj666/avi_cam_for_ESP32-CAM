#include "esp_camera.h"
#include "WiFi.h"
#include "WebServer.h"
#include "ESPmDNS.h"
#include "FS.h"
#include "SD_MMC.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"

// 函数声明
void printMemoryInfo();
void printCameraInfo();
void startRecording();
void stopRecording();
void captureFrame();
void writeAVIHeader();
void updateAVIHeader();
void handleRoot();
void handleDownload();
void handleList();
void handleDelete();
void handleStatus();
void handleStartRecording();
void handleStopRecording();
void handleStream();
String listVideos();

// 选择摄像头型号
#define CAMERA_MODEL_AI_THINKER

// 摄像头引脚定义
#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#endif

// 录制控制引脚 (使用GPIO13作为触发引脚)
#define RECORD_TRIGGER_PIN 13
// ESP32-CAM板载LED引脚 (GPIO4) - 更改为GPIO2避免与摄像头引脚冲突
#define LED_PIN 2

// Web服务器
WebServer server(80);

// 录制状态变量
bool isRecording = false;
File videoFile;
unsigned long frameCount = 0;
unsigned long startTime = 0;
String currentVideoName = "";

// AVI文件结构
const int AVI_HEADER_SIZE = 512;
uint8_t aviHeader[AVI_HEADER_SIZE];

// 内存信息显示函数
void printMemoryInfo() {
  Serial.printf("堆内存 - 可用: %d bytes, 最小可用: %d bytes\n", ESP.getFreeHeap(), ESP.getMinFreeHeap());
  
  if(psramFound()) {
    Serial.printf("PSRAM - 总计: %d bytes, 可用: %d bytes\n", 
                  ESP.getPsramSize(), ESP.getFreePsram());
  } else {
    Serial.println("未检测到PSRAM");
  }
  
  Serial.printf("闪存芯片大小: %d bytes\n", ESP.getFlashChipSize());
  
  // 添加SD卡状态诊断信息
  Serial.printf("SD卡状态诊断:\n");
  Serial.printf("  SD_MMC.cardSize(): %llu bytes\n", SD_MMC.cardSize());
  Serial.printf("  SD_MMC.totalBytes(): %llu bytes\n", SD_MMC.totalBytes());
  Serial.printf("  SD_MMC.usedBytes(): %llu bytes\n", SD_MMC.usedBytes());
  Serial.printf("  SD_MMC.exists('/'):%s\n", SD_MMC.exists("/") ? "是" : "否");
}

// 摄像头信息显示函数
void printCameraInfo() {
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    Serial.printf("摄像头型号: 0x%x\n", s->id.PID);
    Serial.printf("分辨率: %d\n", s->status.framesize);
    Serial.printf("质量: %d\n", s->status.quality);
    Serial.printf("亮度: %d\n", s->status.brightness);
    Serial.printf("对比度: %d\n", s->status.contrast);
    Serial.printf("饱和度: %d\n", s->status.saturation);
    Serial.printf("特殊效果: %d\n", s->status.special_effect);
    Serial.printf("白平衡模式: %d\n", s->status.wb_mode);
    Serial.printf("自动白平衡增益: %s\n", s->status.awb_gain ? "开启" : "关闭");
    Serial.printf("自动曝光: %s\n", s->status.aec2 ? "开启" : "关闭");
    Serial.printf("自动增益: %d\n", s->status.agc_gain);
  } else {
    Serial.println("无法获取摄像头信息");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  
  // 显示系统启动信息和内存状态
  Serial.println("=== AVICAM 2.0 系统启动 ===");
  Serial.printf("启动时间: %lu ms\n", millis());
  printMemoryInfo();
  
  // 初始化触发引脚
pinMode(RECORD_TRIGGER_PIN, INPUT_PULLUP);
// 初始化LED引脚
pinMode(LED_PIN, OUTPUT);
digitalWrite(LED_PIN, HIGH); // 默认关闭LED (低电平点亮)
// 确保LED永远不被打开
  
  // 初始化摄像头
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;  // 添加grab_mode配置
  config.fb_location = CAMERA_FB_IN_PSRAM;  // 添加fb_location配置
  
  // OV2640摄像头配置优化
// 根据PSRAM存在与否调整配置
if(psramFound()){
Serial.println("检测到PSRAM，使用更高分辨率设置");
config.frame_size = FRAMESIZE_XGA;   // 1024x768 (ESP32-CAM有PSRAM时的最佳选择)
config.jpeg_quality = 8;             // 更高质量
config.fb_count = 2;
} else {
Serial.println("未检测到PSRAM，使用OV2640标准设置");
config.frame_size = FRAMESIZE_SVGA;  // 800x600 (ESP32-CAM无PSRAM时的推荐设置)
config.jpeg_quality = 10;            // 平衡质量和性能
config.fb_count = 2;                 // 增加缓冲区数量提高稳定性
config.fb_location = CAMERA_FB_IN_DRAM;  // 无PSRAM时使用DRAM
}
  
  Serial.printf("配置: 分辨率=%d, 质量=%d, 缓冲区数量=%d\n",
config.frame_size, config.jpeg_quality, config.fb_count);
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("摄像头初始化失败: 0x%x", err);
    Serial.println();
    // 添加更详细的错误信息
    switch(err) {
      case ESP_ERR_INVALID_ARG:
        Serial.println("错误原因: 无效参数");
        break;
      case ESP_ERR_NO_MEM:
        Serial.println("错误原因: 内存不足");
        break;
      case ESP_ERR_INVALID_STATE:
        Serial.println("错误原因: 状态无效");
        break;
      case ESP_ERR_NOT_FOUND:
        Serial.println("错误原因: 未找到设备");
        break;
      default:
        Serial.println("错误原因: 未知错误");
        break;
    }
    return;
  }
  
  Serial.println("摄像头初始化成功");
  printCameraInfo();
  
  // OV2640特定优化
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    // 如果是OV2640 (PID = 0x2640)
    if (s->id.PID == 0x2640) {
      Serial.println("检测到OV2640摄像头，应用特定优化");
      
      // 设置适当的亮度、对比度和饱和度
      s->set_brightness(s, 0);     // 亮度 (范围: -2 到 2)
      s->set_contrast(s, 0);       // 对比度 (范围: -2 到 2)
      s->set_saturation(s, 0);     // 饱和度 (范围: -2 到 2)
      
      // 设置特殊效果 (0:普通模式, 1:负片, 2:黑白, 3:红色, 4:绿色, 5:蓝色, 6:复古)
      s->set_special_effect(s, 0);
      
      // 设置白平衡 (0:自动, 1:晴天, 2:阴天, 3:办公室, 4:家庭)
      s->set_wb_mode(s, 0);
      
      // 关闭自动白平衡增益
      s->set_awb_gain(s, 1);
      
      // 设置自动曝光
      s->set_aec2(s, 0);
      
      // 设置自动增益
      s->set_agc_gain(s, 0);
      
      Serial.println("OV2640优化设置已应用");
    }
    
    // 类似于BasicVideo项目，降低初始帧大小以获得更高的初始帧率
    if(config.pixel_format == PIXFORMAT_JPEG) {
      s->set_framesize(s, FRAMESIZE_QVGA);
    }
    
    // 对于AI Thinker ESP32-CAM开发板，设置垂直翻转和水平镜像
#if defined(CAMERA_MODEL_AI_THINKER)
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
#endif
  }

  // 初始化SD卡
  Serial.println("开始初始化SD卡...");
  if(!SD_MMC.begin("/sdcard", true)){
    Serial.println("SD卡挂载失败");
    // 尝试重新初始化
    delay(100);
    if(!SD_MMC.begin("/sdcard", true)){
      Serial.println("SD卡重新挂载也失败");
      return;
    }
  }

  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("未找到SD卡");
    return;
  }

  Serial.println("SD卡初始化成功");
  
  // 显示SD卡信息
  Serial.print("SD卡类型: ");
  switch(cardType){
    case CARD_MMC:
      Serial.println("MMC");
      break;
    case CARD_SD:
      Serial.println("SDSC");
      break;
    case CARD_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("UNKNOWN");
      break;
  }
  
  uint64_t cardSize = SD_MMC.cardSize();
  uint64_t totalBytes = SD_MMC.totalBytes();
  
  Serial.printf("SD卡大小: %lluMB\n", cardSize / (1024 * 1024));
  Serial.printf("SD卡可用空间: %lluMB\n", totalBytes / (1024 * 1024));
  
  // 检查SD卡根目录是否存在
  if (!SD_MMC.exists("/")) {
    Serial.println("错误: SD卡根目录不存在");
    return;
  }
  
  // 检查SD卡是否可写
  File testFile = SD_MMC.open("/test.txt", FILE_WRITE);
  if(!testFile) {
    Serial.println("警告: SD卡可能不可写");
    // 尝试其他方式检查可写性
    File testFile2 = SD_MMC.open("/", FILE_WRITE);
    if (testFile2) {
      Serial.println("SD卡根目录可写");
      testFile2.close();
    } else {
      Serial.println("SD卡根目录也不可写");
    }
  } else {
    // 尝试写入测试数据
    if(testFile.println("SD卡写入测试")) {
      Serial.println("SD卡写入测试成功");
    } else {
      Serial.println("警告: SD卡写入测试失败");
    }
    testFile.close();
    // 删除测试文件
    SD_MMC.remove("/test.txt");
  }
  
  // 显示SD卡详细状态
  Serial.printf("SD卡总字节数: %llu\n", SD_MMC.totalBytes());
  Serial.printf("SD卡已使用字节数: %llu\n", SD_MMC.usedBytes());

  // 创建WiFi热点
  Serial.println("正在创建WiFi热点...");
  
  // 配置WiFi热点参数
  WiFi.disconnect(true);
  delay(100);
  
  boolean result = WiFi.softAP("esp32-cam", "12345678");
  
  if(!result) {
    Serial.println("WiFi热点创建失败!");
    printMemoryInfo();
    return;
  }
  
  // 等待WiFi热点完全启动
  delay(500);
  
  Serial.println("WiFi热点已创建");
  Serial.println("SSID: esp32-cam");
  Serial.println("密码: 12345678");
  Serial.print("IP地址: ");
  Serial.println(WiFi.softAPIP());
  
  // 显示WiFi连接信息
  Serial.printf("WiFi通道: %d\n", WiFi.channel());
  Serial.printf("已连接设备数: %d\n", WiFi.softAPgetStationNum());
  
  // 获取WiFi配置信息
  wifi_config_t conf;
  esp_err_t wifi_err = esp_wifi_get_config(WIFI_IF_AP, &conf);
  if (wifi_err == ESP_OK) {
    Serial.printf("最大连接数: %d\n", conf.ap.max_connection);
    Serial.printf("认证模式: %d\n", conf.ap.authmode);
    Serial.printf("SSID隐藏: %s\n", conf.ap.ssid_hidden ? "是" : "否");
  } else {
    Serial.printf("获取WiFi配置信息失败，错误代码: 0x%x\n", wifi_err);
  }

  // 设置Web服务器路由
  server.on("/", HTTP_GET, handleRoot);
  server.on("/download", HTTP_GET, handleDownload);
  server.on("/list", HTTP_GET, handleList);
  server.on("/delete", HTTP_GET, handleDelete);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/start", HTTP_GET, handleStartRecording);
  server.on("/stop", HTTP_GET, handleStopRecording);
  server.on("/stream", HTTP_GET, handleStream);

  server.begin();
  Serial.println("HTTP服务器已启动");
  Serial.println("系统就绪！使用GPIO13短接到GND来开始/停止录制");
}

void loop() {
  // 处理Web服务器请求
  server.handleClient();
  
  // 控制LED状态指示灯
  // 移除LED控制以提高性能
  
  // 每10秒打印一次系统状态信息
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint > 10000) {
    lastStatusPrint = millis();
    // 减少串口打印以提高性能
    // printMemoryInfo();
  }
  
  // 检查录制触发引脚状态
  static unsigned long lastTriggerCheck = 0;
  if (millis() - lastTriggerCheck > 100) {  // 每100ms检查一次触发引脚
    lastTriggerCheck = millis();
    
    int triggerState = digitalRead(RECORD_TRIGGER_PIN);
    static int lastTriggerState = HIGH;  // 记录上一次状态
    
    // 检测下降沿触发
    if (lastTriggerState == HIGH && triggerState == LOW) {
      if (isRecording) {
        stopRecording();
      } else {
        startRecording();
      }
    }
    
    lastTriggerState = triggerState;
  }
  
  // 如果正在录制，则捕获帧
  if (isRecording) {
    captureFrame();
    
    // 控制帧率，避免过快录制
    delay(50);  // 约20fps
  }
}

void startRecording() {
Serial.println("=== 开始录制准备 ===");
// 移除频繁的内存信息打印以提高性能
// printMemoryInfo();

// 不再控制LED指示灯
// digitalWrite(LED_PIN, LOW); // 注释掉LED控制代码

// 生成唯一的文件名
currentVideoName = "/video_" + String(millis()) + ".avi";
// 减少串口打印频率以提高性能
// Serial.printf("尝试创建视频文件: %s\n", currentVideoName.c_str());

// 快速检查SD卡是否可用
if (!SD_MMC.exists("/")) {
  Serial.println("错误: SD卡不可访问");
  isRecording = false;
  return;
}

// 直接使用FILE_WRITE模式打开文件，移除不必要的重试逻辑
videoFile = SD_MMC.open(currentVideoName, FILE_WRITE);
  
  if (!videoFile) {
    Serial.println("无法创建视频文件");
    Serial.printf("文件名: %s\n", currentVideoName.c_str());
    
    // 简化文件状态检查
    uint8_t cardType = SD_MMC.cardType();
    if(cardType == CARD_NONE){
      Serial.println("错误: SD卡未检测到");
    }
    
    // 检查SD卡根目录
    if (!SD_MMC.exists("/")) {
      Serial.println("SD卡根目录不存在");
    }
    
    isRecording = false;
    return;
  }
  
  // 简化文件状态检查
  if (!videoFile.availableForWrite()) {
    Serial.println("文件未正确打开用于写入");
    videoFile.close();
    isRecording = false;
    return;
  }
  
  // 简化成功信息打印
  // Serial.printf("视频文件创建成功: %s\n", currentVideoName.c_str());
  
  // 写入AVI文件头
  writeAVIHeader();
  
  // 简化文件头检查
  if (videoFile.size() < AVI_HEADER_SIZE) {
    Serial.println("AVI文件头写入失败");
    videoFile.close();
    SD_MMC.remove(currentVideoName);
    isRecording = false;
    return;
  }
  
  isRecording = true;
  frameCount = 0;
  startTime = millis();
  
  Serial.println("开始录制: " + currentVideoName);
}

void stopRecording() {
if (!isRecording) return;

Serial.println("=== 停止录制准备 ===");
printMemoryInfo();

// 不再控制LED指示灯
// digitalWrite(LED_PIN, HIGH); // 注释掉LED控制代码
// Serial.println("录制LED指示灯已关闭"); // 注释掉LED状态信息

isRecording = false;
  
  // 更新AVI文件头
  updateAVIHeader();
  
  // 检查文件是否有效再关闭
  if (videoFile) {
    videoFile.close();
    Serial.println("视频文件已关闭");
  } else {
    Serial.println("警告: 文件句柄已失效");
  }
  
  unsigned long duration = (millis() - startTime) / 1000;
  Serial.printf("停止录制。帧数: %lu, 时长: %lu秒\n", frameCount, duration);
  Serial.println("文件保存为: " + currentVideoName);
  
  // 检查文件是否存在和大小
  if (SD_MMC.exists(currentVideoName)) {
    File checkFile = SD_MMC.open(currentVideoName, FILE_READ);
    if (checkFile) {
      Serial.printf("文件大小: %lu bytes\n", checkFile.size());
      checkFile.close();
    }
  } else {
    Serial.println("警告: 录制文件不存在");
  }
  
  Serial.println("=== 录制停止完成 ===");
  printMemoryInfo();
}

void captureFrame() {
  if (!isRecording) return;
  
  // 简化文件有效性检查
  if (!videoFile) {
    Serial.println("错误: 视频文件未正确打开");
    isRecording = false;
    return;
  }
  
  // 简化文件可写性检查
  if (!videoFile.availableForWrite()) {
    Serial.println("错误: 视频文件不可写入");
    isRecording = false;
    return;
  }
  
  // 简化SD卡可用性检查
  if (!SD_MMC.exists("/")) {
    Serial.println("错误: SD卡不可访问");
    isRecording = false;
    return;
  }
  
  // 获取摄像头帧
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("摄像头获取帧失败");
    return;
  }
  
  // 简化帧数据检查
  if (fb->len == 0) {
    Serial.println("错误: 获取到空帧数据");
    esp_camera_fb_return(fb);
    return;
  }
  
  // 直接写入帧数据到文件
  size_t written = videoFile.write(fb->buf, fb->len);
  if (written != fb->len) {
    Serial.printf("写入帧数据失败，期望: %d, 实际: %d\n", fb->len, written);
    isRecording = false;
  } else {
    // 只有写入成功才更新帧计数
    frameCount++;
    
    // 减少打印频率以提高性能，每200帧打印一次
    if (frameCount % 200 == 0) {
      Serial.printf("已录制 %lu 帧\n", frameCount);
    }
  }
  
  // 释放帧缓冲区
  esp_camera_fb_return(fb);
}

void writeAVIHeader() {
  if (!videoFile) return;
  
  // 创建AVI头部结构
  uint8_t header[AVI_HEADER_SIZE];
  memset(header, 0, AVI_HEADER_SIZE);
  
  // RIFF头部
  memcpy(header, "RIFF", 4);
  memcpy(header + 4, &aviFileSize, 4);  // 文件大小占位符
  memcpy(header + 8, "AVI ", 4);
  
  // hdrl LIST
  memcpy(header + 12, "LIST", 4);
  uint32_t hdrlSize = 8 + 56 + 8 + 8 + 40;  // hdrl + avih + strl + strh + strf
  memcpy(header + 16, &hdrlSize, 4);
  memcpy(header + 20, "hdrl", 4);
  
  // avih主头部
  memcpy(header + 24, "avih", 4);
  uint32_t avihSize = 56;
  memcpy(header + 28, &avihSize, 4);
  memcpy(header + 32, &microSecondsPerFrame, 4);
  memcpy(header + 36, &maxBytesPerSecond, 4);
  memcpy(header + 40, &paddingGranularity, 4);
  memcpy(header + 44, &aviFlags, 4);
  memcpy(header + 48, &frameCount, 4);  // 总帧数占位符
  memcpy(header + 52, &initialFrames, 4);
  memcpy(header + 56, &streams, 4);
  memcpy(header + 60, &suggestedBufferSize, 4);
  memcpy(header + 64, &width, 4);
  memcpy(header + 68, &height, 4);
  memcpy(header + 72, &reserved, 16);
  
  // strl LIST
  memcpy(header + 88, "LIST", 4);
  uint32_t strlSize = 8 + 56 + 40;  // strl + strh + strf
  memcpy(header + 92, &strlSize, 4);
  memcpy(header + 96, "strl", 4);
  
  // strh流头
  memcpy(header + 100, "strh", 4);
  uint32_t strhSize = 56;
  memcpy(header + 104, &strhSize, 4);
  memcpy(header + 108, "vids", 4);  // 视频流
  memcpy(header + 112, &codec, 4);  // MJPG编码
  memcpy(header + 116, &streamFlags, 4);
  memcpy(header + 120, &priority, 4);
  memcpy(header + 124, &timeScale, 4);
  memcpy(header + 128, &sampleRate, 4);
  memcpy(header + 132, &startTime, 4);
  memcpy(header + 136, &length, 4);  // 帧数占位符
  memcpy(header + 140, &suggestedBufferSize, 4);
  memcpy(header + 144, &quality, 4);
  memcpy(header + 148, &sampleSize, 4);
  memcpy(header + 152, &frameRect, 16);
  
  // strf流格式 (BITMAPINFOHEADER)
  memcpy(header + 156, "strf", 4);
  uint32_t strfSize = 40;
  memcpy(header + 160, &strfSize, 4);
  memcpy(header + 164, &biSize, 4);
  memcpy(header + 168, &width, 4);
  memcpy(header + 172, &height, 4);
  memcpy(header + 176, &biPlanes, 2);
  memcpy(header + 178, &bitCount, 2);
  memcpy(header + 180, &compression, 4);
  memcpy(header + 184, &imageSize, 4);
  memcpy(header + 188, &xPelsPerMeter, 4);
  memcpy(header + 192, &yPelsPerMeter, 4);
  memcpy(header + 196, &colorsUsed, 4);
  memcpy(header + 200, &colorsImportant, 4);
  
  // 简化文件写入检查
  if (videoFile && videoFile.availableForWrite()) {
    videoFile.write(header, AVI_HEADER_SIZE);
    
    // 填充头部到512字节
    uint8_t padding[512 - AVI_HEADER_SIZE];
    memset(padding, 0, sizeof(padding));
    videoFile.write(padding, sizeof(padding));
  }
}

void updateAVIHeader() {
  if (!videoFile) return;
  
  // 简化文件可写性检查
  if (!videoFile.availableForWrite()) {
    Serial.println("错误: 无法更新AVI头部，文件不可写入");
    return;
  }
  
  // 更新RIFF大小
  videoFile.seek(4);
  uint32_t riffSize = videoFile.size() - 8;
  videoFile.write((uint8_t*)&riffSize, 4);
  
  // 更新总帧数
  videoFile.seek(48);
  videoFile.write((uint8_t*)&frameCount, 4);
  
  // 更新strh块中的长度字段
  videoFile.seek(136);
  videoFile.write((uint8_t*)&frameCount, 4);
  
  // 刷新缓冲区确保数据写入
  videoFile.flush();
}

void handleRoot() {
  String html = F("<!DOCTYPE html><html lang='zh-CN'><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'><title>AVICAM2.0 视频录制系统</title>");
  html += F("<style>body{font-family:Arial,sans-serif;margin:0;padding:20px;background-color:#f0f0f0;}h1{text-align:center;color:#333;}.container{max-width:800px;margin:0 auto;background:white;padding:20px;border-radius:10px;box-shadow:0 0 10px rgba(0,0,0,0.1);}.btn{display:inline-block;padding:10px 20px;margin:5px;font-size:16px;cursor:pointer;border:none;border-radius:5px;text-decoration:none;transition:background-color 0.3s;}.btn-record{background-color:#4CAF50;color:white;}.btn-stop{background-color:#f44336;color:white;}.btn:hover{opacity:0.8;}.status{margin:20px 0;padding:15px;background-color:#e7f3ff;border-left:6px solid #2196F3;border-radius:4px;}.video-list{margin-top:30px;}.video-item{display:flex;justify-content:space-between;align-items:center;padding:10px;border-bottom:1px solid #ddd;}.video-item:last-child{border-bottom:none;}.video-actions a{margin-left:10px;}</style>");
  html += F("</head><body><div class='container'><h1>AVICAM2.0 视频录制系统</h1>");
  html += F("<div class='status' id='status'>正在加载系统状态...</div>");
  html += F("<button class='btn btn-record' onclick='startRecording()' id='recordBtn'>开始录制</button>");
  html += F("<button class='btn btn-stop' onclick='stopRecording()' id='stopBtn' disabled>停止录制</button>");
  html += F("<hr><h2>实时预览</h2><img src='/stream' width='100%' style='max-width:640px;' alt='摄像头预览'>");
  html += F("<div class='video-list'><h2>已录制视频</h2><div id='videoList'>正在加载视频列表...</div></div>");
  html += F("<script>");
  html += F("function updateStatus(){fetch('/status').then(response=>response.json()).then(data=>{document.getElementById('status').innerHTML=`<strong>系统状态:</strong> ${data.isRecording?'正在录制':'待机'} | 触发状态: ${data.triggerStatus?'激活':'未激活'} | 帧计数: ${data.frameCount} | 堆内存: ${data.freeHeap} bytes | PSRAM: ${data.psramSize} bytes | SD卡: ${data.sdCardUsed}/${data.sdCardTotal} MB`;document.getElementById('recordBtn').disabled=data.isRecording;document.getElementById('stopBtn').disabled=!data.isRecording;}).catch(err=>console.error('状态更新失败:',err));}");
  html += F("function startRecording(){fetch('/startRecording',{method:'POST'}).then(()=>{updateStatus();loadVideoList();}).catch(err=>console.error('开始录制失败:',err));}");
  html += F("function stopRecording(){fetch('/stopRecording',{method:'POST'}).then(()=>{updateStatus();loadVideoList();}).catch(err=>console.error('停止录制失败:',err));}");
  html += F("function loadVideoList(){fetch('/list').then(response=>response.text()).then(data=>{document.getElementById('videoList').innerHTML=data;}).catch(err=>console.error('加载视频列表失败:',err));}");
  html += F("setInterval(updateStatus,2000);updateStatus();setInterval(loadVideoList,5000);loadVideoList();");
  html += F("</script></body></html>");
  server.send(200, "text/html", html);
}

void handleList() {
  String html = listVideos();
  server.send(200, "text/html", html);
}

String listVideos() {
  String html = "";
  File root = SD_MMC.open("/");
  if (!root) {
    return "无法打开根目录";
  }
  
  File file = root.openNextFile();
  while (file) {
    String fileName = file.name();
    if (fileName.endsWith(".avi")) {
      // 简化文件大小计算
      float fileSizeMB = file.size() / (1024.0 * 1024.0);
      
      html += "<div class='video-item'>";
      html += "<span>" + fileName + " (" + String(fileSizeMB, 2) + " MB)</span>";
      html += "<div class='video-actions'>";
      html += "<a href='/download?file=" + fileName + "' target='_blank'>下载</a> ";
      html += "<a href='#' onclick='deleteVideo(\"" + fileName + "\")'>删除</a>";
      html += "</div>";
      html += "</div>";
    }
    file = root.openNextFile();
  }
  
  if (html == "") {
    html = "<p>暂无录制视频</p>";
  }
  
  return html;
}

void handleDownload() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "缺少文件参数");
    return;
  }
  
  String filename = server.arg("file");
  
  // 简化安全检查
  if (filename.indexOf("..") != -1) {
    server.send(400, "text/plain", "无效文件名");
    return;
  }
  
  String path = "/" + filename;
  File file = SD_MMC.open(path, FILE_READ);
  if (!file) {
    server.send(404, "text/plain", "文件不存在");
    return;
  }
  
  // 设置响应头
  server.sendHeader("Content-Type", "application/octet-stream");
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
  server.streamFile(file, "application/octet-stream");
  file.close();
}

void handleDelete() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "缺少文件参数");
    return;
  }
  
  String filename = server.arg("file");
  
  // 简化安全检查
  if (filename.indexOf("..") != -1) {
    server.send(400, "text/plain", "无效文件名");
    return;
  }
  
  String path = "/" + filename;
  if (SD_MMC.remove(path.c_str())) {
    server.send(200, "text/plain", "删除成功");
  } else {
    server.send(500, "text/plain", "删除失败");
  }
}

void handleStatus() {
  String json = "{";
  json += "\"isRecording\":" + String(isRecording ? "true" : "false") + ",";
  json += "\"triggerStatus\":" + String(digitalRead(RECORD_TRIGGER_PIN) == LOW ? "true" : "false") + ",";
  json += "\"frameCount\":" + String(frameCount) + ",";
  
  // 简化内存信息获取
  json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"psramSize\":" + String(ESP.getPsramSize()) + ",";
  
  // 简化SD卡信息获取
  uint64_t sdCardTotal = SD_MMC.totalBytes() / (1024 * 1024);
  uint64_t sdCardUsed = SD_MMC.usedBytes() / (1024 * 1024);
  json += "\"sdCardTotal\":" + String(sdCardTotal) + ",";
  json += "\"sdCardUsed\":" + String(sdCardUsed);
  
  json += "}";
  server.send(200, "application/json", json);
}

void handleStartRecording() {
  startRecording();
  server.send(200, "text/plain", isRecording ? "录制已开始" : "录制启动失败");
}

void handleStopRecording() {
  stopRecording();
  server.send(200, "text/plain", "录制已停止");
}

void handleStream() {
  if (!cameraInitialized) {
    server.send(500, "text/plain", "摄像头未初始化");
    return;
  }
  
  // 设置MJPEG流响应头
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "multipart/x-mixed-replace;boundary=frame", "");
  
  // 发送100帧然后结束流
  for (int i = 0; i < 100; i++) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("摄像头获取帧失败");
      break;
    }
    
    // 发送帧分隔符和头部
    server.sendContent("--frame\r\n");
    server.sendContent("Content-Type: image/jpeg\r\n");
    server.sendContent("Content-Length: " + String(fb->len) + "\r\n\r\n");
    
    // 发送图像数据
    server.sendContent((char*)fb->buf, fb->len);
    server.sendContent("\r\n");
    
    esp_camera_fb_return(fb);
    
    // 短暂延迟以控制帧率
    delay(50);
  }
}

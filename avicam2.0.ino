/*
 * AVICAM 2.0 - ESP32-CAM 视频录制系统
 * 
 * 功能描述：
 * - 基于ESP32-CAM模块的视频录制系统
 * - 支持通过GPIO2引脚触发录制
 * - 使用SD卡存储AVI格式视频文件
 * - 提供Web界面进行远程控制和文件管理
 * - 支持4线SD卡模式，提高数据传输速度
 * 
 * 硬件配置：
 * - 摄像头型号：AI Thinker ESP32-CAM
 * - 触发引脚：GPIO2（录制控制）
 * - LED指示灯：GPIO4（板载LED）
 * - SD卡模式：4线模式（兼容1线模式）
 * 
 * 文件格式：
 * - 视频文件：AVI格式，包含JPEG帧序列
 * - 文件名：video_时间戳.avi
 * 
 * 作者：ai
 * 版本：2.0
 * 创建时间：2025年
 */

#include "esp_camera.h"
#include "WiFi.h"
#include "WebServer.h"
#include "ESPmDNS.h"
#include "FS.h"
#include "SD_MMC.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"

// ==================== 函数声明 ====================

// 系统信息显示函数
void printMemoryInfo();        // 显示内存和SD卡状态信息
void printCameraInfo();        // 显示摄像头配置信息
void checkSDCardStatus();      // 详细检查SD卡状态

// 视频录制控制函数
void startRecording();         // 开始录制视频
void stopRecording();          // 停止录制视频
void captureFrame();           // 捕获单帧图像
void writeAVIHeader();         // 写入AVI文件头部
void updateAVIHeader();        // 更新AVI文件头部信息

// Web服务器处理函数
void handleRoot();             // 处理根路径请求
void handleDownload();         // 处理文件下载请求
void handleList();             // 处理文件列表请求
void handleDelete();           // 处理文件删除请求
void handleStatus();           // 处理系统状态查询
void handleStartRecording();   // 处理开始录制请求
void handleStopRecording();    // 处理停止录制请求
void handleStream();           // 处理视频流请求
String listVideos();           // 生成视频文件列表HTML

// ==================== 硬件配置 ====================

// 选择摄像头型号 - AI Thinker ESP32-CAM开发板
#define CAMERA_MODEL_AI_THINKER

/*
 * AI Thinker ESP32-CAM摄像头引脚定义
 * 注意：这些引脚定义与ESP32-CAM模块硬件布局对应
 */
#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32   // 摄像头电源控制引脚
  #define RESET_GPIO_NUM    -1   // 摄像头复位引脚（未使用）
  #define XCLK_GPIO_NUM      0   // 摄像头时钟引脚
  #define SIOD_GPIO_NUM     26   // I2C数据引脚
  #define SIOC_GPIO_NUM     27   // I2C时钟引脚
  
  // 摄像头数据引脚（Y0-Y9，对应D0-D7）
  #define Y9_GPIO_NUM       35   // D7
  #define Y8_GPIO_NUM       34   // D6
  #define Y7_GPIO_NUM       39   // D5
  #define Y6_GPIO_NUM       36   // D4
  #define Y5_GPIO_NUM       21   // D3
  #define Y4_GPIO_NUM       19   // D2
  #define Y3_GPIO_NUM       18   // D1
  #define Y2_GPIO_NUM        5   // D0
  
  // 摄像头控制引脚
  #define VSYNC_GPIO_NUM    25   // 垂直同步信号
  #define HREF_GPIO_NUM     23   // 水平参考信号
  #define PCLK_GPIO_NUM     22   // 像素时钟
#endif

// ==================== 系统控制引脚 ====================

// 录制控制引脚 - 使用GPIO2作为触发引脚
// 注意：GPIO2在ESP32-CAM上通常用于串口通信，请确保硬件连接正确
#define RECORD_TRIGGER_PIN 2

// ESP32-CAM板载LED引脚 - 使用GPIO2但不激活LED
// 注意：LED为低电平点亮，高电平熄灭
#define LED_PIN 2

// ==================== 全局变量定义 ====================

// Web服务器 - 监听端口80
WebServer server(80);

// 录制状态变量
bool isRecording = false;           // 当前是否正在录制
bool cameraInitialized = false;    // 摄像头是否已初始化
File videoFile;                    // 当前录制的视频文件句柄
unsigned long frameCount = 0;       // 已录制的帧数
unsigned long startTime = 0;       // 录制开始时间（毫秒）
String currentVideoName = "";      // 当前视频文件名

// AVI文件结构
const int AVI_HEADER_SIZE = 512;   // AVI文件头部大小（字节）
uint8_t aviHeader[AVI_HEADER_SIZE]; // AVI头部数据缓冲区

/*
 * printMemoryInfo - 显示系统内存和SD卡状态信息
 * 
 * 功能：
 * - 显示ESP32堆内存使用情况（包括总计、可用、最小可用和占用率）
 * - 显示SRAM占用率信息（基于ESP32-WROOM-32的典型SRAM大小）
 * - 检测并显示PSRAM状态（如果可用，包括占用率）
 * - 显示闪存芯片大小
 * - 显示SD卡基本状态信息
 * 
 * 输出示例：
 * 堆内存 - 总计: 327680 bytes, 可用: 123456 bytes, 最小可用: 123000 bytes, 占用率: 62.3%
 * SRAM - 总计: 520000 bytes, 已用: 234567 bytes, 占用率: 45.1%
 * PSRAM - 总计: 4194304 bytes, 可用: 4190000 bytes, 占用率: 0.1%
 * 闪存芯片大小: 4194304 bytes
 * SD卡状态诊断:
 *   SD_MMC.cardSize(): 4000000000 bytes
 *   SD_MMC.totalBytes(): 4000000000 bytes
 *   SD_MMC.usedBytes(): 100000000 bytes
 *   SD_MMC.exists('/'):是
 */
void printMemoryInfo() {
  // 计算堆内存占用率
  int heapTotal = ESP.getHeapSize();
  int heapFree = ESP.getFreeHeap();
  int heapUsed = heapTotal - heapFree;
  float heapUsagePercent = (float)heapUsed / heapTotal * 100;
  
  // 显示堆内存详细信息
  Serial.printf("堆内存 - 总计: %d bytes, 可用: %d bytes, 最小可用: %d bytes, 占用率: %.1f%%\n", 
                heapTotal, heapFree, ESP.getMinFreeHeap(), heapUsagePercent);
  
  // 计算SRAM占用率（ESP32的总内存信息）
  int sramTotal = 520000;  // ESP32-WROOM-32的典型SRAM大小
  int sramUsed = heapUsed;  // 堆内存使用量作为SRAM使用量的近似值
  float sramUsagePercent = (float)sramUsed / sramTotal * 100;
  
  // 显示SRAM占用率信息
  Serial.printf("SRAM - 总计: %d bytes, 已用: %d bytes, 占用率: %.1f%%\n", 
                sramTotal, sramUsed, sramUsagePercent);
  
  // 检查并显示PSRAM状态
  if(psramFound()) {
    int psramTotal = ESP.getPsramSize();
    int psramFree = ESP.getFreePsram();
    int psramUsed = psramTotal - psramFree;
    float psramUsagePercent = (float)psramUsed / psramTotal * 100;
    
    Serial.printf("PSRAM - 总计: %d bytes, 可用: %d bytes, 占用率: %.1f%%\n", 
                  psramTotal, psramFree, psramUsagePercent);
  } else {
    Serial.println("未检测到PSRAM");
  }
  
  // 显示闪存芯片大小
  Serial.printf("闪存芯片大小: %d bytes\n", ESP.getFlashChipSize());
  
  // SD卡状态诊断信息
  Serial.printf("SD卡状态诊断:\n");
  Serial.printf("  SD_MMC.cardSize(): %llu bytes\n", SD_MMC.cardSize());
  Serial.printf("  SD_MMC.totalBytes(): %llu bytes\n", SD_MMC.totalBytes());
  Serial.printf("  SD_MMC.usedBytes(): %llu bytes\n", SD_MMC.usedBytes());
  Serial.printf("  SD_MMC.exists('/'):%s\n", SD_MMC.exists("/") ? "是" : "否");
}

/*
 * checkSDCardStatus - 详细检查SD卡状态
 * 
 * 功能：
 * - 检查SD卡是否挂载和类型识别
 * - 显示SD卡容量和文件系统信息
 * - 验证根目录存在性
 * - 执行写入测试验证可写性
 * - 列出根目录文件（前10个）
 * 
 * 输出示例：
 * === SD卡详细状态检查 ===
 * SD卡类型: SDHC
 * SD卡总容量: 4000000000 bytes (3814 MB)
 * 文件系统总字节: 4000000000 bytes (3814 MB)
 * 已使用字节: 100000000 bytes (95 MB)
 * 可用空间: 3900000000 bytes (3719 MB)
 * SD卡根目录存在
 * SD卡写入测试成功
 * 根目录文件列表:
 *   video_122615.avi (0 bytes)
 *   video_15322.avi (1 bytes)
 * === SD卡状态检查结束 ===
 */
void checkSDCardStatus() {
  Serial.println("=== SD卡详细状态检查 ===");
  
  // 检查SD卡是否挂载
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("错误: 未检测到SD卡");
    return;
  }
  
  // 识别并显示SD卡类型
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
  
  // 显示容量信息
  uint64_t cardSize = SD_MMC.cardSize();
  uint64_t totalBytes = SD_MMC.totalBytes();
  uint64_t usedBytes = SD_MMC.usedBytes();
  
  Serial.printf("SD卡总容量: %llu bytes (%llu MB)\n", cardSize, cardSize / (1024 * 1024));
  Serial.printf("文件系统总字节: %llu bytes (%llu MB)\n", totalBytes, totalBytes / (1024 * 1024));
  Serial.printf("已使用字节: %llu bytes (%llu MB)\n", usedBytes, usedBytes / (1024 * 1024));
  Serial.printf("可用空间: %llu bytes (%llu MB)\n", totalBytes - usedBytes, (totalBytes - usedBytes) / (1024 * 1024));
  
  // 检查根目录存在性
  if (!SD_MMC.exists("/")) {
    Serial.println("错误: SD卡根目录不存在");
  } else {
    Serial.println("SD卡根目录存在");
  }
  
  // 执行写入测试验证SD卡可写性
  File testFile = SD_MMC.open("/sd_test.tmp", FILE_WRITE);
  if (testFile) {
    if (testFile.println("SD卡测试")) {
      Serial.println("SD卡写入测试成功");
    } else {
      Serial.println("错误: SD卡写入测试失败");
    }
    testFile.close();
    SD_MMC.remove("/sd_test.tmp");  // 清理测试文件
  } else {
    Serial.println("错误: SD卡创建文件测试失败");
  }
  
  // 列出根目录内容（前10个文件）
  File root = SD_MMC.open("/");
  if (root && root.isDirectory()) {
    File file = root.openNextFile();
    int count = 0;
    Serial.println("根目录文件列表:");
    while(file && count < 10) {
      Serial.printf("  %s (%lu bytes)\n", file.name(), file.size());
      file = root.openNextFile();
      count++;
    }
    if (count >= 10) {
      Serial.println("  ... (更多文件)");
    }
    root.close();
  }
  
  Serial.println("=== SD卡状态检查结束 ===");
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

/*
 * setup - 系统初始化函数
 * 
 * 功能：
 * - 初始化串口通信
 * - 配置GPIO引脚
 * - 初始化摄像头模块
 * - 初始化SD卡存储
 * - 配置Web服务器
 * - 启动系统服务
 * 
 * 执行流程：
 * 1. 串口初始化
 * 2. GPIO引脚配置
 * 3. 摄像头初始化
 * 4. SD卡初始化
 * 5. Web服务器配置
 * 6. 系统状态检查
 */
void setup() {
  // 初始化串口通信 - 波特率115200
  Serial.begin(115200);
  Serial.setDebugOutput(true);  // 启用调试输出
  Serial.println();
  
  // 显示系统启动信息和内存状态
  Serial.println("=== AVICAM 2.0 系统启动 ===");
  Serial.printf("启动时间: %lu ms\n", millis());
  printMemoryInfo();
  
  // ==================== GPIO引脚配置 ====================
  
  // 初始化触发引脚 - GPIO2，上拉输入模式
  // 注意：触发信号为低电平有效（引脚被拉低时开始录制）
  pinMode(RECORD_TRIGGER_PIN, INPUT_PULLUP);
  
  // 初始化LED引脚 - GPIO4，输出模式
  // 注意：LED为低电平点亮，高电平熄灭
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // 默认关闭LED
  
  // 确保LED永远不被打开 - 避免与摄像头功能冲突
  
  // ==================== 摄像头配置 ====================
  
  // 初始化摄像头配置结构体
  camera_config_t config;
  
  // LEDC（LED控制器）配置 - 用于摄像头时钟
  config.ledc_channel = LEDC_CHANNEL_0;  // 使用LEDC通道0
  config.ledc_timer = LEDC_TIMER_0;     // 使用LEDC定时器0
  
  // 摄像头数据引脚配置（D0-D7）
  config.pin_d0 = Y2_GPIO_NUM;  // D0数据引脚
  config.pin_d1 = Y3_GPIO_NUM;  // D1数据引脚
  config.pin_d2 = Y4_GPIO_NUM;  // D2数据引脚
  config.pin_d3 = Y5_GPIO_NUM;  // D3数据引脚
  config.pin_d4 = Y6_GPIO_NUM;  // D4数据引脚
  config.pin_d5 = Y7_GPIO_NUM;  // D5数据引脚
  config.pin_d6 = Y8_GPIO_NUM;  // D6数据引脚
  config.pin_d7 = Y9_GPIO_NUM;  // D7数据引脚
  
  // 摄像头控制引脚配置
  config.pin_xclk = XCLK_GPIO_NUM;     // 摄像头时钟引脚
  config.pin_pclk = PCLK_GPIO_NUM;     // 像素时钟引脚
  config.pin_vsync = VSYNC_GPIO_NUM;   // 垂直同步信号
  config.pin_href = HREF_GPIO_NUM;     // 水平参考信号
  config.pin_sccb_sda = SIOD_GPIO_NUM; // I2C数据引脚（SDA）
  config.pin_sccb_scl = SIOC_GPIO_NUM; // I2C时钟引脚（SCL）
  config.pin_pwdn = PWDN_GPIO_NUM;     // 电源控制引脚
  config.pin_reset = RESET_GPIO_NUM;   // 复位引脚
  
  // 摄像头工作参数配置
  config.xclk_freq_hz = 20000000;      // 摄像头时钟频率20MHz
  config.pixel_format = PIXFORMAT_JPEG; // 像素格式：JPEG压缩
  config.grab_mode = CAMERA_GRAB_LATEST;  // 抓取模式：获取最新帧
  config.fb_location = CAMERA_FB_IN_PSRAM;  // 帧缓冲区位置：PSRAM
  
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

  // ==================== SD卡初始化 ====================
  
  Serial.println("开始初始化SD卡...");
  
  // 使用4线模式初始化SD卡（优先尝试）
  // 参数说明：
  // - "/sdcard": 挂载点名称
  // - false: 不使用1线模式（使用4线模式）
  // - false: 不格式化
  // - 10000000: 超时时间10秒
  if(!SD_MMC.begin("/sdcard", false, false, 10000000)){  
    Serial.println("SD卡4线模式挂载失败，尝试其他参数...");
    
    // 尝试1线模式（兼容性更好）
    delay(100);  // 延迟100ms确保稳定
    if(!SD_MMC.begin("/sdcard", true, false, 10000000)){
      Serial.println("SD卡1线模式挂载也失败");
      
      // 尝试默认参数（最后的手段）
      delay(100);
      if(!SD_MMC.begin()){
        Serial.println("使用默认参数也失败");
        return;  // SD卡初始化失败，系统无法继续
      } else {
        Serial.println("使用默认参数成功");
      }
    } else {
      Serial.println("使用1线模式成功");
    }
  } else {
    Serial.println("SD卡4线模式初始化成功");
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
    // 尝试创建根目录
    File root = SD_MMC.open("/", FILE_WRITE);
    if (root) {
      Serial.println("根目录创建成功");
      root.close();
    } else {
      Serial.println("根目录创建失败");
      return;
    }
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
  
  // 执行详细的SD卡状态检查
  checkSDCardStatus();

  // 创建WiFi热点
  Serial.println("正在创建WiFi热点...");
  
  // 配置WiFi热点参数
  WiFi.disconnect(true);
  delay(100);
  
  // 设置WiFi模式为AP模式
  WiFi.mode(WIFI_AP);
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
  
  // 标记摄像头已初始化
  cameraInitialized = true;
  
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
  if (isRecording) return;
  
  Serial.println("=== 开始录制准备 ===");
  printMemoryInfo();
  
  // 执行详细的SD卡状态检查
  checkSDCardStatus();
  
  // 生成文件名 - 使用绝对路径格式
  currentVideoName = "/video_" + String(millis()) + ".avi";
  Serial.println("尝试创建视频文件: " + currentVideoName);
  
  // 检查SD卡状态
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("错误: 未检测到SD卡");
    return;
  }
  
  // 显示SD卡信息
  Serial.print("SD卡类型: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  
  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  uint64_t totalBytes = SD_MMC.totalBytes() / (1024 * 1024);
  uint64_t usedBytes = SD_MMC.usedBytes() / (1024 * 1024);
  Serial.printf("SD卡大小: %lluMB, 总空间: %lluMB, 已使用: %lluMB\n", cardSize, totalBytes, usedBytes);
  
  // 检查可用空间
  if ((totalBytes - usedBytes) < 10) {  // 少于10MB空间
    Serial.println("警告: SD卡空间不足");
  }
  
  // SD卡状态诊断
  Serial.println("SD卡状态诊断:");
  Serial.printf("  SD_MMC.cardSize(): %llu bytes\n", SD_MMC.cardSize());
  Serial.printf("  SD_MMC.totalBytes(): %llu bytes\n", SD_MMC.totalBytes());
  Serial.printf("  SD_MMC.usedBytes(): %llu bytes\n", SD_MMC.usedBytes());
  Serial.printf("  SD_MMC.exists('/'): %s\n", SD_MMC.exists("/") ? "是" : "否");
  
  // 测试写入权限 - 使用绝对路径
  File testFile = SD_MMC.open("/test.txt", FILE_WRITE);
  if (testFile) {
    size_t written = testFile.println("Write test");
    if (written > 0) {
      Serial.println("SD卡写入权限测试成功");
    } else {
      Serial.println("错误: SD卡写入权限测试失败 - 无法写入数据");
    }
    testFile.close();
    SD_MMC.remove("/test.txt");
  } else {
    Serial.println("错误: SD卡写入权限测试失败 - 无法创建测试文件");
    return;
  }
  
  // 尝试打开视频文件 - 使用更可靠的创建方法
  Serial.println("尝试创建视频文件: " + currentVideoName);
  
  // 先删除可能存在的同名文件
  if (SD_MMC.exists(currentVideoName.c_str())) {
    Serial.println("发现同名文件，先删除...");
    SD_MMC.remove(currentVideoName.c_str());
    delay(100);
  }
  
  // 使用更稳健的文件创建方法
  Serial.println("使用稳健的文件创建方法...");
  
  // 方法1: 先创建空文件并立即关闭
  File tempFile = SD_MMC.open(currentVideoName, FILE_WRITE);
  if (!tempFile) {
    Serial.println("错误: 无法创建临时文件");
    return;
  }
  tempFile.close();
  delay(50);
  
  // 方法2: 重新打开文件用于写入
  videoFile = SD_MMC.open(currentVideoName, FILE_WRITE);
  
  // 检查文件是否成功创建
  if (!videoFile) {
    Serial.println("错误: 无法打开视频文件");
    
    // 诊断信息
    Serial.println("诊断信息:");
    
    // 检查根目录是否可写
    File rootTest = SD_MMC.open("/", FILE_WRITE);
    if (rootTest) {
      Serial.println("根目录可写");
      rootTest.close();
    } else {
      Serial.println("根目录不可写");
    }
    
    // 检查SD卡状态
    if (!SD_MMC.exists("/")) {
      Serial.println("错误: SD卡根目录不存在");
      return;
    }
    
    // 检查文件是否存在
    if (SD_MMC.exists(currentVideoName.c_str())) {
      Serial.println("文件存在但无法打开");
    } else {
      Serial.println("文件不存在");
    }
    
    return;
  }
  
  // 检查新创建的文件状态
  Serial.println("文件创建成功");
  Serial.println("详细诊断信息:");
  Serial.println("  文件名: " + currentVideoName);
  Serial.printf("  文件大小: %lu\n", videoFile.size());
  Serial.printf("  文件是否可写: %s\n", videoFile.availableForWrite() ? "是" : "否");
  Serial.printf("  文件是否有效: %s\n", videoFile ? "是" : "否");
  
  // 如果文件不可写，尝试修复
  if (!videoFile.availableForWrite()) {
    Serial.println("警告: 文件创建后不可写，尝试修复...");
    
    // 关闭当前文件句柄
    videoFile.close();
    delay(100);
    
    // 删除文件
    if (SD_MMC.exists(currentVideoName.c_str())) {
      SD_MMC.remove(currentVideoName.c_str());
      delay(100);
    }
    
    // 重新创建文件
    File tempFile2 = SD_MMC.open(currentVideoName, FILE_WRITE);
    if (!tempFile2) {
      Serial.println("错误: 无法重新创建文件");
      return;
    }
    tempFile2.close();
    delay(50);
    
    // 重新打开
    videoFile = SD_MMC.open(currentVideoName, FILE_WRITE);
    
    if (!videoFile || !videoFile.availableForWrite()) {
      Serial.println("错误: 文件修复失败");
      if (videoFile) videoFile.close();
      return;
    } else {
      Serial.println("文件修复成功");
    }
  }
  
  // 验证文件状态
  if (videoFile.size() != 0) {
    Serial.printf("警告: 新创建的文件大小不为0 (%lu bytes)\n", videoFile.size());
    // 关闭文件并重新创建以清空内容
    videoFile.close();
    delay(50);
    
    if (SD_MMC.exists(currentVideoName.c_str())) {
      SD_MMC.remove(currentVideoName.c_str());
      delay(50);
    }
    
    // 重新创建文件
    File tempFile3 = SD_MMC.open(currentVideoName, FILE_WRITE);
    if (!tempFile3) {
      Serial.println("错误: 无法重新创建文件以清空内容");
      return;
    }
    tempFile3.close();
    delay(50);
    
    videoFile = SD_MMC.open(currentVideoName, FILE_WRITE);
    if (!videoFile) {
      Serial.println("错误: 无法重新打开清空后的文件");
      return;
    }
    Serial.println("已清空文件内容");
  }
  
  // 写入AVI头部
  Serial.println("开始写入AVI头部...");
  writeAVIHeader();
  
  // 检查头部是否写入成功
  if (videoFile.size() < 512) {  // 头部应该至少512字节
    Serial.printf("警告: AVI头部写入可能不完整，当前文件大小: %lu bytes\n", videoFile.size());
    // 强制刷新
    videoFile.flush();
    delay(100);
    Serial.printf("刷新后文件大小: %lu bytes\n", videoFile.size());
  } else {
    Serial.printf("AVI头部写入成功，文件大小: %lu bytes\n", videoFile.size());
  }
  
  // 初始化录制状态
  frameCount = 0;
  startTime = millis();
  isRecording = true;
  
  Serial.println("开始录制");
  // 不再控制LED指示灯
  // digitalWrite(LED_PIN, LOW); // 注释掉LED控制代码
  // Serial.println("录制LED指示灯已开启"); // 注释掉LED状态信息
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
    // 强制刷新确保所有数据写入
    videoFile.flush();
    delay(100);
    videoFile.close();
    Serial.println("视频文件已关闭");
  } else {
    Serial.println("警告: 文件句柄已失效");
  }
  
  unsigned long duration = (millis() - startTime) / 1000;
  Serial.printf("停止录制。帧数: %lu, 时长: %lu秒\n", frameCount, duration);
  Serial.println("文件保存为: " + currentVideoName);
  
  // 检查文件是否存在和大小
  if (SD_MMC.exists(currentVideoName.c_str())) {
    File checkFile = SD_MMC.open(currentVideoName, FILE_READ);
    if (checkFile) {
      Serial.printf("文件大小: %lu bytes\n", checkFile.size());
      checkFile.close();
      
      // 验证文件是否有效AVI文件
      if (checkFile.size() < 512) {
        Serial.println("警告: 文件可能不完整");
      } else {
        Serial.println("录制文件保存成功");
      }
    } else {
      Serial.println("错误: 无法打开文件进行大小检查");
    }
  } else {
    Serial.println("警告: 录制文件不存在");
    // 列出根目录下的文件检查
    File root = SD_MMC.open("/");
    if (root) {
      File file = root.openNextFile();
      Serial.println("SD卡根目录文件列表:");
      while(file) {
        Serial.printf("  %s (大小: %lu bytes)\n", file.name(), file.size());
        file = root.openNextFile();
      }
      root.close();
    }
  }
  
  Serial.println("=== 录制停止完成 ===");
  printMemoryInfo();
}

void captureFrame() {
  if (!isRecording) return;
  
  // 检查文件是否有效
  if (!videoFile) {
    Serial.println("错误: 视频文件句柄无效");
    // 尝试重新创建文件
    if (SD_MMC.exists(currentVideoName.c_str())) {
      SD_MMC.remove(currentVideoName.c_str());
      delay(100);
    }
    
    // 使用稳健的文件创建方法
    File tempFile = SD_MMC.open(currentVideoName, FILE_WRITE);
    if (!tempFile) {
      Serial.println("错误: 无法创建临时文件");
      return;
    }
    tempFile.close();
    delay(50);
    
    videoFile = SD_MMC.open(currentVideoName, FILE_WRITE);
    if (!videoFile) {
      Serial.println("错误: 重新创建文件失败");
      Serial.printf("文件名: %s\n", currentVideoName.c_str());
      return;
    }
    Serial.println("重新创建文件成功");
  }
  
  // 检查文件是否可写
  if (!videoFile.availableForWrite()) {
    Serial.println("错误: 文件不可写");
    // 尝试重新创建文件
    videoFile.close();
    delay(100);
    
    if (SD_MMC.exists(currentVideoName.c_str())) {
      SD_MMC.remove(currentVideoName.c_str());
      delay(100);
    }
    
    // 使用稳健的文件创建方法
    File tempFile = SD_MMC.open(currentVideoName, FILE_WRITE);
    if (!tempFile) {
      Serial.println("错误: 无法创建临时文件");
      return;
    }
    tempFile.close();
    delay(50);
    
    videoFile = SD_MMC.open(currentVideoName, FILE_WRITE);
    if (!videoFile || !videoFile.availableForWrite()) {
      Serial.println("错误: 重新创建文件用于写入失败");
      Serial.printf("文件名: %s\n", currentVideoName.c_str());
      Serial.printf("文件是否有效: %s\n", videoFile ? "是" : "否");
      if (videoFile) {
        Serial.printf("文件大小: %lu\n", videoFile.size());
        videoFile.close();
      }
      return;
    }
    Serial.println("重新创建文件用于写入成功");
  }
  
  // 检查SD卡是否仍然可用
  if (!SD_MMC.exists("/")) {
    Serial.println("错误: SD卡根目录不存在");
    return;
  }
  
  // 检查当前视频文件是否存在
  if (!SD_MMC.exists(currentVideoName.c_str())) {
    Serial.println("错误: SD卡上不存在当前视频文件");
    Serial.printf("文件名: %s\n", currentVideoName.c_str());
    return;
  }
  
  // 检查可用空间
  uint64_t freeSpace = SD_MMC.totalBytes() - SD_MMC.usedBytes();
  if (freeSpace < 1024 * 1024) {  // 少于1MB空间
    Serial.printf("警告: SD卡空间不足 (%llu bytes)\n", freeSpace);
  }
  
  // 从摄像头获取帧
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("错误: 无法从摄像头获取帧");
    return;
  }
  
  // 检查帧数据
  if (fb->len == 0) {
    Serial.println("错误: 获取到空帧数据");
    esp_camera_fb_return(fb);
    return;
  }
  
  // 写入帧数据到文件
  size_t written = videoFile.write(fb->buf, fb->len);
  if (written != fb->len) {
    Serial.printf("错误: 帧数据写入不完整。应写入: %d bytes, 实际写入: %d bytes\n", fb->len, written);
    
    // 尝试强制刷新缓冲区
    videoFile.flush();
    delay(100);
    
    // 检查文件状态
    Serial.printf("文件大小: %lu bytes\n", videoFile.size());
    Serial.printf("文件是否有效: %s\n", videoFile ? "是" : "否");
    Serial.printf("文件是否可写: %s\n", videoFile.availableForWrite() ? "是" : "否");
    Serial.printf("SD卡剩余空间: %llu bytes\n", freeSpace);
    
    // 释放帧内存
    esp_camera_fb_return(fb);
    return;
  }
  
  // 更新帧计数
  frameCount++;
  
  // 每10帧刷新一次数据到SD卡
  if (frameCount % 10 == 0) {
    videoFile.flush();
  }
  
  // 每100帧打印一次进度
  if (frameCount % 100 == 0) {
    Serial.printf("已录制 %lu 帧，当前文件大小: %lu bytes\n", frameCount, videoFile.size());
  }
  
  // 释放帧内存
  esp_camera_fb_return(fb);
}

void writeAVIHeader() {
  if (!videoFile) return;
  
  // AVI文件头变量定义
  uint32_t aviFileSize = 0;
  uint32_t microSecondsPerFrame = 1000000 / 30;  // 30 FPS
  uint32_t maxBytesPerSecond = 1000000;
  uint32_t paddingGranularity = 0;
  uint32_t aviFlags = 0x10;  // AVIF_HASINDEX
  uint32_t initialFrames = 0;
  uint32_t streams = 1;
  uint32_t suggestedBufferSize = 0;
  uint32_t width = 640;   // 默认宽度
  uint32_t height = 480;  // 默认高度
  uint8_t reserved[16] = {0};
  
  uint32_t codec = 0x47504A4D;  // MJPG
  uint32_t streamFlags = 0;
  uint32_t priority = 0;
  uint32_t timeScale = 1000000;
  uint32_t sampleRate = 1000000;
  uint32_t length = 0;  // 帧数占位符
  uint32_t quality = 10000;
  uint32_t sampleSize = 0;
  uint8_t frameRect[16] = {0};
  
  uint32_t biSize = 40;
  uint16_t biPlanes = 1;
  uint16_t bitCount = 24;
  uint32_t compression = 0x47504A4D;  // MJPG
  uint32_t imageSize = 0;
  uint32_t xPelsPerMeter = 0;
  uint32_t yPelsPerMeter = 0;
  uint32_t colorsUsed = 0;
  uint32_t colorsImportant = 0;
  
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
  
  // 更严格的文件写入检查
  if (!videoFile) {
    Serial.println("错误: 文件句柄无效，无法写入AVI头部");
    return;
  }
  
  if (!videoFile.availableForWrite()) {
    Serial.println("错误: 文件不可写，无法写入AVI头部");
    return;
  }
  
  // 写入AVI头部
  size_t written = videoFile.write(header, AVI_HEADER_SIZE);
  if (written != AVI_HEADER_SIZE) {
    Serial.printf("错误: AVI头部写入不完整。应写入: %d bytes, 实际写入: %d bytes\n", AVI_HEADER_SIZE, written);
    return;
  }
  
  // 填充头部到512字节
  uint8_t padding[512 - AVI_HEADER_SIZE];
  memset(padding, 0, sizeof(padding));
  written = videoFile.write(padding, sizeof(padding));
  if (written != sizeof(padding)) {
    Serial.printf("错误: AVI头部填充写入不完整。应写入: %d bytes, 实际写入: %d bytes\n", sizeof(padding), written);
    return;
  }
  
  // 强制刷新确保数据写入
  videoFile.flush();
  delay(100);
  
  Serial.printf("AVI头部写入成功，文件大小: %lu bytes\n", videoFile.size());
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

#ifndef WEB_CONFIG_H
#define WEB_CONFIG_H

#include <Arduino.h>

// 启动 AP 模式 + 配网服务器 (阻塞直到用户配置完成)
// 返回: true=用户已配置, false=超时未配置
bool WebConfig_Start();

// 从 NVS 加载 WiFi 凭据
// ssid_out/pswd_out 缓冲区至少 32 字节
bool WebConfig_LoadWiFi(char *ssid_out, char *pswd_out);

// 保存 WiFi 凭据到 NVS
void WebConfig_SaveWiFi(const char *ssid, const char *pswd);

#endif

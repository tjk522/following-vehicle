/**
 * @file WebConfig.cpp
 * @brief WiFi 配网 — ESP32 开启 AP 热点 + Captive Portal 网页配置
 *
 * 使用场景：
 *   小车开机后连不上 WiFi（比如换了环境、热点名称变了），
 *   自动开启 AP 模式：创建一个名为 "FishBot-Config" 的 WiFi 热点，
 *   手机连上后打开任意网页都会跳转到配网页面。
 *
 * Captive Portal 原理：
 *   DNS 服务器拦截所有域名解析请求，全部返回 ESP32 的 IP (192.168.4.1)，
 *   这样手机无论打开什么网址都会被重定向到配网页。
 *
 * NVS (Non-Volatile Storage)：
 *   ESP32 的持久化存储，掉电不丢失。用来保存用户配置的 WiFi 凭据。
 */

#include "WebConfig.h"
#include <WiFi.h>
#include <WebServer.h>   // ESP32 Arduino 内置的 HTTP 服务器
#include <DNSServer.h>   // DNS 服务器（Captive Portal 的核心）
#include <Preferences.h> // NVS 读写

// AP 热点配置
static const char* AP_SSID = "FishBot-Config";
static const char* AP_PASS = "12345678";

static WebServer server(80);        // HTTP 服务器监听 80 端口
static DNSServer dnsServer;         // DNS 服务器用于 Captive Portal
static const byte DNS_PORT = 53;

static bool config_done = false;    // 用户是否已完成配置

// =====================================================================
// 配网主页面（移动端友好的 HTML）
// =====================================================================
static const char CONFIG_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>FishBot WiFi</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,sans-serif;background:#1a1a2e;color:#eee;
     display:flex;justify-content:center;align-items:center;min-height:100vh;padding:16px}
.card{background:#16213e;border-radius:16px;padding:24px;width:100%;
      max-width:360px;box-shadow:0 8px 32px rgba(0,0,0,.3)}
h1{font-size:20px;text-align:center;color:#e94560;margin-bottom:8px}
.sub{text-align:center;font-size:13px;color:#888;margin-bottom:20px}
label{font-size:13px;color:#aaa;display:block;margin-top:12px;margin-bottom:4px}
input{width:100%;padding:10px 12px;border:1px solid #333;border-radius:8px;
      background:#1a1a2e;color:#eee;font-size:15px}
input:focus{outline:none;border-color:#e94560}
button{width:100%;padding:12px;margin-top:20px;background:#e94560;color:#fff;
       border:none;border-radius:8px;font-size:16px;font-weight:600;cursor:pointer}
.note{font-size:11px;color:#666;text-align:center;margin-top:16px}
</style>
</head>
<body><div class="card">
<h1>FishBot</h1><div class="sub">WiFi 配网</div>
<form action="/save" method="POST">
<label>WiFi 名称 (SSID)</label>
<input name="ssid" placeholder="你的 WiFi 名称" required>
<label>WiFi 密码</label>
<input name="pswd" type="password" placeholder="WiFi 密码">
<button type="submit">保存并连接</button>
</form>
<div class="note">配网成功后小车将自动重启</div>
</div></body>
</html>
)rawliteral";

// =====================================================================
// 保存成功页面
// =====================================================================
static const char SAVED_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="zh"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>OK</title>
<style>
body{font-family:-apple-system,sans-serif;background:#1a1a2e;color:#eee;
     display:flex;justify-content:center;align-items:center;min-height:100vh}
.card{background:#16213e;border-radius:16px;padding:32px;text-align:center}
h1{color:#4ecca3;font-size:22px;margin-bottom:12px}
p{color:#888;font-size:14px}
</style>
</head><body><div class="card">
<h1>配网成功!</h1><p>小车正在重启...</p>
</div></body></html>
)rawliteral";

// HTTP 路由：首页 → 配网页
void handleRoot() { server.send_P(200, "text/html", CONFIG_HTML); }

// HTTP 路由：POST /save → 保存 WiFi 凭据到 NVS
void handleSave() {
    String ssid = server.arg("ssid");
    String pswd = server.arg("pswd");
    if (ssid.length() > 0 && ssid.length() <= 32) {
        WebConfig_SaveWiFi(ssid.c_str(), pswd.c_str());
        config_done = true;
        server.send_P(200, "text/html", SAVED_HTML);
    }
}

// HTTP 路由：所有其他路径 → 重定向到首页（实现 Captive Portal）
void handleNotFound() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
}

/**
 * @brief 启动配网模式（阻塞直到配置完成或超时）
 * @return true=已配置, false=超时未配置
 *
 * 流程：
 *   1. 开启 AP 模式（WiFi 热点 FishBot-Config）
 *   2. 启动 DNS 服务器（所有域名 → 192.168.4.1）
 *   3. 启动 HTTP 服务器（配网页面）
 *   4. 等待用户在手机上填写并提交 → 保存 NVS
 *   5. 超过 3 分钟未配置则超时退出
 */
bool WebConfig_Start() {
    config_done = false;

    // 切换到 AP 模式
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    delay(200);

    // 固定 IP 为 192.168.4.1（网关/DNS 都指向自己）
    IPAddress apIP(192, 168, 4, 1);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

    // DNS：把所有域名请求都解析到 192.168.4.1
    dnsServer.start(DNS_PORT, "*", apIP);

    // HTTP 路由注册
    server.on("/", HTTP_GET, handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.onNotFound(handleNotFound);
    server.begin();

    Serial.println("WebConfig AP started: 192.168.4.1");
    Serial.printf("Connect to '%s' with password '%s'\n", AP_SSID, AP_PASS);

    unsigned long start_ms = millis();
    const unsigned long TIMEOUT_MS = 180000;  // 3 分钟超时

    // 事件循环：处理 DNS + HTTP 请求
    while (!config_done && (millis() - start_ms < TIMEOUT_MS)) {
        dnsServer.processNextRequest();
        server.handleClient();
        delay(10);
    }

    // 清理：关闭服务器和 AP
    server.stop();
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(500);

    return config_done;
}

/**
 * @brief 从 NVS 加载保存的 WiFi 凭据
 * @return true=加载成功, false=没有保存的凭据
 */
bool WebConfig_LoadWiFi(char *ssid_out, char *pswd_out) {
    Preferences prefs;
    if (!prefs.begin("wifi", true)) return false;  // true = 只读模式
    String ssid = prefs.getString("ssid", "");
    String pswd = prefs.getString("pswd", "");
    prefs.end();
    if (ssid.length() == 0) return false;
    strncpy(ssid_out, ssid.c_str(), 31);
    strncpy(pswd_out, pswd.c_str(), 31);
    return true;
}

/**
 * @brief 保存 WiFi 凭据到 NVS（掉电不丢失）
 */
void WebConfig_SaveWiFi(const char *ssid, const char *pswd) {
    Preferences prefs;
    prefs.begin("wifi", false);  // false = 读写模式
    prefs.putString("ssid", ssid);
    prefs.putString("pswd", pswd);
    prefs.end();
}

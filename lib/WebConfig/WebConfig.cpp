#include "WebConfig.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

static const char* AP_SSID = "FishBot-Config";
static const char* AP_PASS = "12345678";
static const byte DNS_PORT = 53;

static WebServer server(80);
static DNSServer dnsServer;

// 配网 HTML 页面 (移动端友好)
static const char CONFIG_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>FishBot WiFi 配网</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,system-ui,sans-serif;background:#1a1a2e;color:#eee;display:flex;justify-content:center;align-items:center;min-height:100vh;padding:16px}
.card{background:#16213e;border-radius:16px;padding:24px;width:100%;max-width:360px;box-shadow:0 8px 32px rgba(0,0,0,.3)}
h1{font-size:20px;text-align:center;margin-bottom:8px;color:#0f3460}
.sub{text-align:center;font-size:13px;color:#888;margin-bottom:20px}
label{font-size:13px;color:#aaa;display:block;margin-top:12px;margin-bottom:4px}
input{width:100%;padding:10px 12px;border:1px solid #333;border-radius:8px;background:#1a1a2e;color:#eee;font-size:15px}
input:focus{outline:none;border-color:#e94560}
button{width:100%;padding:12px;margin-top:20px;background:#e94560;color:#fff;border:none;border-radius:8px;font-size:16px;font-weight:600;cursor:pointer}
button:active{opacity:.8}
.note{font-size:11px;color:#666;text-align:center;margin-top:16px}
</style>
</head>
<body>
<div class="card">
<h1>FishBot</h1>
<div class="sub">WiFi 配网</div>
<form action="/save" method="POST">
<label>WiFi 名称 (SSID)</label>
<input name="ssid" placeholder="你的 WiFi 名称" required>
<label>WiFi 密码</label>
<input name="pswd" type="password" placeholder="WiFi 密码">
<button type="submit">保存并连接</button>
</form>
<div class="note">配网成功后小车将自动重启</div>
</div>
</body>
</html>
)rawliteral";

static const char SAVED_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>保存成功</title>
<style>
body{font-family:-apple-system,system-ui,sans-serif;background:#1a1a2e;color:#eee;display:flex;justify-content:center;align-items:center;min-height:100vh}
.card{background:#16213e;border-radius:16px;padding:32px;text-align:center}
h1{color:#4ecca3;font-size:22px;margin-bottom:12px}
p{color:#888;font-size:14px}
</style>
</head>
<body>
<div class="card">
<h1>配网成功!</h1>
<p>小车正在重启...</p>
</div>
</body>
</html>
)rawliteral";

static bool config_done = false;

void handleRoot() {
    server.send_P(200, "text/html", CONFIG_HTML);
}

void handleSave() {
    String ssid = server.arg("ssid");
    String pswd = server.arg("pswd");
    if (ssid.length() > 0 && ssid.length() <= 32) {
        WebConfig_SaveWiFi(ssid.c_str(), pswd.c_str());
        config_done = true;
        server.send_P(200, "text/html", SAVED_HTML);
    } else {
        server.send(400, "text/plain", "SSID invalid");
    }
}

void handleNotFound() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
}

bool WebConfig_Start() {
    config_done = false;

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    delay(200);

    IPAddress apIP(192, 168, 4, 1);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

    dnsServer.start(DNS_PORT, "*", apIP);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.onNotFound(handleNotFound);
    server.begin();

    Serial.println("WebConfig AP started: 192.168.4.1");
    Serial.printf("Connect to '%s' with password '%s'\n", AP_SSID, AP_PASS);

    unsigned long start_ms = millis();
    const unsigned long TIMEOUT_MS = 180000; // 3分钟超时

    while (!config_done && (millis() - start_ms < TIMEOUT_MS)) {
        dnsServer.processNextRequest();
        server.handleClient();
        delay(10);
    }

    server.stop();
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(500);

    return config_done;
}

bool WebConfig_LoadWiFi(char *ssid_out, char *pswd_out) {
    Preferences prefs;
    if (!prefs.begin("wifi", true)) return false;
    String ssid = prefs.getString("ssid", "");
    String pswd = prefs.getString("pswd", "");
    prefs.end();
    if (ssid.length() == 0) return false;
    strncpy(ssid_out, ssid.c_str(), 31);
    strncpy(pswd_out, pswd.c_str(), 31);
    return true;
}

void WebConfig_SaveWiFi(const char *ssid, const char *pswd) {
    Preferences prefs;
    prefs.begin("wifi", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pswd", pswd);
    prefs.end();
}

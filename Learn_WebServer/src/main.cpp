#include <Arduino.h>
#include <WiFi.h>
#include "SPIFFS.h"
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <Preferences.h> // Thư viện để lưu trữ dữ liệu

// --- CẤU HÌNH ADMIN ---
const char* adminUser = "admin";
const char* adminPass = "123456"; // <-- THAY ĐỔI MẬT KHẨU NÀY!

#define RX2_PIN 16
#define TX2_PIN 17

const char* ssid = "wifi";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences preferences; // Đối tượng để quản lý lưu trữ

typedef struct {
    int p, percent;
    char c[10];
    float v, i, t;
} PinData;

int pin_lap = 5, pin_thao = 5;

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if(type == WS_EVT_CONNECT){
        Serial.printf("WebSocket client #%u connected\n", client->id());
    } else if(type == WS_EVT_DISCONNECT){
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
    } else if(type == WS_EVT_DATA){
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT){
            data[len] = 0;
            String message = (char*)data;
            Serial.printf("Received message from client #%u: %s\n", client->id(), message.c_str());

            StaticJsonDocument<256> doc;
            String response;

            if (message == "getSwapSlots") {
                doc["type"] = "swapSlots";
                doc["full"] = (pin_thao != 5) ? String(pin_thao) : "đợi";
                doc["empty"] = (pin_lap != 5) ? String(pin_lap) : "đợi";
            } else if (message == "getPickupSlot") {
                doc["type"] = "pickupSlot";
                doc["slot"] = (pin_thao != 5) ? String(pin_thao) : "đợi";
            } 
            // Xử lý yêu cầu lấy dữ liệu thống kê từ trang admin
            else if (message == "getAdminStats") {
                doc["type"] = "adminStats";
                doc["swapCount"] = preferences.getUInt("swap", 0);
                doc["takeCount"] = preferences.getUInt("take", 0);
            }
            
            if (!doc.isNull()) {
                serializeJson(doc, response);
                client->text(response); // Chỉ gửi trả lời cho client đã yêu cầu
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);

    // Khởi tạo Preferences
    preferences.begin("admin-stats", false); // "admin-stats" là tên namespace, false = read/write
    
    SPIFFS.begin(true);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // --- CÁC ROUTE CŨ ---
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ request->redirect("/dashboard.html"); });
    server.on("/dashboard.html", HTTP_GET, [](AsyncWebServerRequest *request){ request->send(SPIFFS, "/dashboard.html", "text/html"); Serial2.println("back:");});
    server.on("/doipin.html", HTTP_GET, [](AsyncWebServerRequest *request){ Serial2.println("doipin:"); request->send(SPIFFS, "/doipin.html", "text/html"); });
    server.on("/laypin.html", HTTP_GET, [](AsyncWebServerRequest *request){ Serial2.println("laypin:"); request->send(SPIFFS, "/laypin.html", "text/html"); });
    server.on("/lappin.html", HTTP_GET, [](AsyncWebServerRequest *request){ Serial2.println("lappin:"); request->send(SPIFFS, "/lappin.html", "text/html"); });
    server.on("/pindetail.html", HTTP_GET, [](AsyncWebServerRequest *request){ request->send(SPIFFS, "/pindetail.html", "text/html"); });

    // --- CÁC ROUTE MỚI CHO TRANG ADMIN ---
    server.on("/login.html", HTTP_GET, [](AsyncWebServerRequest *request){ request->send(SPIFFS, "/login.html", "text/html"); });
    server.on("/admin.html", HTTP_GET, [](AsyncWebServerRequest *request){ request->send(SPIFFS, "/admin.html", "text/html"); });
    
    // Route xử lý việc đăng nhập
    server.on("/login", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (request->hasParam("username", true) && request->hasParam("password", true)) {
            String username = request->getParam("username", true)->value();
            String password = request->getParam("password", true)->value();
            if (username.equals(adminUser) && password.equals(adminPass)) {
                request->send(200, "text/plain", "OK");
            } else {
                request->send(401, "text/plain", "Unauthorized");
            }
        } else {
            request->send(400, "text/plain", "Bad Request");
        }
    });

    server.begin();
}

void loop() {
    if (Serial2.available()) {
        String dataString = Serial2.readStringUntil('\n');
        dataString.trim();
        if (dataString.length() > 0) {
            Serial.print("Received from STM32: ");
            Serial.println(dataString);
            
            StaticJsonDocument<384> doc;
            String output;

            if (dataString.startsWith("P:")) {
                PinData pin;
                if (sscanf(dataString.c_str(), "P:%d,%%:%d,C:%[^,],V:%f,I:%f,T:%f", &pin.p, &pin.percent, pin.c, &pin.v, &pin.i, &pin.t) == 6) {
                    doc["type"] = "pinData";
                    doc["id"] = pin.p;
                    doc["percent"] = pin.percent;
                    if (strcmp(pin.c, "T")==0) doc["status"]="Trống"; else if (strcmp(pin.c, "S")==0) doc["status"]="Sạc"; else if (strcmp(pin.c, "D")==0) doc["status"]="Đầy"; else if (strcmp(pin.c, "N")==0) doc["status"]="Nhiệt Độ Cao"; else if (strcmp(pin.c, "L")==0) doc["status"]="Lỗi"; else doc["status"]="Không xác định";
                    doc["voltage"] = pin.v;
                    doc["current"] = pin.i;
                    doc["temperature"] = pin.t;
                }
            } else if (dataString.startsWith("doipin:")) {
                sscanf(dataString.c_str(), "doipin:%d,laypin:%d", &pin_lap, &pin_thao);
                doc["type"] = "swapSlotsUpdate";
                doc["full"] = pin_thao;
                doc["empty"] = pin_lap;
                // TĂNG VÀ LƯU BỘ ĐẾM ĐỔI PIN
                unsigned int swapCount = preferences.getUInt("swap", 0) + 1;
                preferences.putUInt("swap", swapCount);
            } else if (dataString.startsWith("laypin:")) {
                sscanf(dataString.c_str(), "laypin:%d", &pin_thao);
                doc["type"] = "pickupSlotUpdate";
                doc["slot"] = pin_thao;
                // TĂNG VÀ LƯU BỘ ĐẾM LẤY PIN
                unsigned int takeCount = preferences.getUInt("take", 0) + 1;
                preferences.putUInt("take", takeCount);
            } 
            
            if (!doc.isNull()) {
                serializeJson(doc, output);
                ws.textAll(output); 
            }
        }
    }
    ws.cleanupClients();
}
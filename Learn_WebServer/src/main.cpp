#include <Arduino.h>
#include <WiFi.h>
#include "SPIFFS.h"
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

#define RX2_PIN 16
#define TX2_PIN 17

// --- THAY ĐỔI THÔNG TIN WIFI CỦA BẠN ---
const char* ssid = "wifi";
const char* password = "12345678";

// --- KHAI BÁO CÁC ĐỐI TƯỢNG SERVER ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- BIẾN TOÀN CỤC ĐỂ LƯU TRẠNG THÁI ĐĂNG NHẬP ---
String loggedInUser = ""; // Lưu username của người đang đăng nhập
String loggedInUserFullName = ""; // Lưu tên đầy đủ

typedef struct {
    int p;          // Tên 'p' khớp với 'P:' trong chuỗi và 'p' trong JSON
    int percent;
    char c[10];     // Tên 'c' khớp với 'C:' trong chuỗi và 'c' trong JSON
    float v;        // Tên 'v' khớp với 'V:' trong chuỗi và 'v' trong JSON
    float i;
    float t;
} PinData;

// --- HÀM XỬ LÝ SỰ KIỆN WEBSOCKET (ĐÃ NÂNG CẤP) ---
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      // Client vừa kết nối
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      // Client vừa ngắt kết nối
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      { // Bắt đầu khối lệnh mới cho case này
        // Nhận được dữ liệu từ client
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
          data[len] = 0; // Thêm ký tự kết thúc chuỗi
          String message = (char*)data;
          Serial.printf("Received message from client #%u: %s\n", client->id(), message.c_str());

          // === LOGIC XỬ LÝ YÊU CẦU TỪ CLIENT ===
          if (message == "getSwapSlots") {
            StaticJsonDocument<128> doc;
            doc["type"] = "swapSlots";
            doc["full"] = random(1, 5); // Mô phỏng khay pin đầy
            doc["empty"] = random(5, 9); // Mô phỏng khay pin trống
            
            String response;
            serializeJson(doc, response);
            client->text(response); // Gửi phản hồi lại cho chỉ client này
          } 
          else if (message == "getPickupSlot") {
            StaticJsonDocument<128> doc;
            doc["type"] = "pickupSlot";
            doc["slot"] = random(1, 9); // Mô phỏng khay lấy pin
            
            String response;
            serializeJson(doc, response);
            client->text(response); // Gửi phản hồi lại cho chỉ client này
          }
        }
      } // Kết thúc khối lệnh
      break;
    case WS_EVT_ERROR:
      // Có lỗi xảy ra
      Serial.printf("WebSocket client #%u error(%u): %s\n", client->id(), *((uint16_t*)arg), (char*)data);
      break;
  }
}

// --- HÀM XỬ LÝ KHI KHÔNG TÌM THẤY TRANG ---
void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Trang không tồn tại.");
}

// --- HÀM ĐỌC VÀ THAY THẾ PLACEHOLDER ---
String processor(const String& var){
  if(var == "USER_NAME"){
    return loggedInUserFullName;
  }
  return String();
}

// --- HÀM KHỞI TẠO ---
void setup() {
    Serial.begin(115200);

    Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN); 

    Serial.println("UART2 Initialized Successfully!");

    // 1. KHỞI ĐỘNG HỆ THỐNG FILE SPIFFS
    if(!SPIFFS.begin(true)){
        Serial.println("Lỗi khi khởi động SPIFFS");
        return;
    }

    // 2. KẾT NỐI WIFI
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());

    // 3. THIẾT LẬP WEBSOCKET
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // 4. ĐỊNH NGHĨA CÁC ĐƯỜNG DẪN (ROUTES) HTTP
    
    // Phục vụ các file HTML tĩnh từ SPIFFS
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/trangchu.html", "text/html");
    });
    server.on("/dangnhap.html", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/dangnhap.html", "text/html");
    });
    server.on("/dangky.html", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/dangky.html", "text/html");
    });
    server.on("/doipin.html", HTTP_GET, [](AsyncWebServerRequest *request){
    if(loggedInUser == ""){
        request->redirect("/dangnhap.html"); // Nếu chưa đăng nhập, đá về trang đăng nhập
    } else {
        request->send(SPIFFS, "/doipin.html", "text/html"); // Đã đăng nhập, cho phép xem trang
    }
    });
    server.on("/laypin.html", HTTP_GET, [](AsyncWebServerRequest *request){
        if(loggedInUser == ""){
            request->redirect("/dangnhap.html"); // Nếu chưa đăng nhập, đá về trang đăng nhập
        } else {
            request->send(SPIFFS, "/laypin.html", "text/html"); // Đã đăng nhập, cho phép xem trang
        }
    });

    // BỘ XỬ LÝ GET CHO DASHBOARD (QUAN TRỌNG)
    server.on("/dashboard.html", HTTP_GET, [](AsyncWebServerRequest *request){
        if(loggedInUser == ""){
            request->redirect("/dangnhap.html");
            return;
        }

        // 1. Mở file dashboard.html
        File file = SPIFFS.open("/dashboard.html", "r");
        if (!file) {
            request->send(404, "text/plain", "File dashboard.html khong tim thay.");
            return;
        }

        // 2. Đọc toàn bộ nội dung file vào một biến String
        String htmlContent = file.readString();
        file.close();

        // 3. Tự thay thế placeholder
        htmlContent.replace("%USER_NAME%", loggedInUserFullName);

        // 4. Gửi chuỗi String đã được sửa đổi đi
        request->send(200, "text/html", htmlContent);
    });

    // Xử lý yêu cầu POST từ form đăng ký
    server.on("/register", HTTP_POST, [](AsyncWebServerRequest *request){
        String fullname, phone, username, password;
        if(request->hasParam("fullname", true)) fullname = request->getParam("fullname", true)->value();
        if(request->hasParam("phone", true)) phone = request->getParam("phone", true)->value();
        if(request->hasParam("username", true)) username = request->getParam("username", true)->value();
        if(request->hasParam("password", true)) password = request->getParam("password", true)->value();

        File file = SPIFFS.open("/users.json", "r");
        StaticJsonDocument<1024> doc; 
        
        if (file && file.size() > 0) {
            deserializeJson(doc, file);
        } else {
            doc.to<JsonArray>();
        }
        file.close();

        JsonArray array = doc.as<JsonArray>();
        for(JsonObject user : array) {
            if (user["username"] == username) {
                request->send(400, "text/plain", "Ten dang nhap da ton tai. Vui long chon ten khac.");
                return;
            }
        }

        JsonObject newUser = array.createNestedObject();
        newUser["fullname"] = fullname;
        newUser["phone"] = phone;
        newUser["username"] = username;
        newUser["password"] = password;

        file = SPIFFS.open("/users.json", "w");
        if (!file) {
            request->send(500, "text/plain", "Loi khong the mo file de luu.");
            return;
        }
        serializeJson(doc, file);
        file.close();

        Serial.println("Da them nguoi dung moi: " + username);
        request->redirect("/dangnhap.html");
    });

    // Xử lý yêu cầu POST từ form đăng nhập
    server.on("/login", HTTP_POST, [](AsyncWebServerRequest *request){
        String username, password;
        if(request->hasParam("username", true)) username = request->getParam("username", true)->value();
        if(request->hasParam("password", true)) password = request->getParam("password", true)->value();

        File file = SPIFFS.open("/users.json", "r");
        if (!file) {
            request->send(400, "text/plain", "Chua co nguoi dung nao duoc dang ky.");
            return;
        }

        StaticJsonDocument<1024> doc;
        DeserializationError error = deserializeJson(doc, file);
        file.close();

        if (error) {
            request->send(500, "text/plain", "Loi doc du lieu nguoi dung.");
            return;
        }

        JsonArray array = doc.as<JsonArray>();
        bool foundUser = false;
        for(JsonObject user : array) {
            if (user["username"] == username && user["password"] == password) {
                loggedInUser = user["username"].as<String>();
                loggedInUserFullName = user["fullname"].as<String>();
                foundUser = true;
                break;
            }
        }

        if (foundUser) {
            Serial.println("Dang nhap thanh cong cho user: " + username);
            request->redirect("/dashboard.html"); // Chuyển hướng khi thành công
        } else {
            Serial.println("Dang nhap that bai cho user: " + username);
            loggedInUser = "";
            loggedInUserFullName = "";
            request->redirect("/dangnhap.html"); // Chuyển hướng khi thất bại
        }
    });
    
    // Xử lý yêu cầu đăng xuất
    server.on("/logout", HTTP_GET, [](AsyncWebServerRequest *request){
        Serial.println("User " + loggedInUser + " da dang xuat.");
        loggedInUser = "";
        loggedInUserFullName = "";
        request->redirect("/");
    });

    // 5. ĐĂNG KÝ HÀM XỬ LÝ KHI KHÔNG TÌM THẤY TRANG
    server.onNotFound(notFound);

    // 6. BẮT ĐẦU CHẠY SERVER
    server.begin();
    
    Serial.println("HTTP server started");
}
// --- VÒNG LẶP CHÍNH ---
void loop() {
    if (Serial2.available()) {
        String dataString = Serial2.readStringUntil('\n');
        dataString.trim();

        if (dataString.length() > 0) {
            Serial.print("Received from UART2: ");
            Serial.println(dataString);

            PinData pin; 

            int itemsParsed = sscanf(dataString.c_str(),
                   "P:%d,%%:%d,C:%[^,],V:%f,I:%f,T:%f",
                   &pin.p,         // Sửa thành &pin.p
                   &pin.percent,
                   pin.c,          // Sửa thành pin.c
                   &pin.v,         // Sửa thành &pin.v
                   &pin.i,         // Sửa thành &pin.i
                   &pin.t);        // Sửa thành &pin.t

            if (itemsParsed == 6) {
                StaticJsonDocument<256> doc;
                doc["id"] = pin.p;
                doc["percent"] = pin.percent;

                if (strcmp(pin.c, "T") == 0) {
                    doc["status"] = "Trống";
                }
                else if (strcmp(pin.c, "S") == 0) {
                    doc["status"] = "Sạc";
                }
                else if (strcmp(pin.c, "D") == 0) {
                    doc["status"] = "Đầy";
                }
                else if (strcmp(pin.c, "N") == 0) {
                    doc["status"] = "Nhiệt Độ Cao";
                }
                else if (strcmp(pin.c, "L") == 0) {
                    doc["status"] = "Lỗi";
                }
                else if (strcmp(pin.c, "R") == 0) {
                    doc["status"] = "Không xác định";
                }
                // Thêm một trường hợp else để xử lý các ký tự không mong muốn
                else {
                    doc["status"] = "Trạng thái lạ";
                }

                String output;
                serializeJson(doc, output);
                
                Serial.print("Sending via WebSocket: ");
                Serial.println(output);

                ws.textAll(output);
            } else {
                Serial.print("Failed to parse data string. Items parsed: ");
                Serial.println(itemsParsed);
            }
        }
    }
}
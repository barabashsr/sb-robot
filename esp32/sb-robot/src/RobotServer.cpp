#include "RobotServer.h"

RobotServer::RobotServer(controllerState& state, BalanceController& controller) 
    : _state(state), _controller(controller), _httpServer(80    ), _ws("/ws") {}

void RobotServer::begin() {
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println(WiFi.localIP());

    _ws.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
        handleWebSocketEvent(server, client, type, arg, data, len);
    });

    _httpServer.addHandler(&_ws);
    _httpServer.begin();
}

void RobotServer::handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            clientAuthenticated = false;
            client->text("Welcome! Please authenticate.");
            break;
        case WS_EVT_DISCONNECT:
            clientAuthenticated = false;
            Serial.println("Client disconnected");
            break;
        case WS_EVT_DATA:
        {
            // Use block scope to avoid initialization issues
            String message = reinterpret_cast<const char*>(data);
            message = message.substring(0, len);

            if (message.startsWith("token:")) {
                String token = message.substring(6);
                if (token == validToken) {
                    clientAuthenticated = true;
                    client->text("Authenticated successfully");
                    Serial.println("Client authenticated successfully.");
                } else {
                    client->text("Invalid token");
                    Serial.println("Invalid token provided.");
                }
            } else if (clientAuthenticated) {
                if (message == "init") {
                    // Handle init command
                    client->text("Initialization command received");
                } else if (message == "reset") {
                    // Handle reset command
                    _controller.resetMotors();
                    client->text("Reset command received");
                } else if (message == "state") {
                    // Handle state command
                    DynamicJsonDocument doc(2048);
                    doc["RW_position"] = (double)_state.positionA;
                    doc["LW_position"] = (double)_state.positionB;
                    doc["RW_velocity"] = (double)_state.speedA;
                    doc["LW_velocity"] = (double)_state.speedB;
                    String json;
                    serializeJson(doc, json);
                    client->text(json);
                } else if (message == "ping") {
                    // Handle ping command
                    client->text("Pong!");
                } else {
                    client->text("Unknown command");
                }
            } else {
                client->text("Please authenticate first");
                Serial.println("Client not authenticated.");
            }
            break;
        }
        case WS_EVT_PING:
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}



#ifndef ROBOTSERVER_H
#define ROBOTSERVER_H

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "utils.h"
#include "BalanceController.h"


class RobotServer {
public:
    RobotServer(controllerState& state, BalanceController& controller);
    void begin();
    void handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

private:
    BalanceController& _controller;
    controllerState& _state;
    AsyncWebServer _httpServer;
    AsyncWebSocket _ws;
    const char* validToken = "your_secret_token"; // Replace with your token
    bool clientAuthenticated = false;
};

#endif // ROBOTSERVER_H

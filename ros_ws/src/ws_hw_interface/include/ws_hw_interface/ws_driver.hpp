#ifndef WS_DRIVER_HPP
#define WS_DRIVER_HPP

#include <websocketpp/config/asio.hpp>
#include <websocketpp/client.hpp>
#include <string>

typedef websocketpp::client<websocketpp::config::asio> client;

class ws_driver {
public:
    enum class Command {
        INIT,
        RESET,
        GET_POSE_A,
        GET_POSE_B,
        GET_VEL_A,
        GET_VEL_B
    };

    ws_driver(const std::string& url, const std::string& token);
    bool connect();
    bool disconnect();
    double getPoseA();
    double getPoseB();
    double getVelA();
    double getVelB();
    bool init();

private:
    std::string url_;
    std::string token_;
    client client_;
    websocketpp::connection_hdl hdl_;
    bool connected_ = false;
    std::string response_;
    void sendCommand(const std::string& cmd);
    void handleResponse();
    void onOpen(websocketpp::connection_hdl hdl);
    void onClose(websocketpp::connection_hdl hdl);
};

#endif // WS_DRIVER_HPP

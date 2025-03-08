#ifndef WS_HW_INTERFACE__WS_DRIVER_HPP
#define WS_HW_INTERFACE__WS_DRIVER_HPP

#include <websocketpp/config/asio.hpp>
#include <websocketpp/client.hpp>
#include <string>
#include <mutex>


typedef websocketpp::client<websocketpp::config::asio> client;
namespace ws_hw_interface
{

class WS_Driver {
public:
    enum class Command {
        INIT,
        RESET,
        GET_POSE_A,
        GET_POSE_B,
        GET_VEL_A,
        GET_VEL_B
    };

    WS_Driver();
    ~WS_Driver();
    //ws_driver(const std::string& url, const std::string& token);
    bool connect();
    bool disconnect();
    void begin(const std::string& url, const std::string& token);
    double getPoseA();
    double getPoseB();
    double getVelA();
    double getVelB();
    bool init();
    void pollClient();
    bool connected() { return connected_; }
    bool authenticated() { return authenticated_; }
    bool updateState(); // New method to update the state

private:
    std::string url_;
    std::string token_;
    client client_;
    websocketpp::connection_hdl hdl_;
    bool connected_ = false;
    std::string response_;
    void sendCommand(const std::string& cmd);
    void handleResponse(websocketpp::connection_hdl hdl, client::message_ptr msg);
    void onOpen(websocketpp::connection_hdl hdl);
    void onClose(websocketpp::connection_hdl hdl);
    bool authenticated_ = false;
    
    // State variables
    double positionA_ = 0.0;
    double positionB_ = 0.0;
    double velocityA_ = 0.0;
    double velocityB_ = 0.0;
    std::mutex stateMutex_; // To protect state variables during updates
};

}

#endif // WS_DRIVER_HPP

#include "ws_hw_interface/ws_driver.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace ws_hw_interface
{
// Constructor
WS_Driver::WS_Driver() 
: url_(""), token_("")
{};

void WS_Driver::begin(const std::string& url, const std::string& token)

{
    url_ = url;
    token_ = token;   
    
    client_.clear_access_channels(websocketpp::log::alevel::all);
    client_.clear_error_channels(websocketpp::log::elevel::all);
    client_.init_asio();
    client_.set_open_handler(websocketpp::lib::bind(&WS_Driver::onOpen, this, websocketpp::lib::placeholders::_1));
    client_.set_close_handler(websocketpp::lib::bind(&WS_Driver::onClose, this, websocketpp::lib::placeholders::_1));
    client_.set_message_handler(websocketpp::lib::bind(&WS_Driver::handleResponse, this, 
    websocketpp::lib::placeholders::_1, websocketpp::lib::placeholders::_2));
}

WS_Driver::~WS_Driver() {
    disconnect();
}
    


// Connect to the WebSocket server
bool WS_Driver::connect() {
           
    try {
        websocketpp::lib::error_code ec;
        client::connection_ptr con = client_.get_connection(url_, ec);
        if (ec) {
            std::cerr << "Connect initialization error: " << ec.message() << std::endl;
            return false;
        }
        hdl_ = con->get_handle();
        client_.connect(con);
        
        // Start the client in a separate thread
        std::thread t([this]() {
            try {
                client_.run();
            } catch (const std::exception& e) {
                std::cerr << "WebSocket client run error: " << e.what() << std::endl;
            }
        });
        t.detach();
        
        // Wait for connection to establish
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Authenticate
        sendCommand("token:" + token_);
        
        // Wait for authentication response
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        return authenticated_;
    } catch (const websocketpp::exception& e) {
        std::cerr << "Connection failed: " << e.what() << std::endl;
        return false;
    }
}

// Disconnect from the WebSocket server
bool WS_Driver::disconnect() {
    try {
        websocketpp::lib::error_code ec;
        client_.close(hdl_, websocketpp::close::status::normal, "", ec);
        if (ec) {
            std::cerr << "Error closing connection: " << ec.message() << std::endl;
            return false;
        }
        connected_ = false;
        authenticated_ = false;
        return true;
    } catch (const websocketpp::exception& e) {
        std::cerr << "Disconnection failed: " << e.what() << std::endl;
        return false;
    }
}

// Update the state by requesting the latest values from the server
bool WS_Driver::updateState() {
    if (!authenticated_) return false;
    
    // Request state
    sendCommand("state");
    
    // Wait for response
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    try {
        // Parse JSON response
        json stateJson = json::parse(response_);
        
        // Update state variables with mutex protection
        std::lock_guard<std::mutex> lock(stateMutex_);
        positionA_ = stateJson["PositionA"].get<double>();
        positionB_ = stateJson["PositionB"].get<double>();
        velocityA_ = stateJson["VelocityA"].get<double>();
        velocityB_ = stateJson["VelocityB"].get<double>();
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing state: " << e.what() << std::endl;
        return false;
    }
}

// Get pose A
double WS_Driver::getPoseA() {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return positionA_;
}

// Get pose B
double WS_Driver::getPoseB() {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return positionB_;
}

// Get velocity A
double WS_Driver::getVelA() {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return velocityA_;
}

// Get velocity B
double WS_Driver::getVelB() {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return velocityB_;
}

// Initialize
bool WS_Driver::init() {
    if (!authenticated_) return false;
    
    sendCommand("init");
    
    // Wait for response
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    return response_ == "Initialization command received";
}

// Poll the client
void WS_Driver::pollClient() {
    // Not needed as we're running the client in a separate thread
}

// Send a command
void WS_Driver::sendCommand(const std::string& cmd) {
    try {
        websocketpp::lib::error_code ec;
        client_.send(hdl_, cmd, websocketpp::frame::opcode::text, ec);
        if (ec) {
            std::cerr << "Error sending message: " << ec.message() << std::endl;
        }
    } catch (const websocketpp::exception& e) {
        std::cerr << "Failed to send command: " << e.what() << std::endl;
    }
}

// Handle response
void WS_Driver::handleResponse(websocketpp::connection_hdl hdl, client::message_ptr msg) {
    response_ = msg->get_payload();
    if (response_ == "Authenticated successfully") {
        authenticated_ = true;
    }
}

// On open handler
void WS_Driver::onOpen(websocketpp::connection_hdl hdl) {
    connected_ = true;
}

// On close handler
void WS_Driver::onClose(websocketpp::connection_hdl hdl) {
    connected_ = false;
    authenticated_ = false;
}

}
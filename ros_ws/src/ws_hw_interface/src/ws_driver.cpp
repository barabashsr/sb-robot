#include "ws_driver.hpp"
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

ws_driver::ws_driver(const std::string& url, const std::string& token) 
    : url_(url), token_(token), client_() {
    client_.init_asio();
    client_.set_access_channels(websocketpp::log::alevel::none);
    client_.set_message_handler([this](websocketpp::connection_hdl hdl, client::message_ptr msg) {
        response_ = msg->get_payload();
    });
    client_.set_open_handler([this](websocketpp::connection_hdl hdl) {
        onOpen(hdl);
    });
    client_.set_close_handler([this](websocketpp::connection_hdl hdl) {
        onClose(hdl);
    });
}

void ws_driver::onOpen(websocketpp::connection_hdl hdl) {
    hdl_ = hdl;
    connected_ = true;
    std::cout << "Connected to server." << std::endl;

    // Authenticate
    client_.send(hdl_, "token:" + token_, websocketpp::frame::opcode::text);
}

void ws_driver::onClose(websocketpp::connection_hdl hdl) {
    connected_ = false;
    std::cout << "Disconnected from server." << std::endl;
}

bool ws_driver::connect() {
    websocketpp::lib::error_code ec;
    client::connection_ptr con = client_.get_connection(url_, ec);
    if (ec) {
        std::cerr << "Could not create connection because: " << ec.message() << std::endl;
        return false;
    }

    client_.connect(con);
    return true;
}

bool ws_driver::disconnect() {
    if (connected_) {
        client_.close(hdl_, websocketpp::close::status::going_away, "Disconnecting");
        return true;
    }
    return false;
}

void ws_driver::sendCommand(const std::string& cmd) {
    if (connected_) {
        client_.send(hdl_, cmd, websocketpp::frame::opcode::text);
        while (true) {
            client_.poll();
            if (!response_.empty()) {
                break;
            }
        }
    }
}

void ws_driver::handleResponse() {
    if (!response_.empty()) {
        std::cout << "Received response: " << response_ << std::endl;
        response_.clear();
    }
}

double ws_driver::getPoseA() {
    if (connected_) {
        sendCommand("state");
        handleResponse();
        try {
            json j = json::parse(response_);
            return j["RW_position"];
        } catch (const json::exception& e) {
            std::cerr << "Error parsing response: " << e.what() << std::endl;
            return 0.0;
        }
    }
    return 0.0;
}

double ws_driver::getPoseB() {
    if (connected_) {
        sendCommand("state");
        handleResponse();
        try {
            json j = json::parse(response_);
            return j["LW_position"];
        } catch (const json::exception& e) {
            std::cerr << "Error parsing response: " << e.what() << std::endl;
            return 0.0;
        }
    }
    return 0.0;
}

double ws_driver::getVelA() {
    if (connected_) {
        sendCommand("state");
        handleResponse();
        try {
            json j = json::parse(response_);
            return j["RW_velocity"];
        } catch (const json::exception& e) {
            std::cerr << "Error parsing response: " << e.what() << std::endl;
            return 0.0;
        }
    }
    return 0.0;
}

double ws_driver::getVelB() {
    if (connected_) {
        sendCommand("state");
        handleResponse();
        try {
            json j = json::parse(response_);
            return j["LW_velocity"];
        } catch (const json::exception& e) {
            std::cerr << "Error parsing response: " << e.what() << std::endl;
            return 0.0;
        }
    }
    return 0.0;
}

bool ws_driver::init() {
    if (connected_) {
        sendCommand("init");
        handleResponse();
        return true;
    }
    return false;
}

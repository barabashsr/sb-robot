#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <websocketpp/client.hpp>
#include <iostream>
#include <string>

typedef websocketpp::client<websocketpp::config::asio_client> client;

class RobotClient {
public:
    RobotClient(const std::string& url) : url_(url), client_() {
        client_.init_asio();
    }

    void connect() {
        websocketpp::lib::error_code ec;
        client_.set_access_channels(websocketpp::log::alevel::none);
        client_.set_message_handler([this](websocketpp::connection_hdl hdl, client::message_ptr msg) {
            std::cout << "Received message: " << msg->get_payload() << std::endl;
        });

        client::connection_ptr con = client_.get_connection(url_, ec);
        if (ec) {
            std::cerr << "Could not create connection because: " << ec.message() << std::endl;
            return;
        }

        client_.connect(con);
        while (client_.get_state() != websocketpp::connection_state::open) {
            client_.poll();
        }

        std::cout << "Connected to server." << std::endl;

        // Authenticate
        client_.send("token:your_secret_token", websocketpp::frame::opcode::text);
        while (true) {
            client_.poll();
            if (client_.get_state() == websocketpp::connection_state::open) {
                break;
            }
        }

        // Wait for authentication response
        while (true) {
            client_.poll();
            if (client_.get_state() == websocketpp::connection_state::open) {
                break;
            }
        }
    }

    void disconnect() {
        client_.close(websocketpp::close::status::going_away, "Disconnecting");
        while (client_.get_state() != websocketpp::connection_state::closed) {
            client_.poll();
        }
        std::cout << "Disconnected from server." << std::endl;
    }

    void getPoseA() {
        client_.send("state", websocketpp::frame::opcode::text);
        while (true) {
            client_.poll();
            if (client_.get_state() == websocketpp::connection_state::open) {
                break;
            }
        }
    }

    void getPoseB() {
        // Since state command returns both poses, this is redundant
        getPoseA();
    }

    void getVelA() {
        // Similar to getPoseA, state command returns both velocities
        getPoseA();
    }

    void getVelB() {
        getPoseA();
    }

    void init() {
        client_.send("init", websocketpp::frame::opcode::text);
        while (true) {
            client_.poll();
            if (client_.get_state() == websocketpp::connection_state::open) {
                break;
            }
        }
    }

private:
    std::string url_;
    client client_;
};

int main() {
    RobotClient client("ws://192.168.1.77:80/ws");
    client.connect();
    client.init();
    client.getPoseA(); // This will print both poses and velocities
    client.disconnect();
    return 0;
}

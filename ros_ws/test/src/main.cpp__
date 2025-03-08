#include "ws_driver.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <csignal>
#include <atomic>

// Global flag for handling Ctrl+C
std::atomic<bool> running(true);

// Signal handler for clean shutdown
void signalHandler(int signal) {
    running = false;
    std::cout << "\nShutting down..." << std::endl;
}

int main() {
    // Set up signal handling for Ctrl+C
    std::signal(SIGINT, signalHandler);
    
    // Create the driver instance
    ws_driver driver("ws://192.168.1.77:80/ws", "your_secret_token");
    
    std::cout << "Connecting to WebSocket server..." << std::endl;
    
    if (driver.connect()) {
        std::cout << "Connected to WebSocket server." << std::endl;
        
        // Wait for authentication
        while (!driver.authenticated() && running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (!running) {
            driver.disconnect();
            return 0;
        }
        
        // Initialize the system
        if (driver.init()) {
            std::cout << "Initialization successful." << std::endl;
        } else {
            std::cout << "Initialization failed." << std::endl;
            driver.disconnect();
            return 1;
        }
        
        // Clear the console and print header
        std::cout << "\033[2J\033[1;1H"; // Clear screen and move cursor to top-left
        std::cout << "Sampling at 50Hz (20ms interval)" << std::endl;
        std::cout << "Press Ctrl+C to exit" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Time(s) | PoseA | PoseB | VelA | VelB" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        
        // Start time for elapsed time calculation
        auto startTime = std::chrono::steady_clock::now();
        
        // Main sampling loop
        while (running) {
            auto loopStart = std::chrono::steady_clock::now();
            
            // Update state
            if (driver.updateState()) {
                // Get current elapsed time
                auto now = std::chrono::steady_clock::now();
                double elapsedSec = std::chrono::duration<double>(now - startTime).count();
                
                // Get all values
                double poseA = driver.getPoseA();
                double poseB = driver.getPoseB();
                double velA = driver.getVelA();
                double velB = driver.getVelB();
                
                // Print values in a single formatted line
                std::cout << "\r" << std::fixed << std::setprecision(3) 
                          << std::setw(7) << elapsedSec << " | "
                          << std::setw(5) << poseA << " | "
                          << std::setw(5) << poseB << " | "
                          << std::setw(4) << velA << " | "
                          << std::setw(4) << velB << std::flush;
            } else {
                std::cout << "\rError updating state                        " << std::flush;
            }
            
            // Calculate time spent in this iteration
            auto loopEnd = std::chrono::steady_clock::now();
            auto loopDuration = std::chrono::duration_cast<std::chrono::milliseconds>(loopEnd - loopStart);
            
            // Sleep for the remaining time to maintain 20ms interval
            auto sleepTime = std::chrono::milliseconds(20) - loopDuration;
            if (sleepTime > std::chrono::milliseconds(0)) {
                std::this_thread::sleep_for(sleepTime);
            }
        }
        
        std::cout << std::endl << "Disconnecting from server..." << std::endl;
        driver.disconnect();
        std::cout << "Disconnected." << std::endl;
    } else {
        std::cerr << "Failed to connect to WebSocket server." << std::endl;
        return 1;
    }
    
    return 0;
}

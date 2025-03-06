#ifndef UTILS_H
#define UTILS_H
#include <Arduino.h>

// #include <memory>

// template<typename T, typename... Args>
// std::unique_ptr<T> make_unique(Args&&... args) {
//     return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
// }

struct pidParams
                {
                    double Kp;
                    double Ki;
                    double Kd;
                    float max;
                    float min;
                    int period;
                    bool direct;
                    bool modeAuto;
                    /* data */
                };
struct controllerState 
                {
                    float speedA;
                    float speedB;
                    float positionA;
                    float positionB;
                    double currentVel;
                    double currentYawRate;
                    double currentPitch;
                    double targetPitch;
                    double controlOutput;
                    bool pitchPIDOn;
                    bool velPIDOn;
                    bool yawPIDOn;
                    
    
                };

#endif // UTILS_H

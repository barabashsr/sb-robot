#ifndef UTILS_H
#define UTILS_H
#include <Arduino.h>

// #include <memory>

// template<typename T, typename... Args>
// std::unique_ptr<T> make_unique(Args&&... args) {
//     return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
// }
struct transform
                {
                    double t_x;
                    double t_y;
                    double t_z;
                    double r_x;
                    double r_y;
                    double r_z;
                    double r_w;

                };

struct pidParams
                {
                    double Kp;
                    double Ki;
                    double Kd;
                    double max;
                    double min;
                    int64_t period;
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
                    double currentCOMVel;
                    double currentYawRate;
                    double currentPitch;
                    double targetPitch;
                    double targetVel;
                    double controlOutput;
                    bool pitchPIDOn;
                    bool velPIDOn;
                    bool yawPIDOn;
                    
    
                };

#endif // UTILS_H

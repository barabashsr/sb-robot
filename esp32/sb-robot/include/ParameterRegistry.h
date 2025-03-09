#ifndef PARAMETER_REGISTRY_H
#define PARAMETER_REGISTRY_H

#include <Arduino.h>
#include <Preferences.h>
#include <std_msgs/msg/int32.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <stdio.h>
#include <string.h>
#include <rcl/timer.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc_parameter/rclc_parameter.h>

// Maximum number of parameters the registry can hold
#define MAX_PARAMS 50
// Maximum length for parameter names
#define MAX_NAME_LENGTH 32
// Maximum length for parameter descriptions
#define MAX_DESC_LENGTH 64

// Enum for parameter types (matches rclc_parameter_type_t)
enum ParameterType {
    PARAM_BOOL = RCLC_PARAMETER_BOOL,
    PARAM_INT = RCLC_PARAMETER_INT,
    PARAM_DOUBLE = RCLC_PARAMETER_DOUBLE,
    //PARAM_STRING = RCLC_PARAMETER_STRING,
    //PARAM_BYTE_ARRAY = RCLC_PARAMETER_BYTE_ARRAY
};

// Structure to hold parameter metadata and values
struct ParameterInfo {
    char name[MAX_NAME_LENGTH];
    char description[MAX_DESC_LENGTH];
    char units[MAX_NAME_LENGTH];
    ParameterType type;
    
    // Value storage
    union {
        bool bool_value;
        int64_t integer_value;
        double double_value;
    } value;
    
    // Constraints for numeric types
    double min_value;
    double max_value;
    double step;
    
    // Flag to track if parameter has been modified
    bool modified;
};

class ParameterRegistry {
private:
    // Fixed-size array for parameters (to avoid dynamic allocation)
    ParameterInfo params[MAX_PARAMS];
    size_t param_count;
    
    // Preferences instance for EEPROM-like storage
    Preferences preferences;
    
    // Namespace for preferences storage
    char namespace_name[MAX_NAME_LENGTH];
    
    // Flag to track if preferences is open
    bool preferences_open;
    
    // Internal methods
    bool openPreferences();
    void closePreferences();
    bool saveParameter(const ParameterInfo* param);
    bool loadParameter(ParameterInfo* param);

    // Structures to track bindings
    struct BoolBinding { const char* name; bool* target; };
    struct IntBinding { const char* name; int64_t* target; };
    struct DoubleBinding { const char* name; double* target; };
    
    // Arrays to store bindings
    BoolBinding bool_bindings[MAX_PARAMS];
    IntBinding int_bindings[MAX_PARAMS];
    DoubleBinding double_bindings[MAX_PARAMS];
    
    // Counters for bindings
    size_t bool_binding_count = 0;
    size_t int_binding_count = 0;
    size_t double_binding_count = 0;
    
public:
    ParameterRegistry(const char* ns = "robot_params");
    ~ParameterRegistry();

    // Binding methods for different parameter types
    bool bindBoolParameter(const char* name, bool* target);
    bool bindIntParameter(const char* name, int64_t* target);
    bool bindDoubleParameter(const char* name, double* target);

    // Method to update all bound parameters
    void updateAllBindings();
    
    // Parameter management methods
    bool addBoolParameter(const char* name, bool default_value, const char* description = "", const char* units = "");
    bool addIntParameter(const char* name, int64_t default_value, const char* description = "", const char* units = "", int64_t min_value = 0, int64_t max_value = 0, int step = 0);
    bool addDoubleParameter(const char* name, double default_value, const char* description = "", const char* units = "", double min_value = 0.0, double max_value = 0.0, double step = 0.0);
    
    // Parameter access methods
    ParameterInfo* getParameter(const char* name);
    bool getBool(const char* name, bool* value);
    bool getInt(const char* name, int64_t* value);
    bool getDouble(const char* name, double* value);
    
    // Parameter modification methods
    bool setBool(const char* name, bool value);
    bool setInt(const char* name, int64_t value);
    bool setDouble(const char* name, double value);
    
    // Bulk operations
    bool saveAllParameters();
    bool loadAllParameters();
    void printAllParameters();
    
    // ROS parameter server integration
    void loadToParamServer(rclc_parameter_server_t* server);
    bool updateFromParamServer(rclc_parameter_server_t* server);
    
    // Get parameter count
    size_t getParameterCount() const { return param_count; }
    
    // Get parameter by index
    ParameterInfo* getParameterByIndex(size_t index);
};

#endif // PARAMETER_REGISTRY_H

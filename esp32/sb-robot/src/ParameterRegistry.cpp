#include "ParameterRegistry.h"

ParameterRegistry::ParameterRegistry(const char* ns) : param_count(0), preferences_open(false) {
    // Copy namespace name
    strncpy(namespace_name, ns, MAX_NAME_LENGTH - 1);
    namespace_name[MAX_NAME_LENGTH - 1] = '\0';
    
    // Initialize all parameters as empty
    for (size_t i = 0; i < MAX_PARAMS; i++) {
        params[i].name[0] = '\0';
        params[i].description[0] = '\0';
        params[i].units[0] = '\0';
        params[i].modified = false;
    }
}

ParameterRegistry::~ParameterRegistry() {
    // Close preferences if open
    if (preferences_open) {
        preferences.end();
    }
}

bool ParameterRegistry::openPreferences() {
    if (!preferences_open) {
        preferences_open = preferences.begin(namespace_name, false);
    }
    return preferences_open;
}

void ParameterRegistry::closePreferences() {
    if (preferences_open) {
        preferences.end();
        preferences_open = false;
    }
}

bool ParameterRegistry::addBoolParameter(const char* name, bool default_value, const char* description, const char* units) {
    if (param_count >= MAX_PARAMS) {
        Serial.println("ERROR: Parameter registry full");
        return false;
    }
    
    // Check if parameter already exists
    for (size_t i = 0; i < param_count; i++) {
        if (strcmp(params[i].name, name) == 0) {
            Serial.print("ERROR: Parameter already exists: ");
            Serial.println(name);
            return false;
        }
    }
    
    // Add new parameter
    strncpy(params[param_count].name, name, MAX_NAME_LENGTH - 1);
    params[param_count].name[MAX_NAME_LENGTH - 1] = '\0';
    
    strncpy(params[param_count].description, description, MAX_DESC_LENGTH - 1);
    params[param_count].description[MAX_DESC_LENGTH - 1] = '\0';
    
    strncpy(params[param_count].units, units, MAX_NAME_LENGTH - 1);
    params[param_count].units[MAX_NAME_LENGTH - 1] = '\0';
    
    params[param_count].type = PARAM_BOOL;
    
    // Try to load from preferences first
    if (openPreferences()) {
        if (preferences.isKey(name)) {
            params[param_count].value.bool_value = preferences.getBool(name, default_value);
        } else {
            params[param_count].value.bool_value = default_value;
            // Save default value to preferences
            preferences.putBool(name, default_value);
        }
        closePreferences();
    } else {
        params[param_count].value.bool_value = default_value;
    }
    
    params[param_count].modified = false;
    param_count++;
    
    return true;
}

bool ParameterRegistry::addIntParameter(const char* name, int64_t default_value, const char* description, const char* units, int64_t min_value, int64_t max_value, int step) {
    if (param_count >= MAX_PARAMS) {
        Serial.println("ERROR: Parameter registry full");
        return false;
    }
    
    // Check if parameter already exists
    for (size_t i = 0; i < param_count; i++) {
        if (strcmp(params[i].name, name) == 0) {
            Serial.print("ERROR: Parameter already exists: ");
            Serial.println(name);
            return false;
        }
    }
    
    // Add new parameter
    strncpy(params[param_count].name, name, MAX_NAME_LENGTH - 1);
    params[param_count].name[MAX_NAME_LENGTH - 1] = '\0';
    
    strncpy(params[param_count].description, description, MAX_DESC_LENGTH - 1);
    params[param_count].description[MAX_DESC_LENGTH - 1] = '\0';
    
    strncpy(params[param_count].units, units, MAX_NAME_LENGTH - 1);
    params[param_count].units[MAX_NAME_LENGTH - 1] = '\0';
    
    params[param_count].type = PARAM_INT;
    params[param_count].min_value = min_value;
    params[param_count].max_value = max_value;
    params[param_count].step = step;
    
    // Try to load from preferences first
    if (openPreferences()) {
        if (preferences.isKey(name)) {
            params[param_count].value.integer_value = preferences.getInt(name, default_value);
        } else {
            params[param_count].value.integer_value = default_value;
            // Save default value to preferences
            preferences.putInt(name, default_value);
        }
        closePreferences();
    } else {
        params[param_count].value.integer_value = default_value;
    }
    
    params[param_count].modified = false;
    param_count++;
    
    return true;
}

bool ParameterRegistry::addDoubleParameter(const char* name, double default_value, const char* description, const char* units, double min_value, double max_value, double step) {
    if (param_count >= MAX_PARAMS) {
        Serial.println("ERROR: Parameter registry full");
        return false;
    }
    
    // Check if parameter already exists
    for (size_t i = 0; i < param_count; i++) {
        if (strcmp(params[i].name, name) == 0) {
            Serial.print("ERROR: Parameter already exists: ");
            Serial.println(name);
            return false;
        }
    }
    
    // Add new parameter
    strncpy(params[param_count].name, name, MAX_NAME_LENGTH - 1);
    params[param_count].name[MAX_NAME_LENGTH - 1] = '\0';
    
    strncpy(params[param_count].description, description, MAX_DESC_LENGTH - 1);
    params[param_count].description[MAX_DESC_LENGTH - 1] = '\0';
    
    strncpy(params[param_count].units, units, MAX_NAME_LENGTH - 1);
    params[param_count].units[MAX_NAME_LENGTH - 1] = '\0';
    
    params[param_count].type = PARAM_DOUBLE;
    params[param_count].min_value = min_value;
    params[param_count].max_value = max_value;
    params[param_count].step = step;
    
    // Try to load from preferences first
    if (openPreferences()) {
        if (preferences.isKey(name)) {
            params[param_count].value.double_value = preferences.getDouble(name, default_value);
        } else {
            params[param_count].value.double_value = default_value;
            // Save default value to preferences
            preferences.putDouble(name, default_value);
        }
        closePreferences();
    } else {
        params[param_count].value.double_value = default_value;
    }
    
    params[param_count].modified = false;
    param_count++;
    
    return true;
}

ParameterInfo* ParameterRegistry::getParameter(const char* name) {
    for (size_t i = 0; i < param_count; i++) {
        if (strcmp(params[i].name, name) == 0) {
            return &params[i];
        }
    }
    return nullptr;
}

ParameterInfo* ParameterRegistry::getParameterByIndex(size_t index) {
    if (index < param_count) {
        return &params[index];
    }
    return nullptr;
}

bool ParameterRegistry::getBool(const char* name, bool* value) {
    ParameterInfo* param = getParameter(name);
    if (param && param->type == PARAM_BOOL) {
        *value = param->value.bool_value;
        return true;
    }
    return false;
}

bool ParameterRegistry::getInt(const char* name, int64_t* value) {
    ParameterInfo* param = getParameter(name);
    if (param && param->type == PARAM_INT) {
        *value = param->value.integer_value;
        return true;
    }
    return false;
}

bool ParameterRegistry::getDouble(const char* name, double* value) {
    ParameterInfo* param = getParameter(name);
    if (param && param->type == PARAM_DOUBLE) {
        *value = param->value.double_value;
        return true;
    }
    return false;
}

bool ParameterRegistry::setBool(const char* name, bool value) {
    ParameterInfo* param = getParameter(name);
    if (param && param->type == PARAM_BOOL) {
        param->value.bool_value = value;
        param->modified = true;
        return true;
    }
    return false;
}

bool ParameterRegistry::setInt(const char* name, int64_t value) {
    ParameterInfo* param = getParameter(name);
    if (param && param->type == PARAM_INT) {
        // Apply constraints if they exist
        if (param->max_value > param->min_value) {
            if (value > param->max_value) value = param->max_value;
            if (value < param->min_value) value = param->min_value;
        }
        param->value.integer_value = value;
        param->modified = true;
        return true;
    }
    return false;
}

bool ParameterRegistry::setDouble(const char* name, double value) {
    ParameterInfo* param = getParameter(name);
    if (param && param->type == PARAM_DOUBLE) {
        // Apply constraints if they exist
        if (param->max_value > param->min_value) {
            if (value > param->max_value) value = param->max_value;
            if (value < param->min_value) value = param->min_value;
        }
        param->value.double_value = value;
        param->modified = true;
        return true;
    }
    return false;
}

bool ParameterRegistry::saveParameter(const ParameterInfo* param) {
    if (!openPreferences()) {
        Serial.println("ERROR: Failed to open preferences");
        return false;
    }
    
    bool success = false;
    
    switch (param->type) {
        case PARAM_BOOL:
            success = preferences.putBool(param->name, param->value.bool_value);
            break;
        case PARAM_INT:
            success = preferences.putInt(param->name, param->value.integer_value);
            break;
        case PARAM_DOUBLE:
            success = preferences.putDouble(param->name, param->value.double_value);
            break;
        default:
            Serial.println("ERROR: Unsupported parameter type");
            break;
    }
    
    closePreferences();
    return success;
}

bool ParameterRegistry::loadParameter(ParameterInfo* param) {
    if (!openPreferences()) {
        Serial.println("ERROR: Failed to open preferences");
        return false;
    }
    
    if (!preferences.isKey(param->name)) {
        closePreferences();
        return false;
    }
    
    bool success = false;
    
    switch (param->type) {
        case PARAM_BOOL:
            param->value.bool_value = preferences.getBool(param->name, param->value.bool_value);
            success = true;
            break;
        case PARAM_INT:
            param->value.integer_value = preferences.getInt(param->name, param->value.integer_value);
            success = true;
            break;
        case PARAM_DOUBLE:
            param->value.double_value = preferences.getDouble(param->name, param->value.double_value);
            success = true;
            break;
        default:
            Serial.println("ERROR: Unsupported parameter type");
            break;
    }
    
    closePreferences();
    return success;
}

bool ParameterRegistry::saveAllParameters() {
    if (!openPreferences()) {
        Serial.println("ERROR: Failed to open preferences");
        return false;
    }
    
    bool all_success = true;
    
    for (size_t i = 0; i < param_count; i++) {
        if (params[i].modified) {
            bool success = false;
            
            switch (params[i].type) {
                case PARAM_BOOL:
                    success = preferences.putBool(params[i].name, params[i].value.bool_value);
                    break;
                case PARAM_INT:
                    success = preferences.putInt(params[i].name, params[i].value.integer_value);
                    break;
                case PARAM_DOUBLE:
                    success = preferences.putDouble(params[i].name, params[i].value.double_value);
                    break;
                default:
                    Serial.print("ERROR: Unsupported parameter type for ");
                    Serial.println(params[i].name);
                    break;
            }
            
            if (success) {
                params[i].modified = false;
            } else {
                all_success = false;
                Serial.print("ERROR: Failed to save parameter: ");
                Serial.println(params[i].name);
            }
        }
    }
    
    closePreferences();
    return all_success;
}

bool ParameterRegistry::loadAllParameters() {
    if (!openPreferences()) {
        Serial.println("ERROR: Failed to open preferences");
        return false;
    }
    
    bool all_success = true;
    
    for (size_t i = 0; i < param_count; i++) {
        if (preferences.isKey(params[i].name)) {
            bool success = false;
            
            switch (params[i].type) {
                case PARAM_BOOL:
                    params[i].value.bool_value = preferences.getBool(params[i].name, params[i].value.bool_value);
                    success = true;
                    break;
                case PARAM_INT:
                    params[i].value.integer_value = preferences.getInt(params[i].name, params[i].value.integer_value);
                    success = true;
                    break;
                case PARAM_DOUBLE:
                    params[i].value.double_value = preferences.getDouble(params[i].name, params[i].value.double_value);
                    success = true;
                    break;
                default:
                    Serial.print("ERROR: Unsupported parameter type for ");
                    Serial.println(params[i].name);
                    break;
            }
            
            if (!success) {
                all_success = false;
                Serial.print("ERROR: Failed to load parameter: ");
                Serial.println(params[i].name);
            }
        }
    }
    
    closePreferences();
    return all_success;
}

void ParameterRegistry::printAllParameters() {
    Serial.println("=== Parameter Registry Contents ===");
    Serial.print("Total parameters: ");
    Serial.println(param_count);
    
    for (size_t i = 0; i < param_count; i++) {
        Serial.print(i);
        Serial.print(": ");
        Serial.print(params[i].name);
        Serial.print(" (");
        
        switch (params[i].type) {
            case PARAM_BOOL:
                Serial.print("bool) = ");
                Serial.println(params[i].value.bool_value ? "true" : "false");
                break;
            case PARAM_INT:
                Serial.print("int) = ");
                Serial.println(params[i].value.integer_value);
                break;
            case PARAM_DOUBLE:
                Serial.print("double) = ");
                Serial.println(params[i].value.double_value, 6);
                break;
            default:
                Serial.println("unknown type)");
                break;
        }
        
        if (!params[i].description[0] == '\0') {
            Serial.print("   Description: ");
            Serial.println(params[i].description);
        }
        
        if (!params[i].units[0] == '\0') {
            Serial.print("   Units: ");
            Serial.println(params[i].units);
        }
        
        if (params[i].type == PARAM_INT || params[i].type == PARAM_DOUBLE) {
            if (params[i].min_value != params[i].max_value) {
                Serial.print("   Range: ");
                Serial.print(params[i].min_value);
                Serial.print(" to ");
                Serial.print(params[i].max_value);
                Serial.print(" (step: ");
                Serial.print(params[i].step);
                Serial.println(")");
            }
        }
        
        Serial.println();
    }
}

void ParameterRegistry::loadToParamServer(rclc_parameter_server_t* server) {
    for (size_t i = 0; i < param_count; i++) {
        ParameterInfo* param = &params[i];
        
        // Convert our parameter type to rclc parameter type
        rclc_parameter_type_t rclc_type;
        switch (param->type) {
            case PARAM_BOOL:
                rclc_type = RCLC_PARAMETER_BOOL;
                break;
            case PARAM_INT:
                rclc_type = RCLC_PARAMETER_INT;
                break;
            case PARAM_DOUBLE:
                rclc_type = RCLC_PARAMETER_DOUBLE;
                break;
            default:
                Serial.print("ERROR: Unsupported parameter type for ROS: ");
                Serial.println(param->name);
                continue;
        }
        
        // Add parameter to server
        rclc_add_parameter(server, param->name, rclc_type);
        
        // Add description and constraints
        rclc_add_parameter_description(server, param->name, param->description, param->units);
        
        // Add constraints for numeric types
        if (param->type == PARAM_DOUBLE) {
            rclc_add_parameter_constraint_double(server, param->name, 
                                               param->min_value, 
                                               param->max_value, 
                                               param->step);
        } else if (param->type == PARAM_INT) {
            rclc_add_parameter_constraint_double(server, param->name, 
                                               param->min_value, 
                                               param->max_value, 
                                               param->step);
        }
        
        // Set parameter value based on type
        switch (param->type) {
            case PARAM_BOOL:
                rclc_parameter_set_bool(server, param->name, param->value.bool_value);
                break;
            case PARAM_INT:
                rclc_parameter_set_int(server, param->name, param->value.integer_value);
                break;
            case PARAM_DOUBLE:
                rclc_parameter_set_double(server, param->name, param->value.double_value);
                break;
            default:
                break;
        }
    }
}

bool ParameterRegistry::updateFromParamServer(rclc_parameter_server_t* server) {
    bool all_success = true;
    
    for (size_t i = 0; i < param_count; i++) {
        ParameterInfo* param = &params[i];
        bool success = false;
        
        switch (param->type) {
            case PARAM_BOOL: {
                bool value;
                if (rclc_parameter_get_bool(server, param->name, &value) == RCL_RET_OK) {
                    param->value.bool_value = value;
                    param->modified = true;
                    success = true;
                }
                break;
            }
            case PARAM_INT: {
                int64_t value;
                if (rclc_parameter_get_int(server, param->name, &value) == RCL_RET_OK) {
                    param->value.integer_value = value;
                    param->modified = true;
                    success = true;
                }
                break;
            }
            case PARAM_DOUBLE: {
                double value;
                if (rclc_parameter_get_double(server, param->name, &value) == RCL_RET_OK) {
                    param->value.double_value = value;
                    param->modified = true;
                    success = true;
                }
                break;
            }
            default:
                Serial.print("ERROR: Unsupported parameter type for ROS: ");
                Serial.println(param->name);
                break;
        }
        
        if (!success) {
            all_success = false;
            Serial.print("ERROR: Failed to update parameter from ROS: ");
            Serial.println(param->name);
        }
    }
    
    return all_success;
}


bool ParameterRegistry::bindBoolParameter(const char* name, bool* target) {
    ParameterInfo* param = getParameter(name);
    if (!param || param->type != PARAM_BOOL) {
        Serial.print("ERROR: Cannot bind to parameter: ");
        Serial.println(name);
        return false;
    }
    
    // Store binding
    if (bool_binding_count < MAX_PARAMS) {
        bool_bindings[bool_binding_count].name = name;
        bool_bindings[bool_binding_count].target = target;
        bool_binding_count++;
        
        // Initialize target with current value
        *target = param->value.bool_value;
        return true;
    }
    
    Serial.println("ERROR: Too many bool bindings");
    return false;
}

bool ParameterRegistry::bindIntParameter(const char* name, int64_t* target) {
    ParameterInfo* param = getParameter(name);
    if (!param || param->type != PARAM_INT) {
        Serial.print("ERROR: Cannot bind to parameter: ");
        Serial.println(name);
        return false;
    }
    
    // Store binding
    if (int_binding_count < MAX_PARAMS) {
        int_bindings[int_binding_count].name = name;
        int_bindings[int_binding_count].target = target;
        int_binding_count++;
        
        // Initialize target with current value
        *target = param->value.integer_value;
        return true;
    }
    
    Serial.println("ERROR: Too many int bindings");
    return false;
}

bool ParameterRegistry::bindDoubleParameter(const char* name, double* target) {
    ParameterInfo* param = getParameter(name);
    if (!param || param->type != PARAM_DOUBLE) {
        Serial.print("ERROR: Cannot bind to parameter: ");
        Serial.println(name);
        return false;
    }
    
    // Store binding
    if (double_binding_count < MAX_PARAMS) {
        double_bindings[double_binding_count].name = name;
        double_bindings[double_binding_count].target = target;
        double_binding_count++;
        
        // Initialize target with current value
        *target = param->value.double_value;
        return true;
    }
    
    Serial.println("ERROR: Too many double bindings");
    return false;
}

void ParameterRegistry::updateAllBindings() {
    // Update all bool bindings
    for (size_t i = 0; i < bool_binding_count; i++) {
        ParameterInfo* param = getParameter(bool_bindings[i].name);
        if (param && param->type == PARAM_BOOL) {
            *(bool_bindings[i].target) = param->value.bool_value;
        }
    }
    
    // Update all int bindings
    for (size_t i = 0; i < int_binding_count; i++) {
        ParameterInfo* param = getParameter(int_bindings[i].name);
        if (param && param->type == PARAM_INT) {
            *(int_bindings[i].target) = param->value.integer_value;
        }
    }
    
    // Update all double bindings
    for (size_t i = 0; i < double_binding_count; i++) {
        ParameterInfo* param = getParameter(double_bindings[i].name);
        if (param && param->type == PARAM_DOUBLE) {
            *(double_bindings[i].target) = param->value.double_value;
        }
    }
}



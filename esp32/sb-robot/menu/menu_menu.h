/*
    The code in this file uses open source libraries provided by thecoderscorner

    DO NOT EDIT THIS FILE, IT WILL BE GENERATED EVERY TIME YOU USE THE UI DESIGNER
    INSTEAD EITHER PUT CODE IN YOUR SKETCH OR CREATE ANOTHER SOURCE FILE.

    All the variables you may need access to are marked extern in this file for easy
    use elsewhere.
 */

#ifndef MENU_GENERATED_CODE_H
#define MENU_GENERATED_CODE_H

#include <Arduino.h>
#include <tcMenu.h>
#include "tcMenuU8g2.h"
#include <IoAbstraction.h>
#include <EepromItemStorage.h>
#include <ArduinoEEPROMAbstraction.h>

// variables we declare that you may need to access
extern const PROGMEM ConnectorLocalInfo applicationInfo;
extern U8G2_SH1106_128X64_NONAME_F_HW_I2C gfx;
extern U8g2Drawable gfxDrawable;
extern GraphicsDeviceRenderer renderer;

// Any externals needed by IO expanders, EEPROMs etc


// Global Menu Item exports
extern AnalogMenuItem menuBNOCalib;
extern BackMenuItem menuBackState;
extern SubMenuItem menuState;
extern FloatMenuItem menuYawRate;
extern FloatMenuItem menuVel;
extern AnalogMenuItem menuPeriodY;
extern AnalogMenuItem menuKdYaw;
extern AnalogMenuItem menuKiYaw;
extern AnalogMenuItem menuKpYaw;
extern BooleanMenuItem menuYawPIDToggle;
extern BackMenuItem menuBackYawPID;
extern SubMenuItem menuYawPID;
extern AnalogMenuItem menuPeriodV;
extern AnalogMenuItem menuKoVel;
extern AnalogMenuItem menuKdVel;
extern AnalogMenuItem menuKiVel;
extern AnalogMenuItem menuKpVel;
extern BooleanMenuItem menuVelPIDToggle;
extern BackMenuItem menuBackVelPID;
extern SubMenuItem menuVelPID;
extern AnalogMenuItem menuPeriodP;
extern AnalogMenuItem menuKdPitch;
extern AnalogMenuItem menuKiPitch;
extern AnalogMenuItem menuKpPitch;
extern BooleanMenuItem menuPitchPIDToggle;
extern BackMenuItem menuBackPitchPID;
extern SubMenuItem menuPitchPID;
extern AnalogMenuItem menuPeriodPalst;
extern AnalogMenuItem menuKoPalst;
extern AnalogMenuItem menuKdPalst;
extern AnalogMenuItem menuKiPalst;
extern AnalogMenuItem menuKpPalst;
extern BooleanMenuItem menuPalstPIDToggle;
extern BackMenuItem menuBackPalstPID;
extern SubMenuItem menuPalstPID;
extern AnalogMenuItem menuThresholdB;
extern AnalogMenuItem menuThresholdA;
extern BackMenuItem menuBackMotorConfig;
extern SubMenuItem menuMotorConfig;
extern ActionMenuItem menuSaveValues;
extern AnalogMenuItem menuPitchOfset;
extern FloatMenuItem menuPitch;
extern AnalogMenuItem menuSetYaw;
extern AnalogMenuItem menuSetVel;
extern FloatMenuItem menuPalstance;

// Provide a wrapper to get hold of the root menu item and export setupMenu
inline MenuItem& rootMenuItem() { return menuPalstance; }
void setupMenu();

// Callback functions must always include CALLBACK_FUNCTION after the return type
#define CALLBACK_FUNCTION

void CALLBACK_FUNCTION SavePID(int id);
void CALLBACK_FUNCTION SetKdPalst(int id);
void CALLBACK_FUNCTION SetKdPitch(int id);
void CALLBACK_FUNCTION SetKdVel(int id);
void CALLBACK_FUNCTION SetKdYaw(int id);
void CALLBACK_FUNCTION SetKiPalst(int id);
void CALLBACK_FUNCTION SetKiPitch(int id);
void CALLBACK_FUNCTION SetKiVel(int id);
void CALLBACK_FUNCTION SetKiYaw(int id);
void CALLBACK_FUNCTION SetKoPalst(int id);
void CALLBACK_FUNCTION SetKoVel(int id);
void CALLBACK_FUNCTION SetKpPalst(int id);
void CALLBACK_FUNCTION SetKpPitch(int id);
void CALLBACK_FUNCTION SetKpVel(int id);
void CALLBACK_FUNCTION SetKpYaw(int id);
void CALLBACK_FUNCTION SetPitchOfset(int id);
void CALLBACK_FUNCTION getBNOCalib(int id);
void CALLBACK_FUNCTION setPalstPIDPeriod(int id);
void CALLBACK_FUNCTION setPitchPIDPeriod(int id);
void CALLBACK_FUNCTION setTargetVel(int id);
void CALLBACK_FUNCTION setTargetYawRa(int id);
void CALLBACK_FUNCTION setThresholdA(int id);
void CALLBACK_FUNCTION setThresholdB(int id);
void CALLBACK_FUNCTION setVelPIDPeriod(int id);
void CALLBACK_FUNCTION setYawPIDPeriod(int id);
void CALLBACK_FUNCTION togglePalstPid(int id);
void CALLBACK_FUNCTION togglePitchPid(int id);
void CALLBACK_FUNCTION toggleVelPid(int id);
void CALLBACK_FUNCTION toggleYawPID(int id);

#endif // MENU_GENERATED_CODE_H

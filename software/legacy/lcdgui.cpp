#include "lcdgui.h"


// Initialize the LCD with the I2C address 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);

void initializeLcdGui() {
    lcd.begin(16, 2); // Initialize the LCD with 16 columns and 2 rows
    lcd.backlight();
    lcd.clear();
    initializeEncoder();
}

void updateLcdGui(const TempData& tempData, const ControlData& controlData, State* state) {
    char line1[17];  // Buffer for first line
    char line2[17];  // Buffer for second line

    // Get the state symbol
    const char* stateSymbol = getStateSymbol(state);

    if (state == standbyState) {
        cursorState.maxPositions = 2; // Two positions: START and SETTINGS
        snprintf(line1, sizeof(line1), "%cSTART %cSETTINGS", 
                 cursorState.position == 0 ? '~' : '≥', 
                 cursorState.position == 1 ? '~' : '≥');
        snprintf(line2, sizeof(line2), "T%3d,%3d     %s", 
                 static_cast<int>(tempData.temperatures[0]), 
                 static_cast<int>(tempData.temperatures[1]), 
                 stateSymbol);
    } 
    else if (state == settingsState) {
        cursorState.maxPositions = 6; // Six positions: PID (x3), D, ST, and ^
        snprintf(line1, sizeof(line1), "PID:%c%3d%c%3d%c%3d", 
                 cursorState.position == 0 ? '~' : '≥', static_cast<int>(pData.Kp[0]), 
                 cursorState.position == 1 ? '~' : '≥', static_cast<int>(pData.Ki[0]), 
                 cursorState.position == 2 ? '~' : '≥', static_cast<int>(pData.Kd[0]));
        snprintf(line2, sizeof(line2), "D:%c010 ST:%c%3d%c^", 
                 cursorState.position == 3 ? '~' : '≥', 
                 cursorState.position == 4 ? '~' : '≥', 
                 static_cast<int>(pData.setpoint[0]), 
                 cursorState.position == 5 ? '~' : '≥');
    } 
    else {
        cursorState.maxPositions = 2; // Three positions: S, D, and state symbol
        snprintf(line1, sizeof(line1), "T%3d,%3d;%3d,%3d", 
                 static_cast<int>(tempData.temperatures[0]), 
                 static_cast<int>(tempData.temperatures[1]), 
                 static_cast<int>(controlData.outputs[0]), 
                 static_cast<int>(controlData.outputs[1]));
        snprintf(line2, sizeof(line2), "S%c%3d;D%c032 %s", 
                 cursorState.position == 0 ? '~' : '≥', static_cast<int>(pData.setpoint[0]), 
                 cursorState.position == 1 ? '~' : '≥', 
                 stateSymbol);
    }

    // Display the formatted lines
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}

const char* getStateSymbol(State* state) {
    if (state == standbyState) {
        return "SBY";
    } else if (state == preheatingState) {
        return "PRE";
    } else if (state == heatingState) {
        return "HEA";
    } else if (state == coolingState) {
        return "COO";
    } else if (state == errorState) {
        return "ERR";
    } else if (state == settingsState) {
        return "SET";
    } else {
        return "";
    }
}

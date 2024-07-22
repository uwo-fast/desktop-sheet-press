#include "lcdgui.h"
#include "_states.h"
#include "encodergui.h"

// Initialize the LCD with the I2C address 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);

bool cursorOnStart = true;

void initializeLcdGui() {
    lcd.begin(16, 2); // Initialize the LCD with 16 columns and 2 rows
    lcd.backlight();
    lcd.clear();
}

void updateLcdGui(const TempData& tempData, const ControlData& controlData, const char* stateSymbol, MachineState currentState) {
    char line1[17];  // Buffer for first line
    char line2[17];  // Buffer for second line

    switch (currentState) {
        case STANDBY:
            cursorState.maxPositions = 2; // Two positions: START and SETTINGS
            snprintf(line1, sizeof(line1), "%cSTART %cSETTINGS", 
                     cursorOnStart ? '>' : ' ', 
                     cursorOnStart ? ' ' : '>');
            snprintf(line2, sizeof(line2), "T%3d,%3d     %s", 
                     static_cast<int>(tempData.temperatures[0]*5), 
                     static_cast<int>(tempData.temperatures[1]*5), 
                     stateSymbol);
            break;
        
        case SETTINGS:
            cursorState.maxPositions = 6; // Six positions: PID (3), D, ST, and ^
            snprintf(line1, sizeof(line1), "PID:%c%3d%c%3d%c%3d", 
                     cursorState.position == 0 ? '>' : ' ', static_cast<int>(pidControllers[0]->GetKp()), 
                     cursorState.position == 1 ? '>' : ' ', static_cast<int>(pidControllers[0]->GetKi()), 
                     cursorState.position == 2 ? '>' : ' ', static_cast<int>(pidControllers[0]->GetKd()));
            snprintf(line2, sizeof(line2), "D:%c010 ST:%c%3d%c^", 
                     cursorState.position == 3 ? '>' : ' ', 
                     cursorState.position == 4 ? '>' : ' ', 
                     static_cast<int>(setpoint[0]), 
                     cursorState.position == 5 ? '>' : ' ');
            break;
        
        default:
            cursorState.maxPositions = 2; // Three positions: S, D, and state symbol
            snprintf(line1, sizeof(line1), "T%3d,%3d;%3d,%3d", 
                     static_cast<int>(tempData.temperatures[0]*6), 
                     static_cast<int>(tempData.temperatures[1]*6), 
                     static_cast<int>(controlData.outputs[0]), 
                     static_cast<int>(controlData.outputs[1]));
            snprintf(line2, sizeof(line2), "S%c%3d;D%c032 %s", 
                     cursorState.position == 0 ? '>' : ' ', static_cast<int>(setpoint[0]), 
                     cursorState.position == 1 ? '>' : ' ', 
                     stateSymbol);
            break;
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

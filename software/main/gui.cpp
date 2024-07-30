#include "gui.h"

#include <LiquidCrystal_I2C.h>
#include <LiquidMenu.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

void initializeLcdGui()
{
    lcd.init();
    lcd.backlight();
    lcd.clear();
    // menu.init();
}

void updateLcdGui()
{
    // menu.update();
}


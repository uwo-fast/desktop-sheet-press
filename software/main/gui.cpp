#include "gui.h"

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ----- MENU -----
// create a menu from the screens
LiquidMenu menu(lcd);
// ----------------

void initializeLcdGui(LiquidScreen main, LiquidScreen standby, LiquidScreen prep, LiquidScreen active, LiquidScreen term, LiquidScreen error)
{
    lcd.init();
    lcd.backlight();
    lcd.clear();
    menu.init();
    menu.add_screen(main);
    menu.add_screen(standby);
    menu.add_screen(prep);
    menu.add_screen(active);
    menu.add_screen(term);
    menu.add_screen(error);
}

void updateLcdGui()
{
    menu.update();
}

ClickEncoder *encoder;
UserEvent uEvent = EV_NONE;
int16_t encLastPos = 0;
int16_t encNewPos = 0;

void timerIsr()
{
    encoder->service();
}

/*
* Initialize the encoder with the given pins and steps per click.
* @params pinA, pinB, pinC: pins for the encoder
* @params steps: number of steps per click

*/
void initializeEncoder(int pinA, int pinB, int pinC, int steps)
{
    // param 5 set to false to keep pins at active high
    encoder = new ClickEncoder(pinA, pinB, pinC, steps, false); // Enable pullup resistors
    Timer1.initialize();
    Timer1.attachInterrupt(timerIsr);
    Timer1.setPeriod(1000); // 1ms or 1000us
    encLastPos = encNewPos = 0;
}

void processEncoderEvents()
{
    encNewPos += encoder->getValue();

    if (encNewPos != encLastPos)
    {

        int direction = (encNewPos < encLastPos) ? 1 : -1;
#ifdef REVERSE_ENCODER
        direction = -direction;
#endif
        uEvent = (direction == 1) ? EV_ENCDN : EV_ENCUP;
        int16_t incr = encNewPos - encLastPos;

        encLastPos = encNewPos;
    }

    ClickEncoder::Button b = encoder->getButton();
    if (b != ClickEncoder::Open)
    {
        switch (b)
        {
        case ClickEncoder::Clicked:
            uEvent = EV_BTN_CLICKED;
            Serial.println("Button Clicked");
            break;
        case ClickEncoder::DoubleClicked:
            uEvent = EV_BTN_2CLICKED;
            break;
        case ClickEncoder::Held:
            uEvent = EV_BTN_HELD;
            break;
        case ClickEncoder::Released:
            uEvent = EV_BTN_RELEASED;
            break;
        default:
            uEvent = EV_NONE;
            break;
        }
    }
}
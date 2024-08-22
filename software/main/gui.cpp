#include "gui.h"

LiquidCrystal_I2C lcd(0x27, 16, 2);

ClickEncoder *encoder;
UserEvent uEvent = EV_NONE;
int16_t encLastPos = 0;
int16_t encNewPos = 0;

// ----- MENU -----
// create a menu from the screens
LiquidMenu menu(lcd);
// ----------------

void initializeLCD()
{
    lcd.init();
    lcd.backlight();
    lcd.clear();
}

void initializeLcdGui()
{
    menu.add_screen(screenMain);
    menu.add_screen(screenStandby);
    menu.add_screen(screenPrep);
    menu.add_screen(screenActive);
    menu.add_screen(screenTerm);
    menu.add_screen(screenError);
    menu.switch_focus(true);
}

void updateLcdGui()
{
    switch (uEvent)
    {
    case EV_BTN_CLICKED:
        if (menu.get_currentScreen() == &screenMain)
        {
            menu.change_screen(&screenStandby);
        }
        else
        {
        }
        break;
    case EV_BTN_2CLICKED:
        break;
    case EV_BTN_HELD:
        break;
    case EV_BTN_RELEASED:
        break;
    case EV_ENCUP:
        menu.switch_focus(false);
        break;
    case EV_ENCDN:
        menu.switch_focus(true);
        break;
    default:
        break;
    }
    uEvent = EV_NONE;
    menu.update();
}

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
    encoder = new ClickEncoder(pinA, pinB, pinC, steps, true); // Enable pullup resistors
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
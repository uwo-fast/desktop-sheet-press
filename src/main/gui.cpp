#include "gui.h"
#include "config.h"
#include <Arduino.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

ClickEncoder *encoder;
UserEvent uEvent = EV_NONE;
int16_t encLastPos = 0;
int16_t encNewPos = 0;
bool toggle = false;

LiquidMenu menu(lcd);

/*
 * Toggle wrapper
 * @params none
 * @returns none
 */
void toggleToggler()
{
    toggle = !toggle;
}

/*
 * Go to the main screen
 * @params none
 * @returns none
 */
void goToMainScreen()
{
    menu.change_screen(&screenMain);
    menu.set_focusedLine(0);
}

/*
 * Go to the options screen
 * @params none
 * @returns none
 */
void goToOptionsScreen()
{
    menu.change_screen(&screenOptions);
    menu.set_focusedLine(0);
}

/*
 * Initialize the LCD
 * @params none
 * @returns none
 */
void initializeLCD()
{
    lcd.init();
    lcd.backlight();
    lcd.clear();
}

/*
 * Initialize the LCD Menu GUI
 * @params none
 * @returns none
 */
void initializeLcdGui()
{
    menu.add_screen(screenMain);
    menu.add_screen(screenOptions);
    menu.add_screen(screenError);
}

void updateLcdGui()
{
    switch (uEvent)
    {
    case EV_BTN_CLICKED:
        if (menu.get_currentScreen() == &screenMain)
        {
            goToOptionsScreen();
        }
        else
        {
            menu.call_function(FUNC_USE);
        }
        break;
    case EV_BTN_2CLICKED:
        menu.call_function(FUNC_SKIP);
        break;
    case EV_BTN_HELD:
        menu.call_function(FUNC_BACK);
        break;
    case EV_BTN_RELEASED:
        break;
    case EV_ENCUP:
        if (toggle == false)
        {
            menu.switch_focus(false);
        }
        else
        {
            menu.call_function(FUNC_INCRT);
        }
        break;
    case EV_ENCDN:
        if (toggle == false)
        {
            menu.switch_focus(true);
        }
        else
        {
            menu.call_function(FUNC_INCRT);
        }
        break;
    default:
        break;
    }
    uEvent = EV_NONE;
    menu.update();
}

/*
 * Timer ISR wrapper to service the encoder
 * @params none
 * @returns none
 */
void timerIsr()
{
    encoder->service();
}

/*
 * Initialize the encoder with the given pins and steps per click.
 * @params pinA, pinB, pinC: pins for the encoder
 * @params steps: number of steps per click
 * @return none
 */
void initializeEncoder(int pinA, int pinB, int pinC, int steps)
{
    // param 5 set to false to keep pins at active high
    encoder = new ClickEncoder(pinA, pinB, pinC, steps, ENC_PULLUP); // Enable pullup resistors
    Timer1.initialize();
    Timer1.attachInterrupt(timerIsr);
    Timer1.setPeriod(ENC_INTERRUP_MS);
    encLastPos = encNewPos = 0;
}

/*
 * Process the encoder events and return the increment value
 * @params none
 * @returns int16_t: increment value
 */
int16_t processEncoderEvents()
{
    encNewPos += encoder->getValue();
    int16_t incr;

    if (encNewPos != encLastPos)
    {

        int direction = (encNewPos < encLastPos) ? 1 : -1;
#ifdef REVERSE_ENCODER
        direction = -direction;
#endif
        uEvent = (direction == 1) ? EV_ENCDN : EV_ENCUP;
        incr = encNewPos - encLastPos;
        encLastPos = encNewPos;
    }
    else
    {
        incr = 0;
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
    return incr;
}
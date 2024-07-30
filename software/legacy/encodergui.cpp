#include "encodergui.h"

ClickEncoder *encoder;
int16_t encLastPos, encNewPos; // Encoder position variables
UserEvent uEvent = EV_NONE;    // Define uEvent

extern String stateCommand; // Use extern declaration for stateCommand

CursorState cursorState(2); // Default to 2 positions for standby screen

void timerIsr()
{
    encoder->service();
}

void initializeEncoder()
{
    encoder = new ClickEncoder(PIN_ENC_DT, PIN_ENC_CLK, PIN_ENC_SW, 4, true); // Enable pullup resistors
    Timer1.initialize(1000);                                                  // 1ms or 1000us
    Timer1.attachInterrupt(timerIsr);
    encLastPos = encNewPos = 0;
}
// TODO: fix to use pData
void processEncoderEvents(State *state)
{
    static unsigned long lastEncTime = 0;

    encNewPos += encoder->getValue();

    if (encNewPos != encLastPos)
    {
        lastEncTime = millis();

        int direction = (encNewPos < encLastPos) ? 1 : -1;
#ifdef REVERSE_ENCODER
        direction = -direction;
#endif

        uEvent = (direction == 1) ? EV_ENCDN : EV_ENCUP;

        if (state == settingsState)
        {
            if (cursorState.isSelected)
            {
                float currentValue;
                if (cursorState.position == 0)
                {

                    currentValue = pidControllers[0]->GetKp();
                    pData.Kp[0] = currentValue + direction;
                }
                else if (cursorState.position == 1)
                {

                    currentValue = pidControllers[0]->GetKi();
                    pData.Ki[0] = currentValue + direction;
                }
                else if (cursorState.position == 2)
                {

                    currentValue = pidControllers[0]->GetKd();
                    pData.Kd[0] = currentValue + direction;
                }
                else if (cursorState.position == 4)
                {
                    pData.setpoint[0] = static_cast<int>(pData.setpoint[0]) + 5 * direction; // Change pData.setpoint in increments of 5
                    pData.setpoint[1] = pData.setpoint[0];                                   // Assuming both setpoints are the same
                }
                pidControllers[0]->SetTunings(pData.Ki[0], pData.Ki[0], pData.Kd[0]);
                pidControllers[1]->SetTunings(pData.Kd[0], pData.Ki[0], pData.Kd[0]);
            }
            else
            {
                cursorState.moveCursor(direction);
            }
        }
        else if (state == standbyState)
        {
            cursorState.moveCursor(direction);
        }
        else if (state == preheatingState || state == heatingState || state == coolingState)
        {
            if (cursorState.isSelected)
            {
                if (cursorState.position == 0)
                {
                    pData.setpoint[0] = static_cast<int>(pData.setpoint[0]) + 5 * direction; // Change pData.setpoint in increments of 5
                    pData.setpoint[1] = pData.setpoint[0];                                   // Assuming both setpoints are the same
                }
            }
            else
            {
                cursorState.moveCursor(direction);
            }
        }
        else
        {
        }

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

    if (uEvent == EV_BTN_CLICKED)
    {
        if (state == standbyState)
        {
            if (cursorState.position == 0)
            {
                stateCommand = "preheat";
            }
            else
            {
                stateCommand = "settings";
            }
        }
        else if (state == settingsState)
        {
            if (cursorState.isSelected)
            {
                if (cursorState.position == 5)
                { // If ^ is selected
                    stateCommand = "standby";
                }
                else
                {
                    cursorState.isSelected = false;
                }
            }
            else
            {
                cursorState.isSelected = true;
            }
        }
        else if (state == preheatingState || state == heatingState || state == coolingState)
        {
            if (cursorState.isSelected)
            {
                cursorState.isSelected = false;
            }
            else
            {
                cursorState.isSelected = true;
            }
        }
        else
        {
            // Handle any other states if needed
        }
    }
    else if (uEvent == EV_BTN_HELD)
    {
        stateCommand = "standby";
    }
    // Clear the event after handling
    uEvent = EV_NONE;
}
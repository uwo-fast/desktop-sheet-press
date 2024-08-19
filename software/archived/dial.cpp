#include "dial.h"
#include "config.h"

ClickEncoder *encoder;
int16_t encLastPos, encNewPos; 
UserEvent uEvent = EV_NONE;

void timerIsr()
{
    encoder->service();
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
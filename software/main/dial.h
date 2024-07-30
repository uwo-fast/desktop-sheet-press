#ifndef DIAL_H
#define DIAL_H

#include <ClickEncoder.h>
#include <TimerOne.h>

void timerIsr();
void processEncoderEvents();

// Define possible events
enum UserEvent
{
    EV_NONE,
    EV_BTN_CLICKED,
    EV_BTN_2CLICKED,
    EV_BTN_HELD,
    EV_BTN_RELEASED,
    EV_ENCUP,
    EV_ENCDN
};

extern ClickEncoder *encoder;
extern UserEvent uEvent;
extern int16_t encLastPos, encNewPos;

#endif // DIAL_H
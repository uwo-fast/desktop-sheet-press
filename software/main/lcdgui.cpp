// LCD State and Event variables
#ifdef _LCDGUI_
uint8_t uEvent;				   /**< Current pending user event */
uint8_t uToggle;			   /**< Current user toggle */
int16_t encLastPos, encNewPos; /**< Encoder position variables */
unsigned long lastActiveTime;
#endif /* _LCDGUI_ */

unsigned int lcdRefreshPeriod = 300;
unsigned long lcdLastRefresh = 0;

#ifdef _LCDGUI_
void updateLcdGui()
{
	if (millis() - lcdLastRefresh > lcdRefreshPeriod)
	{
		lcdEventHandler();
		lcdLastRefresh = millis();
		menu_system.update();
	}
}
#endif /* _LCDGUI_ */


#ifdef _LCDGUI_
ClickEncoder *encoder;				/**< Encoder object */
LiquidCrystal_I2C lcd(0x27, 16, 2); /**< LCD display object */

// Declare strings to store in flash memory
const char fastResearchText[] PROGMEM = "FAST Research";
const char oschssPressText[] PROGMEM = "OSCHSS Press";
const char versionText[] PROGMEM = "Version:";
const char versionNumberText[] PROGMEM = "1.1.0";
const char licenseText[] PROGMEM = "License:";
const char gplv3FreeText[] PROGMEM = "GPLv3 \"FREE\"";

// SETUP DISPLAYS
LiquidLine setup_line1(0, 0, fastResearchText);
LiquidLine setup_line2(0, 1, oschssPressText);
LiquidLine setup_line3(0, 0, versionText);
LiquidLine setup_line4(0, 1, versionNumberText);
LiquidLine setup_line5(0, 0, licenseText);
LiquidLine setup_line6(0, 1, gplv3FreeText);
LiquidScreen setup_screenA(setup_line1, setup_line2);
LiquidScreen setup_screenB(setup_line3, setup_line4);
LiquidScreen setup_screenC(setup_line5, setup_line6);
LiquidMenu setup_menu(lcd, setup_screenA, setup_screenB, setup_screenC);

// MAIN DISPLAY MACHINE STATUS
LiquidLine machine_status_line1(0, 0, "1:", temp1, "C ", getStateInfo);
LiquidLine machine_status_line2(0, 1, "2:", temp2, "C ", getStatusInfo);
LiquidScreen machine_status_screenA(machine_status_line1, machine_status_line2);
LiquidMenu machine_status_menu(lcd, machine_status_screenA);

const char statusScreenText[] PROGMEM = "/Status Screen";
const char durationText[] PROGMEM = "Duration: ";
const char activateText[] PROGMEM = "Activate";
const char stopText[] PROGMEM = "Stop";

// MAIN DISPLAY MAIN MENU
LiquidLine main_line0(0, 0, statusScreenText);
LiquidLine main_line1(0, 1, getSetTempInfo);
LiquidLine main_line2(0, 1, durationText, getDurationInfo);
LiquidLine main_line3(0, 1, activateText);
LiquidLine main_line4(0, 1, stopText);
LiquidScreen main_screen;

LiquidMenu main_menu(lcd, main_screen);

// Menu system
LiquidSystem menu_system(setup_menu, machine_status_menu, main_menu);

void setupLcdLines()
{
	setup_line1.set_asProgmem(1);
	setup_line2.set_asProgmem(1);
	setup_line3.set_asProgmem(1);
	setup_line4.set_asProgmem(1);
	setup_line5.set_asProgmem(1);
	setup_line6.set_asProgmem(1);

	machine_status_line1.attach_function(FUNC_ENTER_MENU, goto_main_menu);
	machine_status_line1.set_focusPosition(Position::CUSTOM, 16, 0);

	machine_status_line2.attach_function(FUNC_ENTER_MENU, goto_main_menu);
	machine_status_line2.set_focusPosition(Position::CUSTOM, 16, 0);

	// Add lines and attach functions to the main menu
	main_line0.set_asProgmem(1);
	main_line1.set_asProgmem(1);
	main_line2.set_asProgmem(1);
	main_line3.set_asProgmem(1);
	main_line4.set_asProgmem(1);
	main_screen.add_line(main_line0);
	main_screen.add_line(main_line1);
	main_screen.add_line(main_line2);
	main_screen.add_line(main_line3);
	main_screen.add_line(main_line4);
	main_screen.set_displayLineCount(2);
	main_line0.attach_function(FUNC_ENTER_MENU, goto_machine_status_menu);
	main_line1.attach_function(FUNC_ENTER_MENU, toggle_function);
	main_line1.attach_function(FUNC_DECRT_PDATA, decrt_pdata_setTemp);
	main_line1.attach_function(FUNC_INCRT_PDATA, incrt_pdata_setTemp);
	main_line2.attach_function(FUNC_ENTER_MENU, toggle_function);
	main_line2.attach_function(FUNC_DECRT_PDATA, decrt_pdata_heatingDuration);
	main_line2.attach_function(FUNC_INCRT_PDATA, incrt_pdata_heatingDuration);
	main_line3.attach_function(FUNC_ENTER_MENU, activate_proc);
	main_line4.attach_function(FUNC_ENTER_MENU, deactivate_proc);
}

void timerIsr()
{
	encoder->service();
}

#endif /* _LCDGUI_ */

/* --------------------------------------------------------------------------------------*/
/*--------------------------------------- LCD_GUI ---------------------------------------*/
/* --------------------------------------------------------------------------------------*/
#ifdef _LCDGUI_
/* -------------------------------LCD Event Handler----------------------------------*/
void lcdEventHandler()
{

	switch (uEvent)
	{
	case (EV_NONE):
		break;
	case (EV_BTN_CLICKED):
		menu_system.call_function(FUNC_ENTER_MENU);
		uEvent = EV_NONE;
		break;
	case (EV_ENCUP):
		if (uToggle == TOGGLE_OFF)
		{
			menu_system.switch_focus(false);
			uEvent = EV_NONE;
		}
		else
		{
			menu_system.call_function(FUNC_DECRT_PDATA);
			uEvent = EV_NONE;
		}
		break;
	case (EV_ENCDN):
		if (uToggle == TOGGLE_OFF)
		{
			menu_system.switch_focus(true);
			uEvent = EV_NONE;
		}
		else
		{
			menu_system.call_function(FUNC_INCRT_PDATA);
			uEvent = EV_NONE;
		}
		break;
	case (EV_BOOTDN):
		resetEeprom(EE_FULL_RESET);
		break;
	case (EV_STBY_TIMEOUT):

		break;

	// UNUSED AT THE MOMENT//
	case (EV_BTN_2CLICKED):
		break;
	case (EV_BTN_HELD):
		break;
	case (EV_BTN_RELEASED):
		break;
		// UNUSED AT THE MOMENT//

	default:
		break;
	}
}
/* -------------------------------------FUNCTIONS----------------------------------------*/
void blank_function()
{
	// Do nothing
}

void toggle_function()
{
	uToggle = uToggle == TOGGLE_OFF ? TOGGLE_ON : TOGGLE_OFF;
}

// ------ FUNC_ENTER_MENU ------
void goto_main_menu()
{
	menu_system.change_menu(main_menu);
	menu_system.update();
	menu_system.set_focusedLine(0);
}

void goto_machine_status_menu()
{
	menu_system.change_menu(machine_status_menu);
	menu_system.update();
	menu_system.set_focusedLine(0);
}

void activate_proc()
{
	currentProcessState.setState(ACTIVE_PROCESS);
	currentActiveProcessSubstate.setSubstate(PREHEATING);
	menu_system.change_menu(machine_status_menu);
	menu_system.update();
	menu_system.set_focusedLine(0);
}

void deactivate_proc()
{
	currentProcessState.setState(INACTIVE_PROCESS);
	currentActiveProcessSubstate.setSubstate(COOLING_DOWN);
	menu_system.change_menu(machine_status_menu);
	menu_system.update();
	menu_system.set_focusedLine(0);
}

// ------ FUNC_INCRT_PDATA and FUNC_DECRT_PDATA ------

// Increment set temperature
void incrt_pdata_setTemp()
{
	if (pData.setTemp + 5 <= MAX_TEMP)
		pData.setTemp += 5;
	menu_system.update();
}

// Decrement set temperature
void decrt_pdata_setTemp()
{
	if (pData.setTemp - 5 >= MIN_TEMP)
		pData.setTemp -= 5;
	menu_system.update();
}

// Increment control period
void incrt_pdata_controlPeriod()
{
	if (pData.controlPeriod + 60000 <= MAX_CONTROL_PERIOD)
		pData.controlPeriod += 60000;
	menu_system.update();
}

// Decrement control period
void decrt_pdata_controlPeriod()
{
	if (pData.controlPeriod - 60000 >= MIN_CONTROL_PERIOD)
		pData.controlPeriod -= 60000;
	menu_system.update();
}

// Increment heating duration
void incrt_pdata_heatingDuration()
{
	if (pData.heatingDuration + 60000 <= MAX_HEATING_DURATION)
		pData.heatingDuration += 60000;
	menu_system.update();
}

// Decrement heating duration
void decrt_pdata_heatingDuration()
{
	if (pData.heatingDuration - 60000 >= MIN_HEATING_DURATION)
		pData.heatingDuration -= 60000;
	menu_system.update();
}



/***************************************************************************************************
 * Click Button Encoder Event Processor                                                            *
 ***************************************************************************************************/
/**
 *  \brief    Processes the button and encoder events.
 *  \remarks  This function processes the button and encoder events and sets the user event
 *            variable accordingly.
 */

void encoderEvent()
{
	static unsigned long lastEncTime = 0;

	// Check rotary action first since button takes precendence and we want to avoid
	// overwriting the button event and missing a button event.
	encNewPos += encoder->getValue();

	if (encNewPos != encLastPos)
	{
		lastActiveTime = lastEncTime = millis();

		if (encNewPos < encLastPos)
		{
			uEvent = EV_ENCUP;
		}
		else if (encNewPos > encLastPos)
		{
			uEvent = EV_ENCDN;
		}

		encLastPos = encNewPos;
	}

	ClickEncoder::Button b = encoder->getButton();
	if (b != ClickEncoder::Open)
	{
		switch (b)
		{
		case ClickEncoder::Clicked:
			lastActiveTime = lastEncTime = millis();
			uEvent = EV_BTN_CLICKED;
			break;
		case ClickEncoder::DoubleClicked:
			lastActiveTime = lastEncTime = millis();
			uEvent = EV_BTN_2CLICKED;
			break;
		case ClickEncoder::Held:
			lastActiveTime = lastEncTime = millis();
			uEvent = EV_BTN_HELD;
			break;
		case ClickEncoder::Released:
			lastActiveTime = lastEncTime = millis();
			uEvent = EV_BTN_RELEASED;
			break;
		default:
			uEvent = EV_NONE;
			break;
		}
	}
}
#endif /* _LCDGUI_ */

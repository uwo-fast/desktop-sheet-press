// /***************************************************************************************************
//  * LCD User State Machine                                                                                    *
//  ***************************************************************************************************/
// /**
//  *  \brief  Implementation of the user state machine to navigate the GUI.
//  * 	USM = User State Machine
//  */


// uint8_t mState = ST_MAIN_MENU; /**< Current user machine state */


// // Machine states
// enum states
// {
// 	ST_SETUP_DISP, 			/**< User state: setup screen */

// 	ST_STANDBY_DISP, 		/**< User state: standby screen */

// 	ST_HOME_DISP, 			/**< User state: home screen */

// 	ST_MAIN_MENU,			/**< User state: main screen */
// 	ST_START_MENU,			/**< User state: start screen */
// 	ST_INFO_MENU,			/**< User state: information screen */
// 	ST_PRESETS_MENU,		/**< User state: presets screen */
// 	ST_SETTINGS_MENU,		/**< User state: settings screen */
// 	ST_SETPROCESS_MENU,		/**< User state: set process screen */
// 	ST_ACTIVESETTINGS_MENU, /**< User state: active settings screen */

// 	ST_SYSTEM_MENU, /**< User state: display system screen, currently only manual eeprom reset */
// };



// void lcdUserStateMachine()
// {
// 	static uint8_t currentMenuItemIndex = 0;
// 	static uint8_t mainMenuSelectionIndex = 0;
// 	static uint8_t subMenuSelectionIndex = 0;


// 	// Scan for events - the event queue length is one.
// 	// Process any public boot events. These events are the highest priority
// 	// and must be processed before any of the private events.
// 	if (mEvent == EV_BOOTDN)
// 	{
// 		mState = ST_SYSTEM_MENU;
// 		mEvent = EV_NONE;
// 	}
// 	else
// 	{
// 		// MACHINE EVENTS -----------------------
// 		switch (mEvent)
// 		{
// 		case EV_STBY_TIMEOUT:
// 			mState = ST_STANDBY_DISP;
// 			mEvent = EV_NONE;
// 			break;
// 		default:
// 			break;
// 		}

// 		// MACHINE STATES -----------------------
// 		switch (mState)
// 		{
// 		case ST_SYSTEM_MENU:

// 			break;

// 		case ST_SETUP_DISP:

// 			break;

// 		case ST_STANDBY_DISP:
		
// 			break;

// 		case ST_HOME_DISP:

// 			break;

// 		case ST_MAIN_MENU:
		
// 			break;

// 		case ST_START_MENU:
		
// 			break;

// 		case ST_INFO_MENU:
		
// 			break;
		
// 		case ST_PRESETS_MENU:
		
// 			break;

// 		case ST_SETTINGS_MENU:
		
// 			break;

// 		case ST_SETPROCESS_MENU:
		
// 			break;

// 		case ST_ACTIVESETTINGS_MENU:

// 			break;

// 		default:
// 			break;
// 		}
// 	}
// }


// /***************************************************************************************************
//  * LCD DISPLAYS SCREENS		                                                                       *
//  ***************************************************************************************************/
// /**
//  *  \brief    Display the SPLASH screen.
//  *  \remarks  This is shown once only at start-up and is for general information and advertising.
//  */
// void splash()
// {

// }


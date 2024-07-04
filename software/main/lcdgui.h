// Most of these macros define strings that appear on the splash screen during set up
// (each line of the splash screen is limited to 16 characters)
#define _DEVICENAME_ "OSCHSSP"
#define _PROGNAME_ "Press Controller"
#define _AUTHOR_ "CKB - MCW - JMP"
#define _VERSION_MAJOR_ 1
#define _VERSION_MINOR_ 1
#define _REVISION_ 0
#define _COPYRIGHT_ "2024"
#define _LICENSE_ "GNU V3 " FREE ""
#define _ORGANIZATION_ "FAST Research"


/***************************************************************************************************
 * Display Configuration                                                                       *
 ***************************************************************************************************/

#define SPLASHTIME 2500 /**< Splash screen time (ms) */


// User events
#ifdef _LCDGUI_
enum Event
{
    // Private user events
    EV_NONE,         /**< User event: no pending event */
    EV_BTN_CLICKED,  /**< User event: button pressed */
    EV_BTN_2CLICKED, /**< User event: button double pressed */
    EV_BTN_HELD,     /**< User event: button held */
    EV_BTN_RELEASED, /**< User event: button released */
    EV_ENCUP,        /**< User event: encoder rotate right */
    EV_ENCDN,        /**< User event: encoder rotate left */

    // Public user events
    EV_BOOTDN,      /**< User event: button pressed on boot, enter system menu */
    EV_STBY_TIMEOUT /**< User event: standby timer has timed out, enter standby screen */
};

enum ToggleState
{
    TOGGLE_OFF, // Indicates the toggle is off
    TOGGLE_ON   // Indicates the toggle is on
};

enum FunctionTypes
{
    FUNC_ENTER_MENU,  // Indicates a menu enter function, uToggle will not be set
    FUNC_INCRT_PDATA, // Indicates an increment function, uToggle must be set to TOGGLE_ON using button to increment
    FUNC_DECRT_PDATA, // Indicates a decrement function, uToggle must be set to TOGGLE_ON using button to decrement
};
#endif /* _LCDGUI_ */


#ifdef _LCDGUI_
// LCD GUI Functions
void updateLcdGui();
void lcdEventHandler();
void encoderEvent();
void checkSleep();

// LCD Screen Navigation and Action Functions
void goto_main_menu();
void goto_machine_status_menu();
void activate_proc();
void deactivate_proc();

// LCD Menu Adjustment Functions
void incrt_pdata_setTemp();
void decrt_pdata_setTemp();
void incrt_pdata_controlPeriod();
void decrt_pdata_controlPeriod();
void incrt_pdata_heatingDuration();
void decrt_pdata_heatingDuration();

// Misc. LCD GUI Functions
void toggle_function();
void blank_function();
#endif

// Macro masquerading as a function - makes the code more readable
/** This macro reads the state of the pushbutton switch on the encoder. */
#define btnState() (!digitalRead(PIN_SW))

// EEPROM Utility Functions
void resetEeprom(boolean full);
void loadEeprom();

#endif // MAIN_H
/***************************************************************************************************
 * Structure, union, and enumerated type definitions                                                *
 ***************************************************************************************************/

enum ProcessState
{
    INACTIVE_PROCESS, // Indicates an inactive process
    ACTIVE_PROCESS,   // Indicates an active process
    ERROR_PROCESS,    // Indicates an error in the process
    STANDBY_PROCESS   // Indicates the system is in standby
};

class ProcessStateWrapper
{
private:
    ProcessState state;

public:
    ProcessStateWrapper(ProcessState initialState) : state(initialState) {}

    // Method to set the underlying enum value
    void setState(ProcessState newState)
    {
        state = newState;
    }

    // Method to get the underlying enum value
    ProcessState getState() const
    {
        return state;
    }

    String toString() const
    {
        switch (state)
        {
        case INACTIVE_PROCESS:
            return "INACTIVE";
        case ACTIVE_PROCESS:
            return "ACTIVE";
        case ERROR_PROCESS:
            return "ERROR";
        case STANDBY_PROCESS:
            return "STANDBY";
        default:
            return "UNKNOWN";
        }
    }

    const char *toChar() const
    {
        switch (state)
        {
        case INACTIVE_PROCESS:
            return "INACTIVE";
        case ACTIVE_PROCESS:
            return "ACTIVE";
        case ERROR_PROCESS:
            return "ERROR";
        case STANDBY_PROCESS:
            return "STANDBY";
        default:
            return "UNKNOWN";
        }
    }
};

enum ActiveProcessSubstate
{
    UNKNOWN,     // Default or initial substate
    PREHEATING,  // Preheating phase of the active process
    HEATING,     // Main processing phase
    COOLING_DOWN // Cooling down phase after processing
};

class ActiveProcessSubstateWrapper
{
private:
    ActiveProcessSubstate substate;

public:
    ActiveProcessSubstateWrapper(ActiveProcessSubstate initialSubstate) : substate(initialSubstate) {}

    // Method to set the underlying enum value
    void setSubstate(ActiveProcessSubstate newSubstate)
    {
        substate = newSubstate;
    }

    // Method to get the underlying enum value
    ActiveProcessSubstate getSubstate() const
    {
        return substate;
    }

    String toString() const
    {
        switch (substate)
        {
        case UNKNOWN:
            return "UNKNOWN";
        case PREHEATING:
            return "PREHEATING";
        case HEATING:
            return "HEATING";
        case COOLING_DOWN:
            return "COOLING DOWN";
        default:
            return "UNDEFINED";
        }
    }

    const char *toChar() const
    {
        switch (substate)
        {
        case UNKNOWN:
            return "UNKNOWN";
        case PREHEATING:
            return "PREHEATING";
        case HEATING:
            return "HEATING";
        case COOLING_DOWN:
            return "COOLING DOWN";
        default:
            return "UNDEFINED";
        }
    }
};



#ifndef PID_v3_h
#define PID_v3_h

/**
 * @brief PID (Proportional-Integral-Derivative) Controller class.
 */
class PID
{
public:
  /**
   * @brief Modes for the PID controller.
   */
  enum Mode
  {
    Manual = 0,
    Automatic = 1
  };

  /**
   * @brief Directions for the PID controller.
   */
  enum Direction
  {
    Direct = 0,
    Reverse = 1
  };

  /**
   * @brief Proportional modes for the PID controller.
   */
  enum class P_On
  {
    Measurement = 0,
    Error = 1
  };

  /**
   * @brief Constructor for PID controller.
   *
   * @param Input Pointer to the input variable.
   * @param Output Pointer to the output variable.
   * @param Setpoint Pointer to the setpoint variable.
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   * @param pOn Proportional mode (Measurement or Error).
   * @param ControllerDirection Direction of the controller.
   */
  PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki,
      double Kd, P_On pOn, Direction ControllerDirection);

  /**
   * @brief Constructor for PID controller with default Proportional mode.
   *
   * @param Input Pointer to the input variable.
   * @param Output Pointer to the output variable.
   * @param Setpoint Pointer to the setpoint variable.
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   * @param ControllerDirection Direction of the controller.
   */
  PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki,
      double Kd, Direction ControllerDirection);

  /**
   * @brief Set the mode of the PID controller (Manual or Automatic).
   *
   * @param mode Mode to set.
   */
  void SetMode(Mode mode);

  /**
   * @brief Compute the PID output.
   *
   * @return True if the output was computed, false otherwise.
   */
  bool Compute();

  /**
   * @brief Set the output limits of the PID controller.
   *
   * @param Min Minimum output value.
   * @param Max Maximum output value.
   */
  void SetOutputLimits(double Min, double Max);

  /**
   * @brief Set the tuning parameters of the PID controller.
   *
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   */
  void SetTunings(double Kp, double Ki, double Kd);

  /**
   * @brief Set the tuning parameters of the PID controller with proportional
   * mode.
   *
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   * @param pOn Proportional mode (Measurement or Error).
   */
  void SetTunings(double Kp, double Ki, double Kd, P_On pOn);

  /**
   * @brief Set the direction of the PID controller.
   *
   * @param Direction Direction to set.
   */
  void SetControllerDirection(Direction Direction);

  /**
   * @brief Set the sample time for the PID controller.
   *
   * @param NewSampleTime Sample time in milliseconds.
   */
  void SetSampleTime(int NewSampleTime);

  /**
   * @brief Get the proportional gain.
   *
   * @return Proportional gain.
   */
  double GetKp() const;

  /**
   * @brief Get the integral gain.
   *
   * @return Integral gain.
   */
  double GetKi() const;

  /**
   * @brief Get the derivative gain.
   *
   * @return Derivative gain.
   */
  double GetKd() const;

  /**
   * @brief Get the current mode of the PID controller.
   *
   * @return Current mode.
   */
  Mode GetMode() const;

  /**
   * @brief Get the current direction of the PID controller.
   *
   * @return Current direction.
   */
  Direction GetDirection() const;

  /**
   * @brief Get the last proportional term used in computation.
   *
   * @return Last proportional term.
   */
  double GetLastP() const;

  /**
   * @brief Get the last integral term used in computation.
   *
   * @return Last integral term.
   */
  double GetLastI() const;

  /**
   * @brief Get the last derivative term used in computation.
   *
   * @return Last derivative term.
   */
  double GetLastD() const;

private:
  /**
   * @brief Initialize the PID controller.
   */
  void Initialize();

  double dispKp; ///< Proportional tuning parameter for display.
  double dispKi; ///< Integral tuning parameter for display.
  double dispKd; ///< Derivative tuning parameter for display.

  double kp; ///< Proportional tuning parameter.
  double ki; ///< Integral tuning parameter.
  double kd; ///< Derivative tuning parameter.

  Direction controllerDirection; ///< Direction of the controller.
  P_On pOn;                      ///< Proportional mode.

  double *myInput;    ///< Pointer to the input variable.
  double *myOutput;   ///< Pointer to the output variable.
  double *mySetpoint; ///< Pointer to the setpoint variable.

  unsigned long lastTime; ///< Last time the PID computation was performed.
  double outputSum;        ///< Sum of the output.
  double lastInput;        ///< Last input value.
  double lastP;            ///< Last proportional term.
  double lastD;            ///< Last derivative term.

  unsigned long SampleTime; ///< Sample time in milliseconds.
  double outMin;             ///< Minimum output value.
  double outMax;             ///< Maximum output value.
  bool inAuto;              ///< True if the controller is in automatic mode.
  bool pOnE;                ///< True if proportional on error mode is set.
};

/**
 * @brief PID_v2 class, inheriting from PID class.
 */
class PID_v2 : public PID
{
public:
  /**
   * @brief Constructor for PID_v2 class.
   *
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   * @param dir Direction of the controller.
   * @param POn Proportional mode (default is Error).
   */
  PID_v2(double Kp, double Ki, double Kd, Direction dir, P_On POn = P_On::Error)
      : PID(&this->input, &this->output, &this->setpoint, Kp, Ki, Kd, POn,
            dir) {}

  /**
   * @brief Set the setpoint value.
   *
   * @param v Setpoint value.
   */
  void Setpoint(double v) { this->setpoint = v; }

  /**
   * @brief Get the current setpoint value.
   *
   * @return Setpoint value.
   */
  double GetSetpoint() const { return this->setpoint; }

  /**
   * @brief Start the PID controller with initial values.
   *
   * @param input_ Initial input value.
   * @param currentOutput Initial output value.
   * @param setpoint_ Initial setpoint value.
   */
  void Start(double input_, double currentOutput, double setpoint_)
  {
    this->input = input_;
    this->output = currentOutput;
    this->setpoint = setpoint_;
    this->SetMode(Mode::Automatic);
  }

  /**
   * @brief Run the PID computation with the given input.
   *
   * @param input_ Input value.
   * @return Computed output value.
   */
  double Run(double input_)
  {
    this->input = input_;
    this->Compute();
    return this->output;
  }

private:
  double input;    ///< Current input value.
  double output;   ///< Current output value.
  double setpoint; ///< Current setpoint value.
};

#ifndef PID_v3_SKIP_COMPAT_WITH_v1
const PID::Mode AUTOMATIC = PID::Automatic;
const PID::Mode MANUAL = PID::Manual;
const PID::Direction DIRECT = PID::Direct;
const PID::Direction REVERSE = PID::Reverse;
const PID::P_On P_ON_M = PID::P_On::Measurement;
const PID::P_On P_ON_E = PID::P_On::Error;
#endif

#endif // PID_v3_h
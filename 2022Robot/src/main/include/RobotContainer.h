#pragma once

//Include header files
#include "Robot.h"
#include "mainAuto.h"
#include "Taxi.h"
#include "ScoreOne.h"
#include <frc/smartdashboard/SendableChooser.h>

#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Command.h>
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>
#include <sstream>
#include <iostream>
#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/PowerDistribution.h>
#include <frc/Joystick.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/AnalogGyro.h>
#include <frc/Solenoid.h>
#include <frc/Compressor.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/PneumaticsModuleType.h>
#include <frc/PneumaticsBase.h>
#include <frc/CompressorConfigType.h>
#include <WPILibVersion.h>
#include <cmath>
#include <math.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include "cameraserver/CameraServer.h"
#include <chrono>
#include <ctime>
#include <ratio>

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/CommandScheduler.h>
#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer;

class RobotContainer {
 public:

  void testRobotContainer();
  void FrontLeft();
  void LeftMotorDrive(double speed);
  void RightMotorDrive(double speed);
  void IntakeMotors(double speed);
  void Conveyor(double speed);
  void Shooter(double speed);
  void Climb(double speed);

  frc2::Command* GetAutonomousCommand();

  private:
  // The robot's subsystems and commands are defined here...
  
  mainAuto m_MainAuto;
  Taxi m_TaxiAuto;
  ScoreOne m_ScoreOneAuto;

};
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Include header files
#include "Robot.h"
#include "RobotContainer.h"

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


using namespace std::chrono;

//Declarations

//Joysticks
frc::Joystick JoyStick1(0), Wheel(3), Xbox(1);

//Drive Motors
TalonFX FrontLeftMotor {15};
TalonFX MiddleLeftMotor {14};
TalonFX BackLeftMotor {13};

TalonFX FrontRightMotor {3};
TalonFX MiddleRightMotor {2};
TalonFX BackRightMotor {1};

//Intake Motor
TalonFX IntakeMotorOne {0};
TalonFX IntakeMotorTwo {11};

//Shooter Motors
VictorSPX ConveyorMotor1 {4};
VictorSPX ConveyorMotor3 {5};

TalonFX ShooterMotor1 {10};
TalonFX ShooterMotor2 {9};

//Climb Motors
TalonFX LeftClimbMotor {7};
TalonFX RightClimbMotor {8};

//set amp limit
bool enable = true;
double currentLimit = 30;
double triggerThresholdCurrent = 30;
double triggerThresholdTime = .1;

//Pneumatics
frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};
frc::Solenoid IntakeBar{frc::PneumaticsModuleType::CTREPCM, 5};

//Gyro
WPI_PigeonIMU gyro{6};

//PID (Proportional, Integral, Derivative) to calculate error and overshoot and correct it
double PVal = 0.1;

//Auto Variables
double LeftDriveEncValue;
double RightDriveEncValue;
double avgEncValue;

double setpoint;
double distError;
double turnError;

double turnPIDOutput;
double distPIDOutput;

int autoStep;
bool timeStampChecked = true;
bool isShooterRunning = false;

steady_clock::time_point clock_begin;

RobotContainer m_container;
RobotContainer chooseAuto;

void RobotContainer::LeftMotorDrive(double speed) {
  FrontLeftMotor.Set(ControlMode::PercentOutput, speed);
  MiddleLeftMotor.Set(ControlMode::PercentOutput, speed);
  BackLeftMotor.Set(ControlMode::PercentOutput, speed);
}
void RobotContainer::RightMotorDrive(double speed) { 
  FrontRightMotor.Set(ControlMode::PercentOutput, speed);
  MiddleRightMotor.Set(ControlMode::PercentOutput, speed);
  BackRightMotor.Set(ControlMode::PercentOutput, speed);
}
void RobotContainer::IntakeMotors(double speed) {
  IntakeMotorOne.Set(ControlMode::PercentOutput, speed);
  IntakeMotorTwo.Set(ControlMode::PercentOutput, speed);
}
void RobotContainer::Conveyor(double speed) {
  ConveyorMotor1.Set(ControlMode::PercentOutput, speed);
  ConveyorMotor3.Set(ControlMode::PercentOutput, speed);
}
void RobotContainer::Shooter(double speed) {
  ShooterMotor1.Set(ControlMode::PercentOutput, speed);
  ShooterMotor2.Set(ControlMode::PercentOutput, speed);
}
void RobotContainer::Climb(double speed) {
  LeftClimbMotor.Set(ControlMode::PercentOutput, speed);
  RightClimbMotor.Set(ControlMode::PercentOutput, speed);
}

void RobotContainer::testRobotContainer() {

  // Add commands to the autonomous command chooser
  /*m_chooser.SetDefaultOption("Main Auto", &m_MainAuto);
  m_chooser.AddOption("Taxi Auto", &m_TaxiAuto);
  m_chooser.AddOption("Shoot One Auto", &m_ScoreOneAuto);*/
  frc::SmartDashboard::PutNumber("Auto: 1=Main, 2=ScoreOne, 3=Taxi", 1);
}

//Initializing robot & variables
void Robot::RobotInit() {
  //camera
  frc::CameraServer::StartAutomaticCapture();

  // The chooser for the autonomous routines (put into RobotContainer)
  chooseAuto.testRobotContainer();

  FrontLeftMotor.SetInverted(true);
  MiddleLeftMotor.SetInverted(true);
  BackLeftMotor.SetInverted(true);

  ShooterMotor2.SetInverted(true);

  //Drop Intake Bar at the beginning of match
  IntakeBar.Set(true);
  
  FrontLeftMotor.SetSelectedSensorPosition(0);
  MiddleLeftMotor.SetSelectedSensorPosition(0);
  BackLeftMotor.SetSelectedSensorPosition(0);

  FrontRightMotor.SetSelectedSensorPosition(0);
  MiddleRightMotor.SetSelectedSensorPosition(0);
  BackRightMotor.SetSelectedSensorPosition(0);

  LeftClimbMotor.SetSelectedSensorPosition(0);
  RightClimbMotor.SetSelectedSensorPosition(0);

  clock_begin = steady_clock::now();
  
  //Set Current limits for intake and drivetrain motors
  SupplyCurrentLimitConfiguration current_limit_config(enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
  IntakeMotorOne.ConfigSupplyCurrentLimit(current_limit_config);

  FrontLeftMotor.ConfigSupplyCurrentLimit(current_limit_config);
  MiddleLeftMotor.ConfigSupplyCurrentLimit(current_limit_config);
  BackLeftMotor.ConfigSupplyCurrentLimit(current_limit_config);

  FrontRightMotor.ConfigSupplyCurrentLimit(current_limit_config);
  MiddleRightMotor.ConfigSupplyCurrentLimit(current_limit_config);
  BackRightMotor.ConfigSupplyCurrentLimit(current_limit_config);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
}

/**
 * This autonomous (alon)g with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */

void Robot::AutonomousInit() {
  //fmt::print("Auto selected: {}\n", m_autoSelected);

  clock_begin = steady_clock::now();
  
  FrontLeftMotor.SetSelectedSensorPosition(0);
  MiddleLeftMotor.SetSelectedSensorPosition(0);
  BackLeftMotor.SetSelectedSensorPosition(0);

  FrontRightMotor.SetSelectedSensorPosition(0);
  MiddleRightMotor.SetSelectedSensorPosition(0);
  BackRightMotor.SetSelectedSensorPosition(0);

  ShooterMotor1.SetSelectedSensorPosition(0);
  ShooterMotor2.SetSelectedSensorPosition(0);

  LeftClimbMotor.SetSelectedSensorPosition(0);
  RightClimbMotor.SetSelectedSensorPosition(0);

  frc::CameraServer::StartAutomaticCapture();

  autoStep = 1;

  IntakeBar.Set(true);

  gyro.Reset();

  /*m_autonomousCommand = m_container.GetAutonomousCommand();
  
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }*/

}


void Robot::AutonomousPeriodic() {
  //frc2::CommandScheduler::GetInstance().Run();

  //553.1248 cycles of the encoder per in. ---> Multiply by # of inches to find encoder units
  RobotContainer Auto;

  //Next steps: fixing drifting
  LeftDriveEncValue = FrontLeftMotor.GetSelectedSensorPosition();
  RightDriveEncValue = FrontRightMotor.GetSelectedSensorPosition();

  double avgEncValue = (FrontLeftMotor.GetSelectedSensorPosition() + FrontRightMotor.GetSelectedSensorPosition())/2;

  steady_clock::time_point clock_end = steady_clock::now();
  steady_clock::duration time_span = clock_end - clock_begin;

  double seconds = double(time_span.count()) * steady_clock::period::num / steady_clock::period::den;

  std::cout << "Avg Enc Value: " << ((FrontLeftMotor.GetSelectedSensorPosition() + FrontRightMotor.GetSelectedSensorPosition())/2) << std::endl;

  /*distError = (setpoint - avgEncValue);
  turnError = (setpoint - gyro.GetAngle());

  turnPIDOutput = PVal * turnError;
  distPIDOutput = PVal * distError;

  if (turnPIDOutput > 0.25) {
    turnPIDOutput = 0.25;
  } else if (distPIDOutput > 0.25) {
    distPIDOutput = 0.25;
  }*/
  
  //Main Auto - Grab 2 cargo, Score those, Grab 2 more cargo, Score those
  if (frc::SmartDashboard::GetNumber("Auto", 1) == 1) {
    if (avgEncValue < 21500 && (autoStep == 1)) {
        //std::cout << "Encoder Value: " << FrontLeftMotor.GetSelectedSensorPosition() << std::endl;
        
        Auto.LeftMotorDrive(0.2);
        Auto.RightMotorDrive(0.2);
        Auto.Shooter(0.7); 
        Auto.IntakeMotors(-0.5);
        
        autoStep = 1;
    } else if (avgEncValue >= 21500 && autoStep <= 2 && seconds < 4.5) {
        Auto.LeftMotorDrive(0);
        Auto.RightMotorDrive(0);
        Auto.Conveyor(-0.2);
        Auto.Shooter(0.8);
        Auto.IntakeMotors(-0.3);
        //IntakeBar.Set(false);
        //std::cout << "Step 2" << std::endl;
        autoStep = 2;
    } else if (seconds > 4.5 && autoStep >= 2) {
        //Turn 30 degrees
        if (avgEncValue >= -12000 && autoStep <= 3) {
          Auto.LeftMotorDrive(-0.15);
          Auto.RightMotorDrive(-0.15);
          Auto.IntakeMotors(0);
          Auto.Conveyor(0);
          Auto.Shooter(0);
          IntakeBar.Set(false);
          autoStep = 3;
        } else if (avgEncValue < -12000 && gyro.GetAngle() < 33 && autoStep >= 3 && autoStep < 5) {
          Auto.RightMotorDrive(-0.15);
          Auto.LeftMotorDrive(0.15);
          autoStep = 4;
          //std::cout << "Turn 33" << std::endl;
        } else if (avgEncValue < 20000 && gyro.GetAngle() > 33 && autoStep > 3 && autoStep <= 5) {
          Auto.LeftMotorDrive(0.2);
          Auto.RightMotorDrive(0.2);
          IntakeBar.Set(true);
          Auto.IntakeMotors(-0.5);
          autoStep = 5;
          //std::cout << "Drive to cargo" << std::endl;
        } else if (avgEncValue > 20000 && gyro.GetAngle() > 33 && gyro.GetAngle() < 65 && autoStep > 4 && autoStep < 6) {
              Auto.LeftMotorDrive(0.15);
              Auto.RightMotorDrive(-0.15);
              Auto.IntakeMotors(-0.35);
              //std::cout << "Turn to 65" << std::endl;
        } else if (avgEncValue > 20000 && avgEncValue < 96500 && gyro.GetAngle() >= 65 && autoStep >= 4 && autoStep < 6) {
              Auto.LeftMotorDrive(0.2);
              Auto.RightMotorDrive(0.2);
              Auto.IntakeMotors(-0.5);
              //std::cout << "Drive to terminal" << std::endl;
        } else if (autoStep >= 5 && avgEncValue > 96500) {
            Auto.LeftMotorDrive(-0.2);
            Auto.RightMotorDrive(-0.2);
            Auto.IntakeMotors(-0.3);
            Auto.Shooter(0.65);
            //std::cout << "Drive back to hub" << std::endl;
            autoStep = 6;
          } else if (autoStep == 6 && avgEncValue < 21500) {
            Auto.LeftMotorDrive(0);
            Auto.RightMotorDrive(0);
            Auto.IntakeMotors(-0.3);
            Auto.Conveyor(-0.4);
            Auto.Shooter(0.8);
            //std::cout << "Shoot two more cargo" << std::endl;
          }
      } 
  }

  //Auto - Two ball (lower hub)
  if (frc::SmartDashboard::GetNumber("Auto", 1) == 2) {
    if (avgEncValue < 42500 && (autoStep == 1)) {
          //std::cout << "Encoder Value: " << FrontLeftMotor.GetSelectedSensorPosition() << std::endl;
          setpoint = 6637;
          
          Auto.LeftMotorDrive(0.2);
          Auto.RightMotorDrive(0.2);
          Auto.Shooter(0); 
          Auto.IntakeMotors(-0.5);

          autoStep = 1;
      } else if (avgEncValue >= 42500 && autoStep <= 2) {
          autoStep = 2;
            
          Auto.LeftMotorDrive(-0.2);
          Auto.RightMotorDrive(-0.2);
          Auto.Conveyor(-0.1);
          Auto.Shooter(0);
      } else if (avgEncValue <= 500 && (autoStep >= 2) && seconds < 8) {
          Auto.LeftMotorDrive(0);
          Auto.RightMotorDrive(0);
          Auto.Shooter(0.35);
          Auto.Conveyor(-0.5);
          //figure out how many encoder rotations are equivalent to shooting two cargo and use that as qualification for next else if statement
          Auto.IntakeMotors(0);
          autoStep = 3;
          //std::cout << "Shooter Encoder Value: " << ShooterMotor1.GetSelectedSensorPosition() << std::endl;
      }
  }

  //Auto - Two Ball (upper hub)
  if (frc::SmartDashboard::GetNumber("Auto", 1) == 3) {
    if (avgEncValue < 21500 && (autoStep == 1) && seconds < 4) {
          //std::cout << "Encoder Value: " << FrontLeftMotor.GetSelectedSensorPosition() << std::endl;
          setpoint = 6637;
          
          Auto.LeftMotorDrive(0.15);
          Auto.RightMotorDrive(0.15);
          Auto.Shooter(0.65); 
          Auto.IntakeMotors(-0.5);
          Auto.Conveyor(0);

          autoStep = 1;
      } else if (avgEncValue >= 21500 && autoStep <= 2 && seconds < 5) {
          autoStep = 2;
 
          Auto.LeftMotorDrive(0);
          Auto.RightMotorDrive(0);
          Auto.Shooter(0.8);
          Auto.Conveyor(-0.4);
          Auto.IntakeMotors(-0.4);
      } else {
          Auto.Shooter(0);
          Auto.IntakeMotors(0);
          Auto.Conveyor(0);
          IntakeBar.Set(false);
      }

  //Auto - Score One + Taxi
  if (frc::SmartDashboard::GetNumber("Auto", 1) == 4) {
    if (seconds < 13) {
      std::cout << "Less than 13 s" << std::endl;
      Auto.Shooter(0.35);
      Auto.Conveyor(-0.5);
      Auto.LeftMotorDrive(0);
      Auto.RightMotorDrive(0);
    } else if (FrontLeftMotor.GetSelectedSensorPosition() < 70000) {
      std::cout << "Else is running (past 13 sec)" << std::endl;
      Auto.Shooter(0);
      Auto.Conveyor(0);
      Auto.LeftMotorDrive(0.4);
      Auto.RightMotorDrive(0.4);
    } else {
      Auto.Shooter(0);
      Auto.Conveyor(0);
      Auto.LeftMotorDrive(0);
      Auto.RightMotorDrive(0);
    }
  }
}
}

void Robot::TeleopInit() {
  /*FrontRightMotor.SetInverted(true);
  MiddleRightMotor.SetInverted(true);
  BackRightMotor.SetInverted(true);*/

  ShooterMotor2.SetInverted(true);

  /*if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }*/

}

void Robot::TeleopPeriodic() {

  RobotContainer TeleOp;

  double JoyY = -JoyStick1.GetY();
  double WheelX = Wheel.GetX();

  //Drive Code
  if ((WheelX > 0.05 || WheelX < -0.05) && (JoyY > 0.1 || JoyY < -0.1)) {
    TeleOp.LeftMotorDrive((JoyY*0.45) + (0.3*WheelX));
    TeleOp.RightMotorDrive((JoyY*0.45) - (0.3*WheelX));
  } else if (JoyY > 0.1 || JoyY < -0.1) {
    TeleOp.LeftMotorDrive((JoyY * 0.45));
    TeleOp.RightMotorDrive((JoyY * 0.45));
  } // Point turning
    else if (Wheel.GetRawButton(5)) {
    if (WheelX > 0) {
      TeleOp.RightMotorDrive(-(WheelX * WheelX));
      TeleOp.LeftMotorDrive((WheelX * WheelX));
    } else if (WheelX < 0) {
      TeleOp.RightMotorDrive((WheelX * WheelX));
      TeleOp.LeftMotorDrive(-(WheelX * WheelX));
    }
  } else {
    TeleOp.LeftMotorDrive(0);
    TeleOp.RightMotorDrive(0);
  } 

  //Intake Code (button X to intake & run conveyor, button A to spit) & Shooter+Conveyor Code (button Y)
  if(Xbox.GetRawAxis(2) > 0) {
    IntakeBar.Set(true);
    TeleOp.IntakeMotors(-0.45);
    TeleOp.Conveyor(-0.1);
    TeleOp.Shooter(0);
  } else if (Xbox.GetRawButton(4)) {
    IntakeBar.Set(true);
    TeleOp.IntakeMotors(0.45);
    TeleOp.Conveyor(0.5);
    TeleOp.Shooter(0);
  } else if (Xbox.GetRawButton(2)) {
    TeleOp.Conveyor(-0.65);
    TeleOp.IntakeMotors(0);
    TeleOp.Shooter(0);
  } else if(Xbox.GetRawButton(1)) {
    TeleOp.Shooter(0.35);
    TeleOp.Conveyor(-0.5);
    TeleOp.IntakeMotors(0);
  } else if (Xbox.GetRawAxis(3) > 0) {
    TeleOp.Shooter(0.8);
    TeleOp.Conveyor(-0.5);
    TeleOp.IntakeMotors(0);
  } else if(Xbox.GetPOV() == 270){
    IntakeBar.Set(true);
    TeleOp.IntakeMotors(-0.45);
    TeleOp.Shooter(0.2);
    TeleOp.Conveyor(-0.5);
  } else {
    TeleOp.IntakeMotors(0);
    TeleOp.Conveyor(0);
    TeleOp.Shooter(0);
    IntakeBar.Set(false);
  }

  //Climb Code (Left Button brings climber up, Right button brings climber down)
  if(Xbox.GetRawButton(5)) {
    TeleOp.Climb(0.45);
  } else if(Xbox.GetRawButton(6)) {
    TeleOp.Climb(-0.35);
  } else if (Xbox.GetRawButtonPressed(8)) {
    //Button closest to xbox controller
    LeftClimbMotor.SetNeutralMode(Brake);
    RightClimbMotor.SetNeutralMode(Brake);
  } else {
    TeleOp.Climb(0);
  }

  /*//Brings intake bar down (Owen + Alex's work)
  if(Xbox.GetRawButton(2)) {
    IntakeBar.Set(true);
    TeleOp.IntakeMotors(-0.5);
  } else {
    TeleOp.IntakeMotors(0);
    IntakeBar.Set(false);
  }*/
   
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  FrontLeftMotor.SetSelectedSensorPosition(0);
  MiddleLeftMotor.SetSelectedSensorPosition(0);
  BackLeftMotor.SetSelectedSensorPosition(0);

  FrontRightMotor.SetSelectedSensorPosition(0);
  MiddleRightMotor.SetSelectedSensorPosition(0);
  BackRightMotor.SetSelectedSensorPosition(0);

  gyro.Reset();
  gyro.Calibrate();
}

void Robot::TestPeriodic() {
  /*else if(Xbox.GetRawButton(8)) {
    if(LeftDriveEncValue > 6660) {
      LeftMotorDrive(-0.5);
      RightMotorDrive(-0.5);
    }
  }*/


  /*//Determine tolerance of gyro
  if (gyro.GetAngle() < 90) {
    setpoint = 90;
    LeftMotorDrive(PIDOutput/5);
    RightMotorDrive(-PIDOutput/5);
    //LeftMotorDrive(0.1);
    //RightMotorDrive(-0.1);
    if (gyro.GetAngle() > 90) {
      LeftMotorDrive(-0.1);
      RightMotorDrive(0.1);
    }
  }
  else {
    LeftMotorDrive(0);
    RightMotorDrive(0);
  }*/


  //Timer test for intake
  /*steady_clock::time_point clock_end = steady_clock::now();
  steady_clock::duration time_span = clock_end - clock_begin;

  double seconds = double(time_span.count()) * steady_clock::period::num / steady_clock::period::den;*/

  //std::cout << "Time (seconds)" << seconds << std::endl;

  /*if (seconds < 6) {
    Intake (0.75);
  } else {
    Intake (0);
  }*/
  
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

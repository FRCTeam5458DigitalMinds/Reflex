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
#include <frc/BuiltInAccelerometer.h>
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
frc::Joystick JoyStick1(0), Wheel(2), Xbox(1);

//Drive Motors
TalonFX FrontLeftMotor {15};
TalonFX MiddleLeftMotor {14};
TalonFX BackLeftMotor {13};

TalonFX FrontRightMotor {3};
TalonFX MiddleRightMotor {2};
TalonFX BackRightMotor {1};

//Intake Motor
TalonFX LeftIntake {0};
TalonFX RightIntake {11};

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
frc::Solenoid IntakePistons{frc::PneumaticsModuleType::CTREPCM, 5};
frc::Solenoid ClimbPiston{frc::PneumaticsModuleType::CTREPCM, 3}; 

//Gyro + Accelerometer
WPI_PigeonIMU gyro{6};
frc::BuiltInAccelerometer accelerometer{};

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

frc::Timer *shooterTimer;
bool timerStarted = false;

bool gyroResetted = false;


//RobotContainer m_container;
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
  LeftIntake.Set(ControlMode::PercentOutput, speed);
  RightIntake.Set(ControlMode::PercentOutput, speed);
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
  frc::SmartDashboard::PutNumber("Auto: 1=Main, 2=ScoreOne, 3=Taxi", 3);
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

  LeftIntake.SetInverted(true);
  ShooterMotor2.SetInverted(true);

  //Drop Intake Bar at the beginning of match 
  IntakePistons.Set(true);
  
  FrontLeftMotor.SetSelectedSensorPosition(0);
  MiddleLeftMotor.SetSelectedSensorPosition(0);
  BackLeftMotor.SetSelectedSensorPosition(0);

  FrontRightMotor.SetSelectedSensorPosition(0);
  MiddleRightMotor.SetSelectedSensorPosition(0);
  BackRightMotor.SetSelectedSensorPosition(0);

  LeftClimbMotor.SetSelectedSensorPosition(0);
  RightClimbMotor.SetSelectedSensorPosition(0);

  clock_begin = steady_clock::now();

  frc::SmartDashboard::PutNumber("EncVal: ", LeftClimbMotor.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("Gyro Angle", gyro.GetAngle());
  std::cout << "Gyro Angle" << gyro.GetAngle() << std::endl;

  //Set Current limits for intake and drivetrain motors
  SupplyCurrentLimitConfiguration current_limit_config(enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
  LeftIntake.ConfigSupplyCurrentLimit(current_limit_config);
  RightIntake.ConfigSupplyCurrentLimit(current_limit_config);

  FrontLeftMotor.ConfigSupplyCurrentLimit(current_limit_config);
  MiddleLeftMotor.ConfigSupplyCurrentLimit(current_limit_config);
  BackLeftMotor.ConfigSupplyCurrentLimit(current_limit_config);

  FrontRightMotor.ConfigSupplyCurrentLimit(current_limit_config);
  MiddleRightMotor.ConfigSupplyCurrentLimit(current_limit_config);
  BackRightMotor.ConfigSupplyCurrentLimit(current_limit_config);

  shooterTimer = new frc::Timer();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */

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

  IntakePistons.Set(true);

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
  if (frc::SmartDashboard::GetNumber("Auto", 3) == 1) {
    if (avgEncValue < 22000 && (autoStep == 1)) {
        //std::cout << "Encoder Value: " << FrontLeftMotor.GetSelectedSensorPosition() << std::endl;
        
        Auto.LeftMotorDrive(0.25);
        Auto.RightMotorDrive(0.25);
        Auto.Conveyor(0);
        Auto.Shooter(0.75); 
        Auto.IntakeMotors(-0.5);
        
        autoStep = 1;
    } else if (avgEncValue >= 22000 && autoStep <= 2 && seconds < 3.5) {
        Auto.LeftMotorDrive(0);
        Auto.RightMotorDrive(0);
        Auto.Conveyor(-0.45);
        Auto.Shooter(0.85);
        Auto.IntakeMotors(-0.3);
        //IntakePistons.Set(false);
        //std::cout << "Step 2" << std::endl;
        autoStep = 2;
    } else if (seconds > 3.5 && autoStep >= 2) {
        //Turn 30 degrees
        if (avgEncValue >= -11900 && autoStep <= 3) {
          Auto.LeftMotorDrive(-0.15);
          Auto.RightMotorDrive(-0.15);
          Auto.IntakeMotors(0);
          Auto.Conveyor(0);
          Auto.Shooter(0);
          IntakePistons.Set(false);
          autoStep = 3;
        } else if (avgEncValue < -11900 && gyro.GetAngle() < 33 && autoStep >= 3 && autoStep < 5) {
          Auto.RightMotorDrive(-0.15);
          Auto.LeftMotorDrive(0.15);
          autoStep = 4;
          //std::cout << "Turn 33" << std::endl;
        } else if (avgEncValue < 20500 && gyro.GetAngle() > 33 && autoStep > 3 && autoStep <= 5) {
          Auto.LeftMotorDrive(0.25);
          Auto.RightMotorDrive(0.25);
          IntakePistons.Set(true);
          Auto.IntakeMotors(-0.5);
          autoStep = 5;
          //std::cout << "Drive to cargo" << std::endl;
        } else if (avgEncValue > 20500 && gyro.GetAngle() > 33 && gyro.GetAngle() < 69 && autoStep > 4 && autoStep < 6) {
              Auto.LeftMotorDrive(0.15);
              Auto.RightMotorDrive(-0.15);
              Auto.IntakeMotors(-0.35);
              //std::cout << "Turn to 65" << std::endl;
        } else if (avgEncValue > 20500 && avgEncValue < 100500 && gyro.GetAngle() >= 69 && autoStep >= 4 && autoStep < 6) {
              Auto.LeftMotorDrive(0.25);
              Auto.RightMotorDrive(0.25);
              Auto.IntakeMotors(-0.5);
              Auto.Conveyor(-0.15);
              //std::cout << "Drive to terminal" << std::endl;
        } else if (avgEncValue > 98500 && avgEncValue < 120500 && autoStep >= 4 && autoStep < 6) {
              Auto.LeftMotorDrive(0.15);
              Auto.RightMotorDrive(0.17);
              Auto.IntakeMotors(-0.5);
              Auto.Conveyor(-0.15);
        } else if (autoStep >= 5 && avgEncValue > 120500) {
            Auto.LeftMotorDrive(-0.25);
            Auto.RightMotorDrive(-0.25);
            Auto.IntakeMotors(-0.35);
            Auto.Conveyor(0);
            Auto.Shooter(0.8);
            //std::cout << "Drive back to hub" << std::endl;
            autoStep = 6;
          } else if (autoStep == 6 && avgEncValue < 43500) {
            std::cout << "Shoot two more cargo" << std::endl; 
            if (gyro.GetAngle() > 50) {
              Auto.LeftMotorDrive(-0.15);
              Auto.RightMotorDrive(0.15);
              Auto.Shooter(0.65);
              Auto.IntakeMotors(-0.3);
            }
            else if (gyro.GetAngle() <= 50) {
              Auto.LeftMotorDrive(0);
              Auto.RightMotorDrive(0);
              Auto.IntakeMotors(0);
              Auto.Conveyor(-0.4);
              Auto.Shooter(0.8);
            }
          }
      }
       
  }

//Main Auto - Shot 2 pre-loaded cargo, Score those, Grab 2 more cargo, Score those



  //Auto - Two ball (lower hub)
  if (frc::SmartDashboard::GetNumber("Auto", 3) == 5) {
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
  if (frc::SmartDashboard::GetNumber("Auto", 3) == 3) {
    if (avgEncValue < 21800 && (autoStep == 1) && seconds < 5) {
          //std::cout << "Encoder Value: " << FrontLeftMotor.GetSelectedSensorPosition() << std::endl;
          setpoint = 6637;
          
          Auto.LeftMotorDrive(0.2);
          Auto.RightMotorDrive(0.2);
          Auto.Shooter(0.75); 
          Auto.IntakeMotors(-0.5);
          Auto.Conveyor(0);

          autoStep = 1;
      } else if (avgEncValue >= 21800 && autoStep <= 2 && seconds < 6) {
          autoStep = 2;
 
          Auto.LeftMotorDrive(0);
          Auto.RightMotorDrive(0);
          Auto.Shooter(0.79);
          Auto.Conveyor(-0.4);
          Auto.IntakeMotors(-0.4);
      } else {
          Auto.Shooter(0);
          Auto.IntakeMotors(0);
          Auto.Conveyor(0);
          IntakePistons.Set(false);
      }
    }
  //Auto - Score One + Taxi
  if (frc::SmartDashboard::GetNumber("Auto", 3) == 5) {
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



  //5 Ball Auto
  if(frc::SmartDashboard::GetNumber("Auto", 3) == 6) {
    if (avgEncValue < 22000 && (autoStep == 1)) {
        //std::cout << "Encoder Value: " << FrontLeftMotor.GetSelectedSensorPosition() << std::endl;
        
        Auto.LeftMotorDrive(0.25);
        Auto.RightMotorDrive(0.25);
        Auto.Conveyor(0);
        Auto.Shooter(0.75); 
        Auto.IntakeMotors(-0.5);
        
        autoStep = 1;
    } else if (avgEncValue >= 22000 && autoStep <= 2 && seconds < 3.5) {
        Auto.LeftMotorDrive(0);
        Auto.RightMotorDrive(0);
        Auto.Conveyor(-0.45);
        Auto.Shooter(0.8);
        Auto.IntakeMotors(-0.3);
        //IntakePistons.Set(false);
        //std::cout << "Step 2" << std::endl;
        autoStep = 2;
    } else if (seconds > 3.5 && autoStep >=2) {
      if(gyro.GetAngle() < 100 && autoStep <= 3) {
        IntakePistons.Set(false);
        Auto.Shooter(0);
        Auto.Conveyor(0);
        Auto.IntakeMotors(0);
        Auto.LeftMotorDrive(0.15);
        Auto.RightMotorDrive(-0.15);
        autoStep = 3;
        std::cout << "Turn 105 deg" << std::endl;
      } else if (avgEncValue < 43500 && gyro.GetAngle() >= 100 && autoStep >= 3 && autoStep < 5) {
        Auto.LeftMotorDrive(0.25);
        Auto.RightMotorDrive(0.25);
        Auto.IntakeMotors(-0.5);
        Auto.Shooter(0.75);
        autoStep = 4;
        std::cout << "Drive forward" << std::endl;
      } else if (avgEncValue >= 43500 && gyro.GetAngle() > 42 && autoStep >= 3 && autoStep < 5) {
        Auto.RightMotorDrive(0.2);
        Auto.LeftMotorDrive(-0.2);
        Auto.IntakeMotors(0);
        Auto.Shooter(0.75);
        autoStep = 5;
        std::cout << "Turn again" << std::endl;
      } else if (avgEncValue >= 43500 && gyro.GetAngle() <= 42 && seconds < 7.5) {
        Auto.LeftMotorDrive(0);
        Auto.RightMotorDrive(0);
        Auto.IntakeMotors(-0.3);
        Auto.Shooter(0);
        Auto.Conveyor(-0.4);
      } else if (avgEncValue >= 43500 && avgEncValue < 96000 && gyro.GetAngle() <= 42 && seconds >= 7.5 && seconds < 11.5 && autoStep > 4 && autoStep < 6) {
        Auto.LeftMotorDrive(0.25);
        Auto.RightMotorDrive(0.25);
        Auto.IntakeMotors(-0.5);
        Auto.Conveyor(0);
        Auto.Shooter(0);
        std::cout << "Drive to terminal" << std::endl;
      } else if (autoStep >= 5 && avgEncValue >= 96000 && seconds > 11.5) {
        Auto.LeftMotorDrive(-0.25);
        Auto.RightMotorDrive(-0.25);
        Auto.IntakeMotors(-0.2);
        Auto.Shooter(0.75);
      } else if (autoStep >= 5 && avgEncValue <= 43500 && seconds > 11.5) {
        Auto.LeftMotorDrive(0);
        Auto.RightMotorDrive(0);
        Auto.Conveyor(-0.15);
        Auto.Shooter(0.8);
      }
    }
  }
}
void Robot::TeleopInit() {
  gyro.Reset();
  
  gyroResetted = false;
  FrontRightMotor.SetInverted(true);
  MiddleRightMotor.SetInverted(true);
  BackRightMotor.SetInverted(true);

  ShooterMotor2.SetInverted(true);
  LeftClimbMotor.SetSelectedSensorPosition(0);
  RightClimbMotor.SetSelectedSensorPosition(0);

  /*if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }*/

}

void Robot::TeleopPeriodic() {

  RobotContainer TeleOp;

  double JoyY = Wheel.GetX();
  double WheelX = -JoyStick1.GetY();

  frc::SmartDashboard::PutNumber("EncVal: ", LeftClimbMotor.GetSelectedSensorPosition());

  //Drive Code
  if ((WheelX > 0.05 || WheelX < -0.05) && (JoyY > 0.1 || JoyY < -0.1)) {
    TeleOp.LeftMotorDrive((JoyY*0.4) + (0.3*WheelX));
    TeleOp.RightMotorDrive((JoyY*0.4) - (0.3*WheelX));
  } else if (JoyY > 0.1 || JoyY < -0.1) {
    TeleOp.LeftMotorDrive((JoyY * 0.35));
    TeleOp.RightMotorDrive((JoyY * 0.35));
  } else if (Xbox.GetRawButton(3)) {
    if (!gyroResetted) {
      gyro.Reset();
      gyroResetted = true;
    } else if (gyro.GetAngle() < 15) {
      TeleOp.LeftMotorDrive(0.18);
      TeleOp.RightMotorDrive(-0.18);
    } else {
      TeleOp.LeftMotorDrive(0);
      TeleOp.RightMotorDrive(0);
    }
  } // Point turning
  else if (JoyStick1.GetRawButton(1)) {
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
    IntakePistons.Set(true);
    TeleOp.IntakeMotors(-0.45);
    TeleOp.Conveyor(-0.2);
    TeleOp.Shooter(0);
  } else if (Xbox.GetRawButton(1)) {
    IntakePistons.Set(true);
    TeleOp.IntakeMotors(0.45);
    TeleOp.Conveyor(0.5);
    TeleOp.Shooter(0);
  } else if (Xbox.GetRawButton(8) && !Xbox.GetRawButton(7)) {
    TeleOp.Conveyor(0.25);
    TeleOp.IntakeMotors(0);
    TeleOp.Shooter(0.75);
  } else if(Xbox.GetRawButton(2)) {
    TeleOp.Shooter(0.35);
    TeleOp.Conveyor(-0.5);
    TeleOp.IntakeMotors(0);
  } else if (Xbox.GetRawAxis(3) > 0) {
    if (!timerStarted) {
      timerStarted = true;
      shooterTimer->Reset();
      shooterTimer->Start();
    }
    
    if (shooterTimer->Get() < (units::time::second_t)0.5) {
      TeleOp.Shooter(0.95);
      TeleOp.Conveyor(0.1);
      TeleOp.IntakeMotors(0);
    } /*else if (shooterTimer->Get() > (units::time::second_t)2.45 && shooterTimer->Get() < (units::time::second_t)5) {
      TeleOp.Conveyor(0);
      TeleOp.Shooter(-1);
      TeleOp.IntakeMotors(0);
    } */else {
      TeleOp.Shooter(0.98);
      TeleOp.Conveyor(-0.6);
      TeleOp.IntakeMotors(0);
    }
  } else if(Xbox.GetPOV() == 270){
    IntakePistons.Set(true);
    TeleOp.IntakeMotors(-0.45);
    TeleOp.Shooter(0.2);
    TeleOp.Conveyor(-0.5);
  } else {
    TeleOp.IntakeMotors(0);
    TeleOp.Conveyor(0);
    TeleOp.Shooter(0);
    IntakePistons.Set(false);
    timerStarted = false;
  }

  //Climb Code (Left Button brings climber up, Right button brings climber down)
ClimbPiston.Set(false);
if (Xbox.GetRawButton(7) && Xbox.GetRawButton(8)) {
    ClimbPiston.Set(true); 
  }
if (Xbox.GetRawButton(5)) {
    TeleOp.Climb(0.48);
}else if(Xbox.GetRawButton(6)) {
    TeleOp.Climb(-0.35);
    //std::cout << "Accel Y: " << accelerometer.GetY() << std::endl;
    //std::cout << "Accel Z: " << accelerometer.GetZ() << std::endl;
}else if(Xbox.GetRawButton(7) && !Xbox.GetRawButton(8)) {
  if (LeftClimbMotor.GetSelectedSensorPosition() < 88806) {
      TeleOp.Climb(0.48);
    } else if (LeftClimbMotor.GetSelectedSensorPosition() >= 88806) {
      TeleOp.Climb(0);
    }
}else if (Xbox.GetRawButton(9)) {
  if (LeftClimbMotor.GetSelectedSensorPosition() < 158235) {
      TeleOp.Climb(0.48);
    } else {
      TeleOp.Climb(0);
    }
  } /*else if (Xbox.GetRawButtonPressed(8)) {
    //Button closest to xbox controller
    LeftClimbMotor.SetNeutralMode(Brake);
    RightClimbMotor.SetNeutralMode(Brake); 
  }*/
   else {
    TeleOp.Climb(0);
  }
  /*if (Xbox.GetRawButton(5)) {
    TeleOp.Climb(0.48); 
  } else if (Xbox.GetRawButton(6)) { 
    TeleOp.Climb(-0.35);
  }   if (LeftClimbMotor.GetSelectedSensorPosition() < 158235){             //(gyro.GetAngle() < 35){ 
        TeleOp.Climb(-0.35); 
  } 
  if (LeftClimbMotor.GetSelectedSensorPosition() < 158235) {
    TeleOp.Climb(0.48);
  }   else if (LeftClimbMotor.GetSelectedSensorPosition() < 158235){
        TeleOp.Climb(-0.35); 
  } else if (LeftClimbMotor.GetSelectedSensorPosition() < 158235) {
      ClimbPiston.Set(true);
  }*/
}

//void Robot::DisabledInit() {}

//void Robot::DisabledPeriodic() {}

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

//void Robot::TestPeriodic() {


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
  
//}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

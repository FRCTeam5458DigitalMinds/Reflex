// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// O_o hello is 

//Include header files
#include "Robot.h"
#include "RobotContainer.h"
#include "ScoreOne.h"
#include "Taxi.h"
#include "mainAuto.h"

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
TalonFX FrontRightMotor {15};
TalonFX MiddleRightMotor {14};
TalonFX BackRightMotor {13};

TalonFX FrontLeftMotor {1};
TalonFX MiddleLeftMotor {2};
TalonFX BackLeftMotor {3};

//Intake Motors
VictorSPX Intake {5};

//Shooter Motors
VictorSPX ConveyorMotor1 {4};
VictorSPX ConveyorMotor3 {7};

TalonFX ShooterMotor1 {10};

TalonFX ShooterMotor2 {9};
// Closest to intake

//Climb Motors
TalonFX LeftClimbMotor {7};
TalonFX RightClimbMotor {8};

//set amp limit
bool enable = true;
double currentLimit = 60;
double triggerThresholdCurrent = 5;
double triggerThresholdTime = 5;
double turnFact = 0.9;

//Pneumatics
// frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};
// frc::Solenoid IntakeBar{frc::PneumaticsModuleType::CTREPCM, 5};

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

int autoStep = 1;

bool timeStampChecked = true;
bool isShooterRunning = false;

steady_clock::time_point clock_begin;

//Set up motors to drive
void LeftMotorDrive (double speed) {
  FrontLeftMotor.Set(ControlMode::PercentOutput, speed);
  MiddleLeftMotor.Set(ControlMode::PercentOutput, speed);
  BackLeftMotor.Set(ControlMode::PercentOutput, speed);
}
void RightMotorDrive (double speed) { 
  FrontRightMotor.Set(ControlMode::PercentOutput, speed);
  MiddleRightMotor.Set(ControlMode::PercentOutput, speed);
  BackRightMotor.Set(ControlMode::PercentOutput, speed);
}
void IntakeMotors (double speed) {
  Intake.Set(ControlMode::PercentOutput, speed);
}
void Conveyor (double speed) {
  ConveyorMotor1.Set(ControlMode::PercentOutput, speed);
}
void Shooter (double speed) {
  ShooterMotor1.Set(ControlMode::PercentOutput, speed);
  ShooterMotor2.Set(ControlMode::PercentOutput, speed);
  ConveyorMotor3.Set(ControlMode::PercentOutput, speed);
}
void Climb (double speed) {
  LeftClimbMotor.Set(ControlMode::PercentOutput, speed);
  RightClimbMotor.Set(ControlMode::PercentOutput, speed);
}

//Auto Selector
frc::SendableChooser<frc2::Command*>m_chooser;

//Initializing robot & variables
void Robot::RobotInit() {
  //camera
  frc::CameraServer::GetInstance()->StartAutomaticCapture();

  frc::SmartDashboard::PutData(&m_chooser);
  // The chooser for the autonomous routines
  
  //frc::SmartDashboard::PutData("Auto Mode", &autoMode);
  //m_chooser.SetDefaultOption("mainAuto", autoMode::mainAuto);
  //frc::SmartDashboard::PutData("Play", &m_chooser);
  

  FrontRightMotor.SetInverted(true);
  MiddleRightMotor.SetInverted(true);
  BackRightMotor.SetInverted(true);

  ShooterMotor2.SetInverted(true);

  //Drop intake down at the beginning of a match
  // IntakeBar.Set(false);
  
  FrontLeftMotor.SetSelectedSensorPosition(0);
  MiddleLeftMotor.SetSelectedSensorPosition(0);
  BackLeftMotor.SetSelectedSensorPosition(0);

  FrontRightMotor.SetSelectedSensorPosition(0);
  MiddleRightMotor.SetSelectedSensorPosition(0);
  BackRightMotor.SetSelectedSensorPosition(0);

  LeftClimbMotor.SetSelectedSensorPosition(0);
  RightClimbMotor.SetSelectedSensorPosition(0);

  clock_begin = steady_clock::now();
  
  //amp change
  SupplyCurrentLimitConfiguration (enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
/*void Robot::RobotPeriodic() {
}*/

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
  //autoMode = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

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

  frc::CameraServer::GetInstance()->StartAutomaticCapture();

  autoStep = 1;

  gyro.Reset();

  m_autonomousCommand = m_container.GetAutonomousCommand();
  
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }

}


void Robot::AutonomousPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  //553.1248 cycles of the encoder per in. ---> Multiply by # of inches to find encoder units

  //Next steps: fixing drifting
  LeftDriveEncValue = FrontLeftMotor.GetSelectedSensorPosition();
  RightDriveEncValue = FrontRightMotor.GetSelectedSensorPosition();

  steady_clock::time_point clock_end = steady_clock::now();
  steady_clock::duration time_span = clock_end - clock_begin;

  double seconds = double(time_span.count()) * steady_clock::period::num / steady_clock::period::den;

  std::cout << "seconds: " << seconds << std::endl;
  /*distError = (setpoint - avgEncValue);
  turnError = (setpoint - gyro.GetAngle());

  turnPIDOutput = PVal * turnError;
  distPIDOutput = PVal * distError;

  if (turnPIDOutput > 0.25) {
    turnPIDOutput = 0.25;
  } else if (distPIDOutput > 0.25) {
    distPIDOutput = 0.25;
  }*/

  /*
  //Main Auto (Normal Distance - 42500)
  if (LeftDriveEncValue < 42500 && RightDriveEncValue < 42500 && (autoStep == 1)) {
      //std::cout << "Encoder Value: " << FrontLeftMotor.GetSelectedSensorPosition() << std::endl;
      setpoint = 6637;
      
      LeftMotorDrive(0.2);
      RightMotorDrive(0.2);
      Shooter(0); 

      Intake.Set(ControlMode::PercentOutput, -0.75);
      autoStep = 1;
  } else if (LeftDriveEncValue > 42500 && RightDriveEncValue > 42500) {
      autoStep = 2;
        
      LeftMotorDrive(-0.2);
      RightMotorDrive(-0.2);
      Conveyor(-0.1);
      Shooter(0);
      IntakeMotors(-0.5);
  } else if (LeftDriveEncValue <= -1000 && RightDriveEncValue <= -1000 && (autoStep >= 2)) {
      LeftMotorDrive(0);
      RightMotorDrive(0);
      Shooter(-0.35);
      Conveyor(-0.5);
      //figure out how many encoder rotations are equivalent to shooting two cargo and use that as qualification for next else if statement
      IntakeMotors(0);
      autoStep = 3;
      //std::cout << "Shooter Encoder Value: " << ShooterMotor1.GetSelectedSensorPosition() << std::endl;
  } //Part 2 of Main Auto (if we are on blue tarmac, turn 70 to left, meaning turn to -70 degrees)
   if (ShooterMotor1.GetSelectedSensorPosition() < -3000 && ShooterMotor2.GetSelectedSensorPosition() < -3000) {
        //Turn 70 degrees
        if (gyro.GetAngle() < 70) {
          RightMotorDrive(0.35);
          LeftMotorDrive(-0.35);
          autoStep = 4;
        } else if (LeftDriveEncValue < 165000 && RightDriveEncValue < 165000 && autoStep == 4) {
          LeftMotorDrive(0.25);
          RightMotorDrive(0.25);
          Intake.Set(ControlMode::PercentOutput, -0.75);
        } else {
          LeftMotorDrive(0);
          RightMotorDrive(0);
          Intake.Set(ControlMode::PercentOutput, 0);
        }
      }*/

  
  /*Auto - Shoot One + Taxi
  if (seconds < 13) {
    std::cout << "Less than 13 s" << std::endl;
    Shooter(-0.35);
    Conveyor(-0.35);
    LeftMotorDrive(0);
    RightMotorDrive(0);
  } else if (FrontLeftMotor.GetSelectedSensorPosition() < 70000) {
    std::cout << "Else is running (past 13 sec)" << std::endl;
    Shooter(0);
    Conveyor(0);
    LeftMotorDrive(0.4);
    RightMotorDrive(0.4);
  } else {
    Shooter(0);
    Conveyor(0);
    LeftMotorDrive(0);
    RightMotorDrive(0);
  }*/

    /*
    //Auto 2A - Shoot cargo then drive to the terminal and intake
    Shooter(0.35);
    if (gyro.GetAngle() < 31) {
      RightMotorDrive(-turnPIDOutput/5);
      LeftMotorDrive(turnPIDOutput/5);
    }
    if (avgEncValue < 137252.387872) {
      setpoint = 137252.387872;
      
      LeftMotorDrive(distPIDOutput/5);
      RightMotorDrive(distPIDOutput/5);
      Intake.Set(ControlMode::PercentOutput, 0.3);
    }

    //Auto 2B - Shoot, then turn and drive to terminal and intake
    Shooter(0.35);
    //Wait a couple seconds
    if (gyro.GetAngle() < 112.5) {
      RightMotorDrive(turnPIDOutput/5);
      LeftMotorDrive(-turnPIDOutput/5);
    }
    else if (avgEncValue < 222991) {
      LeftMotorDrive(distPIDOutput/5);
      RightMotorDrive(distPIDOutput/5);
      Intake.Set(ControlMode::PercentOutput, 0.3);
    } else {
      LeftMotorDrive(0);
      RightMotorDrive(0);
    }
  
    //Auto #3 (All Tarmacs)
    if(avgEncValue < 42131.516016) {
      LeftMotorDrive(0.25);
      RightMotorDrive(0.25);*/
}

void Robot::TeleopInit() {
  /*FrontRightMotor.SetInverted(true);
  MiddleRightMotor.SetInverted(true);
  BackRightMotor.SetInverted(true);*/

  ShooterMotor2.SetInverted(true);

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }

}

void Robot::TeleopPeriodic() {

double JoyY = -JoyStick1.GetY();
double WheelX = Wheel.GetX();

  if ((WheelX > 0.05 || WheelX < -0.05) && (JoyY > 0.1 || JoyY < -0.1)) {
    LeftMotorDrive((JoyY*0.2) + (0.5*WheelX));
    RightMotorDrive((JoyY*0.2) - (0.5*WheelX));
  } else if (JoyY > 0.1 || JoyY < -0.1) {
    LeftMotorDrive((JoyY * 0.8));
    RightMotorDrive((JoyY * 0.8));
  } // Point turning
    else if (Wheel.GetRawButton(5)) {
    if (WheelX > 0) {
      RightMotorDrive((WheelX * WheelX));
      LeftMotorDrive(-(WheelX * WheelX));
    } else if (WheelX < 0) {
      RightMotorDrive(-(WheelX * WheelX));
      LeftMotorDrive((WheelX * WheelX));
    }
  } else {
    LeftMotorDrive(0);
    RightMotorDrive(0);
  } 


  //Intake Code (button X to intake & run conveyor, button A to spit) & Shooter+Conveyor Code (button Y)
  if(Xbox.GetRawButton(3)) {
    IntakeMotors(-0.95);
    Conveyor(-0.1);
    Shooter(0);
  } else if (Xbox.GetRawButton(1)) {
    IntakeMotors(0.95);
    Conveyor(0.5);
    Shooter(0);
  } else if (Xbox.GetRawButton(2)) {
    Conveyor(-0.65);
    IntakeMotors(0);
    Shooter(0);
  } else if(Xbox.GetRawButton(4)) {
    Shooter(-0.35);
    Conveyor(-0.5);
    IntakeMotors(0);
  } else {
    IntakeMotors(0);
    Conveyor(0);
    Shooter(0);
  }
  

  //Climb Code (Left Button brings climber up, Right button brings climber down)
  if(Xbox.GetRawButton(5)) {
    Climb (0.38);
  } else if(Xbox.GetRawButton(6)) {
    Climb (-0.38);
  } else if (Xbox.GetRawButtonPressed(8)) {
    LeftClimbMotor.SetNeutralMode(Brake);
    RightClimbMotor.SetNeutralMode(Brake);
  } else {
    Climb (0);
  }

  //Brings intake bar down
  if(Xbox.GetRawButtonPressed(7)) {
    // IntakeBar.Set(!IntakeBar.Get());
  }

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

  
  //std::cout << "Gyro Angle: " << gyro.GetAngle() << std::endl;
  
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

// arc turning possible code
 // double Joystick1 = -JoyStick1.wpilib_joystick()->GetRawAxis(1);
 // double wheel = -Wheel_.wpilib_joystick()->GetRawAxis(0);
 // bool quickturn = quickturn_->is_pressed();

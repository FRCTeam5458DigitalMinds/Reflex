// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// O_o
#include "Robot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>
#include <sstream>
#include <iostream>
#include <ctre/Phoenix.h>
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
#include <frc/controller/PIDController.h>
#include <cmath>
#include <math.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include "cameraserver/CameraServer.h"
#include "networktables/NetworkTableEntry.inc"
#include "networktables/NetworkTableInstance.inc"
#include <chrono>
#include <ctime>
#include <ratio>

using namespace std::chrono;


//Declarations

//Joysticks
frc::Joystick JoyStick1(0), Wheel(2), Xbox(1);

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
VictorSPX ConveyorMotor2 {8};
TalonFX ShooterMotor1 {10};
TalonFX ShooterMotor2 {9};

//Climb Motors
TalonFX LeftClimbMotor {12};
TalonFX RightClimbMotor {11};

//Power Distribution Panel
//frc::PowerDistribution::PowerDistribution();

float turnFact = 0.9;

//Pneumatics
frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};
frc::Solenoid IntakeBar{frc::PneumaticsModuleType::CTREPCM, 5};
/*frc::PneumaticsControlModule PneumaticsControlModule();
PneumaticsControlModule.EnableCompressorAnalog(1, 10);
pcmCompressor.EnableAnalog(1, 10);
double current = pcmCompressor.GetCurrent();*/

//Gyro
WPI_PigeonIMU gyro{6};
//PigeonIMU gyro{6};

//PID (Proportional, Integral, Derivative) to calculate error and overshoot and correct it
//frc2::PIDController pid{0.1, 0, 0};
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

bool timeStampChecked = true;


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
void Conveyor (double speed) {
  ConveyorMotor1.Set(ControlMode::PercentOutput, speed);
  ConveyorMotor2.Set(ControlMode::PercentOutput, speed);
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


//Initializing robot & variables
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  FrontLeftMotor.SetInverted(true);
  MiddleLeftMotor.SetInverted(true);
  BackLeftMotor.SetInverted(true);

  ShooterMotor2.SetInverted(true);
  ConveyorMotor2.SetInverted(true);

  //Drop intake down at the beginning of a match
  IntakeBar.Set(true);
  
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

  //Limelight
  auto inst = nt::NetworkTableInstance::GetDefault();
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("<variablename>",0.0);
  double targetOffsetAngle_Horizontal = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
  double targetOffsetAngle_Vertical = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);
  double targetArea = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta",0.0);
  double targetSkew = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ts",0.0);

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
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
  /*FrontLeftMotor.SetSelectedSensorPosition(0);
  MiddleLeftMotor.SetSelectedSensorPosition(0);
  BackLeftMotor.SetSelectedSensorPosition(0);

  FrontRightMotor.SetSelectedSensorPosition(0);
  MiddleRightMotor.SetSelectedSensorPosition(0);
  BackRightMotor.SetSelectedSensorPosition(0);*/

  steady_clock::time_point clock_begin = steady_clock::now();
}

void Robot::AutonomousPeriodic() {
  //553.1248 cycles of the encoder per in. ---> Multiply by # of inches to find encoder units

  LeftDriveEncValue = FrontLeftMotor.GetSelectedSensorPosition();
  RightDriveEncValue = FrontRightMotor.GetSelectedSensorPosition();
  avgEncValue = (LeftDriveEncValue + RightDriveEncValue)/2;

  distError = (setpoint - avgEncValue);
  turnError = (setpoint - gyro.GetAngle());

  turnPIDOutput = PVal * turnError;
  distPIDOutput = PVal * distError;

  if (turnPIDOutput > 1) {
    turnPIDOutput = 1;
  } else if (distPIDOutput > 1) {
    distPIDOutput = 1;
  }

  if (m_autoSelected == kAutoNameCustom) {

    // Auto 1 - Same for all tarmacs
    if (avgEncValue < 42131.516016) {
      setpoint = 42131.516016;
      
      LeftMotorDrive(-distPIDOutput/5);
      RightMotorDrive(-distPIDOutput/5);
      Intake.Set(ControlMode::PercentOutput, -0.3);
    }
    else if (avgEncValue > 43000) {
      LeftMotorDrive(distPIDOutput/5);
      RightMotorDrive(distPIDOutput/5);
      Conveyor(0.5);
      Intake.Set(ControlMode::PercentOutput, 0);
    } else if (avgEncValue <= 5328) {
        LeftMotorDrive(0);
        RightMotorDrive(0);
        Shooter(0.25);
        Conveyor(0.5);
        Intake.Set(ControlMode::PercentOutput, 0);
      }
    }
    
    // Auto 2A - Shoot cargo then drive to the terminal and intake
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
      RightMotorDrive(0.25);
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  FrontLeftMotor.SetSelectedSensorPosition(0);
  MiddleLeftMotor.SetSelectedSensorPosition(0);
  BackLeftMotor.SetSelectedSensorPosition(0);

  FrontRightMotor.SetSelectedSensorPosition(0);
  MiddleRightMotor.SetSelectedSensorPosition(0);
  BackRightMotor.SetSelectedSensorPosition(0);
}

void Robot::TeleopPeriodic() {

LeftDriveEncValue = FrontLeftMotor.GetSelectedSensorPosition();
RightDriveEncValue = FrontRightMotor.GetSelectedSensorPosition();
avgEncValue = (LeftDriveEncValue + RightDriveEncValue)/2;

double JoyY = JoyStick1.GetY();
double WheelX = Wheel.GetX();

  if (WheelX > 0.1 && (JoyY > 0.1 || JoyY < -0.1)) {
    LeftMotorDrive((JoyY)/4);
    RightMotorDrive((JoyY)/2);
  } /*else if(Xbox.GetRawButton(8)) {
    if(LeftDriveEncValue > 6660) {
      LeftMotorDrive(-0.5);
      RightMotorDrive(-0.5);
    }
  }*/
  else if (WheelX < -0.1 && (JoyY > 0.1 || JoyY < -0.1)) {
    LeftMotorDrive((JoyY)/2);
    RightMotorDrive(JoyY/4);
  }
  //If joystick is pushed forward or backward
  else if (JoyY > 0.1 || JoyY < -0.1) {
    LeftMotorDrive(JoyY);
    RightMotorDrive(JoyY);
  } else {
    LeftMotorDrive(0);
    RightMotorDrive(0);
  }


  //Point turning (turning in place)
  if (Wheel.GetRawButton(5)) {
    if (WheelX > 0) {
      RightMotorDrive(-(WheelX * WheelX));
      LeftMotorDrive((WheelX * WheelX));
    } else if (WheelX < 0) {
      RightMotorDrive((WheelX * WheelX));
      LeftMotorDrive(-(WheelX * WheelX));
    }
  } 
  //Regular turning while driving
  else {
    LeftMotorDrive(0);
    RightMotorDrive(0);
  }


  //Intake Code (button # is subject to change)
  if(Xbox.GetRawButton(3)) {
    Intake.Set(ControlMode::PercentOutput, -0.85);
    Conveyor(-0.5);
    Shooter(0);
  } else if (Xbox.GetRawButton(1)) {
    Intake.Set(ControlMode::PercentOutput, 0.85);
    Conveyor(0);
    Shooter(0);
  } else if(Xbox.GetRawButton(4)) {
    Shooter(-0.35);
    Conveyor(-0.5);
    Intake.Set(ControlMode::PercentOutput, 0);
  } else {
    Intake.Set(ControlMode::PercentOutput, 0);
    Conveyor(0);
    Shooter(0);
  }
  

  //Climb Code 
  if(Xbox.GetRawButton(5)) {
    Climb (0.5);
  } else {
    Climb (0);
  }

  if(Xbox.GetRawButtonPressed(7)) {
    IntakeBar.Set(!IntakeBar.Get());
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

steady_clock::time_point clock_begin = steady_clock::now();

void Robot::TestPeriodic() {

  //Auto Testing
  LeftDriveEncValue = FrontLeftMotor.GetSelectedSensorPosition();
  RightDriveEncValue = FrontRightMotor.GetSelectedSensorPosition();
  avgEncValue = (LeftDriveEncValue + RightDriveEncValue)/2;

  distError = (setpoint - avgEncValue);
  turnError = (setpoint - gyro.GetAngle());

  turnPIDOutput = PVal * turnError;
  distPIDOutput = PVal * distError;

  if (turnPIDOutput > 1) {
    turnPIDOutput = 1;
  } else if (distPIDOutput > 1) {
    distPIDOutput = 1;
  }

  //Timer test for intake
  steady_clock::time_point clock_end = steady_clock::now();
  steady_clock::duration time_span = clock_end - clock_begin;

  double seconds = double(time_span.count()) * steady_clock::period::num / steady_clock::period::den;
  
  if (avgEncValue < 42131.516016) {
    setpoint = 42131.516016;
      
    LeftMotorDrive(distPIDOutput/5);
    RightMotorDrive(distPIDOutput/5);
    Intake.Set(ControlMode::PercentOutput, 0.25);
  }
  if (avgEncValue > 42131.516016) {
    LeftMotorDrive(-distPIDOutput/5);
    RightMotorDrive(-distPIDOutput/5);
    Intake.Set(ControlMode::PercentOutput, 0);
  } else if (avgEncValue <= 3328) {
    LeftMotorDrive(0);
    RightMotorDrive(0);
      if (seconds < 2 && timeStampChecked) {
        Shooter(0.25);
      }
    timeStampChecked = false;
  }


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


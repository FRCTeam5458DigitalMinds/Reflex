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
#include <frc/Solenoid.h>
#include <WPILibVersion.h>


//Declarations

//Joysticks
frc::Joystick JoyStick1(0), Wheel(2), Xbox(1);


//Drive Motors
TalonFX FrontLeftMotor {15};
TalonFX MiddleLeftMotor {14};
TalonFX BackLeftMotor {13};

TalonFX FrontRightMotor {1};
TalonFX MiddleRightMotor {2};
TalonFX BackRightMotor {3};


//Intake Motors
TalonFX LeftIntakeMotor {6};
TalonFX RightIntakeMotor {7};

//Shooter Motors
TalonFX LeftShooterMotor {12};
TalonFX RightShooterMotor {69};

//Climb Motors
TalonFX FirstClimbMotor {10};
TalonFX SecondClimbMotor {11};
TalonFX ThirdClimbMotor {13};

//Power Distribution Panel
//frc::PowerDistribution::PowerDistribution();

float turnFact = 0.9;

//Gyro
frc::ADXRS450_Gyro gyro;

//Variables
bool isStartingAngleSet;

//PID (Proportional, Integral, Derivative) to calculate error and overshoot and correct it
//double P, I, D, error, setpoint, rcw;

//Set up motors to drive
void LeftMotorDrive (double speed) {
  //negative speed because left motors are reversed
  FrontLeftMotor.Set(ControlMode::PercentOutput, speed);
  MiddleLeftMotor.Set(ControlMode::PercentOutput, speed);
  BackLeftMotor.Set(ControlMode::PercentOutput, speed);
}
void RightMotorDrive (double speed) { 
  FrontRightMotor.Set(ControlMode::PercentOutput, speed);
  MiddleRightMotor.Set(ControlMode::PercentOutput, speed);
  BackRightMotor.Set(ControlMode::PercentOutput, speed);
}
void Intake (double speed) {
  LeftIntakeMotor.Set(ControlMode::PercentOutput, speed);
  RightIntakeMotor.Set(ControlMode::PercentOutput, speed);
}
void Shooter (double speed) {
  LeftShooterMotor.Set(ControlMode::PercentOutput, speed);
  RightShooterMotor.Set(ControlMode::PercentOutput, speed);
}
void Climb (double speed) {
  FirstClimbMotor.Set(ControlMode::PercentOutput, speed);
  SecondClimbMotor.Set(ControlMode::PercentOutput, speed);
  ThirdClimbMotor.Set(ControlMode::PercentOutput, speed);
}
/*void setSetpoint(int setpoint) {
  this.setpoint = setpoint;
}
void PID() {
  error = setpoint - gyro.getAngle() // Error = Target - Actual
  this.integral += (error*.02) // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
  derivative = (error - this.previous_error) / .02
  rcw = P*error + I*self.integral + D*derivative
}*/

//Initializing robot & variables
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  FrontLeftMotor.SetInverted(true);
  MiddleLeftMotor.SetInverted(true);
  BackLeftMotor.SetInverted(true);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

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
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

double JoyY = -JoyStick1.GetY();
double WheelX = Wheel.GetX();

  if (WheelX > 0.05 && (JoyY > 0.05 || JoyY < -0.05)) {
    LeftMotorDrive((turnFact * WheelX));
    RightMotorDrive(-(turnFact * WheelX));
  }
  else if (WheelX < -0.05 && (JoyY > 0.05 || JoyY < -0.05)) {
    LeftMotorDrive(-(turnFact * WheelX));
    RightMotorDrive((turnFact * WheelX));
  }
  //If joystick is pushed forward or backward
  else if (JoyY > 0.1 || JoyY < -0.1) {
    LeftMotorDrive(JoyY/2);
    RightMotorDrive(JoyY/2);
  }
  //Point turning (turning in place)
  else if (Wheel.GetRawButton(5)) {
    if (WheelX > 0) {
      LeftMotorDrive(WheelX * WheelX);
      RightMotorDrive(-(WheelX * WheelX));
    } else if (WheelX < 0) {
      LeftMotorDrive(-(WheelX * WheelX));
      RightMotorDrive(WheelX * WheelX);
    }
  } 
  //Regular turning while driving
  else {
    LeftMotorDrive(0);
    RightMotorDrive(0);
  }


  /*Intake Code (button # is subject to change)
  if(Xbox.GetRawButton(1)) {
    Intake(0.2);
  }

  //Shooter Code
  if(Xbox.GetRawButton(2)) {
    Shooter(0.2);
  }

  //Climb Code
  if(Xbox.GetRawButton(3)) {
    Climb (0.2);
  }*/

  frc::SmartDashboard::PutNumber("JoyStick Value", JoyY);
  frc::SmartDashboard::PutNumber("LeftMotorValue", FrontLeftMotor.GetMotorOutputPercent());
  frc::SmartDashboard::PutNumber("RightMotorValue", FrontRightMotor.GetMotorOutputPercent());
  frc::SmartDashboard::PutNumber("TurnValue", WheelX);
} 

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

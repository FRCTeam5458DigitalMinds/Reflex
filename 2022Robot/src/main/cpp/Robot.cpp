
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
#include <frc/CompressorConfigType.h>
#include <WPILibVersion.h>
#include <frc/controller/PIDController.h>
#include <string.h>
#include <cmath>
#include <math.h>

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
VictorSPX IntakeMotor {8};


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

//Pneumatics
frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};
frc::Solenoid solenoid1{frc::PneumaticsModuleType::CTREPCM, 1};
frc::Solenoid solenoid2{frc::PneumaticsModuleType::CTREPCM, 2};
//frc::Solenoid

//Gyro
WPI_PigeonIMU gyro{6};
//PigeonIMU gyro{6};
double gyroAngle = gyro.GetAngle();

//PID (Proportional, Integral, Derivative) to calculate error and overshoot and correct it
//frc2::PIDController pid{0.1, 0, 0};
double PVal = 0.1;
double setpoint;
double PIDOutput;
double turnError;


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
void Intake (double speed) {
  IntakeMotor.Set(ControlMode::PercentOutput, speed);
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


//Initializing robot & variables
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

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
  /*FrontLeftMotor.SetSelectedSensorPosition(0);
  MiddleLeftMotor.SetSelectedSensorPosition(0);
  BackLeftMotor.SetSelectedSensorPosition(0);

  FrontRightMotor.SetSelectedSensorPosition(0);
  MiddleRightMotor.SetSelectedSensorPosition(0);
  BackRightMotor.SetSelectedSensorPosition(0);*/
}

void Robot::AutonomousPeriodic() {
  //553.1248 cycles of the encoder per in. ---> Multiply by # of inches to find encoder units

  double LeftDriveEncValue = FrontLeftMotor.GetSelectedSensorPosition();
  double RightDriveEncValue = FrontRightMotor.GetSelectedSensorPosition();
  double avgEncValue = (LeftDriveEncValue + RightDriveEncValue)/2;

  if (m_autoSelected == kAutoNameCustom) {
    /*if (avgEncValue < 12732.365) {
      frc::SmartDashboard::PutNumber("Average Encoder Value", avgEncValue);
      setpoint = 12732.365;
      LeftMotorDrive(pid.Calculate(avgEncValue, setpoint));
      RightMotorDrive(pid.Calculate(avgEncValue, setpoint));
    }
    else {
      LeftMotorDrive(0);
      RightMotorDrive(0);
    }
  }*/

    /*
    // Auto 1 - Same for all tarmacs
    // Shoot cargo (code for that will be written here)
    if (avgEncValue < 42,131.516016) {
      LeftMotorDrive(0.2);
      RightMotorDrive(0.2);
      Intake(0.2);
      
    }
    if(avgEncValue > 0) {
      LeftMotorDrive(-0.2);
      RightMotorDrive(-0.2);
    }
    //Shooter Code Goes Here...
    
    // Auto #2 - Blue Bottom & Red Top (locations near terminal)
    // Scoring code will go here
    if (avgEncValue < 137,252.387872) {
      LeftMotorDrive (0.2);
      RightMotorDrive (0.2);
    }

     //Auto #2 - Blue Top & Red Bottom (Locations closest to hangar)
     Scoring code will go here
     //if (GyroSensor < 112.5)
      RightMotorDrive(0.2);
      LeftMotorDrive(-0.2);
     else {
       LeftMotorDrive(0);
       RightMotorDrive(0);
     }
     
    if (avgEncValue < 222,991) {
      LeftMotorDrive (0.2);
      RightMotorDrive (0.2);
    }
  
    //Auto #3 (All Tarmacs)
    if(avgEncValue < 42,131.516016) {
      LeftMotorDrive (0.2);
      RightMotorDrive (0.2);


    
  } else {
    // Default Auto goes here
  }*/
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

double JoyY = -JoyStick1.GetY();
double WheelX = Wheel.GetX();

  if (WheelX > 0.1 && (JoyY > 0.05 || JoyY < -0.05)) {
    LeftMotorDrive((JoyY)/2);
    RightMotorDrive((JoyY)/4);
  }
  else if (WheelX < -0.1 && (JoyY > 0.05 || JoyY < -0.05)) {
    LeftMotorDrive((JoyY)/4);
    RightMotorDrive(JoyY/2);
  }
  //If joystick is pushed forward or backward
  else if (JoyY > 0.1 || JoyY < -0.1) {
    LeftMotorDrive(JoyY/2);
    RightMotorDrive(JoyY/2);
  }
  //Point turning (turning in place)
  else if (Wheel.GetRawButton(5)) {
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
    Intake(0.2);
  }

  //Shooter Code
  if(Xbox.GetRawButton(4)) {
    Shooter(0.2);
  }

  //Climb Code 
  if(Xbox.GetRawButton(5)) {
    Climb (0.2);
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

  setpoint = 0;
  gyro.Reset();
  gyro.Calibrate();
}

void Robot::TestPeriodic() {
  //Find the distance from goal angle
  turnError = (setpoint - gyro.GetAngle());
  //Multiply by constant PVal to get motor speed; this will decrease as the robot gets closer to the goal angle
  PIDOutput = PVal * turnError;

  if (PIDOutput > 1) {
    PIDOutput = 1;
  }
  
  //Determine tolerance of gyro
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
  }
  
  std::cout << "Gyro Angle: " << gyro.GetAngle() << std::endl;
  

  /*if ((LeftDriveEncValue + RightDriveEncValue)/2 < 12732.365) {
      frc::SmartDashboard::PutNumber("Average Encoder Value", avgEncValue);
      setpoint = 12732.365;
      //LeftMotorDrive(pid.Calculate(avgEncValue, setpoint));
      //RightMotorDrive(pid.Calculate(avgEncValue, setpoint));
      LeftMotorDrive(0.1);
      RightMotorDrive(0.1);
  }
  else {
      LeftMotorDrive(0);
      RightMotorDrive(0);
  }*/

  /*Command to get angle from gyro is gyro.GetAngle()

  if(FrontLeftMotor.GetSelectedSensorPosition() < 75000 && FrontRightMotor.GetSelectedSensorPosition() < 75000) {
      if (gyroAngle > 10) {
        LeftMotorDrive(0.1/2);
        RightMotorDrive(0.1);
      }
      else if (gyroAngle < -10) {
        LeftMotorDrive(0.1);
        RightMotorDrive(0.1/2);
      }
      else {
        LeftMotorDrive(0.1);
        RightMotorDrive(0.1);
      }
  }
  else {
    LeftMotorDrive(0);
    RightMotorDrive(0);
  }*/
  
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

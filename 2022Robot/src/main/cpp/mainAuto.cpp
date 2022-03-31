
#include "mainAuto.h"

#include <frc/shuffleboard/Shuffleboard.h>
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
#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>
#include <frc2/command/ConditionalCommand.h>

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

//Gyro
WPI_PigeonIMU gyro{6};

int autoStep = 1;

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


void mainAuto::mainMainAuto() {
  double LeftDriveEncValue = FrontLeftMotor.GetSelectedSensorPosition();
  double RightDriveEncValue = FrontRightMotor.GetSelectedSensorPosition();
  
  //mainAuto
  if (LeftDriveEncValue < 42500 && RightDriveEncValue < 42500 && (autoStep == 1)) {
      //std::cout << "Encoder Value: " << FrontLeftMotor.GetSelectedSensorPosition() << std::endl;
      
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
   }
}

int main() {
  mainAuto testMainAuto;     // Create an object of MyClass
  testMainAuto.mainMainAuto();

  return 0;
}
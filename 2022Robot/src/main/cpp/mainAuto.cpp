
#include "mainAuto.h"
#include "RobotContainer.h"

/*
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


void mainAuto::mainMainAuto() {
  RobotContainer AutoMain;

  double LeftDriveEncValue = FrontLeftMotor.GetSelectedSensorPosition();
  double RightDriveEncValue = FrontRightMotor.GetSelectedSensorPosition();
  
  //mainAuto
  if (LeftDriveEncValue < 42500 && RightDriveEncValue < 42500 && (autoStep == 1)) {
      //std::cout << "Encoder Value: " << FrontLeftMotor.GetSelectedSensorPosition() << std::endl;
      
      AutoMain.LeftMotorDrive(0.2);
      AutoMain.RightMotorDrive(0.2);
      AutoMain.Shooter(0); 

      Intake.Set(ControlMode::PercentOutput, -0.75);
      autoStep = 1;
  } else if (LeftDriveEncValue > 42500 && RightDriveEncValue > 42500) {
      autoStep = 2;
        
      AutoMain.LeftMotorDrive(-0.2);
      AutoMain.RightMotorDrive(-0.2);
      AutoMain.Conveyor(-0.1);
      AutoMain.Shooter(0);
      AutoMain.IntakeMotors(-0.5);
  } else if (LeftDriveEncValue <= -1000 && RightDriveEncValue <= -1000 && (autoStep >= 2)) {
      AutoMain.LeftMotorDrive(0);
      AutoMain.RightMotorDrive(0);
      AutoMain.Shooter(-0.35);
      AutoMain.Conveyor(-0.5);
      //figure out how many encoder rotations are equivalent to shooting two cargo and use that as qualification for next else if statement
      AutoMain.IntakeMotors(0);
      autoStep = 3;
      //std::cout << "Shooter Encoder Value: " << ShooterMotor1.GetSelectedSensorPosition() << std::endl;
  } //Part 2 of Main Auto (if we are on blue tarmac, turn 70 to left, meaning turn to -70 degrees)
   if (ShooterMotor1.GetSelectedSensorPosition() < -3000 && ShooterMotor2.GetSelectedSensorPosition() < -3000) {
        //Turn 70 degrees
        if (gyro.GetAngle() < 70) {
          AutoMain.RightMotorDrive(0.35);
          AutoMain.LeftMotorDrive(-0.35);
          autoStep = 4;
        } else if (LeftDriveEncValue < 165000 && RightDriveEncValue < 165000 && autoStep == 4) {
          AutoMain.LeftMotorDrive(0.25);
          AutoMain.RightMotorDrive(0.25);
          Intake.Set(ControlMode::PercentOutput, -0.75);
        } else {
          AutoMain.LeftMotorDrive(0);
          AutoMain.RightMotorDrive(0);
          Intake.Set(ControlMode::PercentOutput, 0);
        }
   }
}

int main() {
  mainAuto testMainAuto;     // Create an object of MyClass
  testMainAuto.mainMainAuto();

  return 0;
}*/
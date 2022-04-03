#include "Taxi.h"
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


//steady_clock::time_point clock_begin;


void Taxi::taxiAuto() {
  RobotContainer AutoTaxi;
  //Taxi Auto
  if(FrontLeftMotor.GetSelectedSensorPosition() < 42131.516016) {
      AutoTaxi.LeftMotorDrive(0.25);
      AutoTaxi.RightMotorDrive(0.25);
  } else {
    AutoTaxi.LeftMotorDrive(0);
    AutoTaxi.RightMotorDrive(0);
  }
}

int main() {
  Taxi testTaxiAuto;     // Create an object of MyClass
  testTaxiAuto.taxiAuto();

  return 0;
}*/
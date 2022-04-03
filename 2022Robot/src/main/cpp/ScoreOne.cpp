#include "ScoreOne.h"
#include "RobotContainer.h"

using namespace std::chrono;

/*
int autoStep = 1;


steady_clock::time_point clock_begin;


void ScoreOne::ScoreOneAuto() {
  RobotContainer AutoScore;

  //ScoreOne Auto
  steady_clock::time_point clock_end = steady_clock::now();
  steady_clock::duration time_span = clock_end - clock_begin;

  AutoScore.FrontLeft(double LeftEncoderValue);
  double seconds = double(time_span.count()) * steady_clock::period::num / steady_clock::period::den;
  //Auto - Shoot One + Taxi
  if (seconds < 13) {
    std::cout << "Less than 13 s" << std::endl;
    AutoScore.Shooter(-0.35);
    AutoScore.Conveyor(-0.35);
    AutoScore.LeftMotorDrive(0);
    AutoScore.RightMotorDrive(0);
  } else if (LeftEncoderValue < 70000) {
    std::cout << "Else is running (past 13 sec)" << std::endl;
    AutoScore.Shooter(0);
    AutoScore.Conveyor(0);
    AutoScore.LeftMotorDrive(0.4);
    AutoScore.RightMotorDrive(0.4);
  } else {
    AutoScore.Shooter(0);
    AutoScore.Conveyor(0);
    AutoScore.LeftMotorDrive(0);
    AutoScore.RightMotorDrive(0);
  }
}

int main() {
  ScoreOne testAuto;     // Create an object of MyClass
  testAuto.ScoreOneAuto();

  return 0;
}*/


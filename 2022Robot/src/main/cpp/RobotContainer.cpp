// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include "Robot.h"
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

#include "frc/smartdashboard/Smartdashboard.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/ConditionalCommand.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/button/JoystickButton.h>


void RobotContainer::testRobotContainer() {
  // Initialize all of your commands and subsystems here

  // Add commands to the autonomous command chooser
  m_chooser.SetDefaultOption("Main Auto", &m_MainAuto);
  m_chooser.AddOption("Taxi Auto", &m_TaxiAuto);
  m_chooser.AddOption("Shoot One Auto", &m_ScoreOneAuto);

  // Put the chooser on the dashboard
  frc::Shuffleboard::GetTab("Autonomous").Add(m_chooser);

}

int main() {
  RobotContainer test;
  test.testRobotContainer();

  return 0;
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Runs the chosen command in autonomous
  return m_chooser.GetSelected();
}
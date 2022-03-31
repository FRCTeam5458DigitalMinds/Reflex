#pragma once

//Include header files
#include "Robot.h"
#include "mainAuto.h"
#include "Taxi.h"
#include "ScoreOne.h"
#include <frc/smartdashboard/SendableChooser.h>

#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Command.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer;

class RobotContainer {
 public:
  void testRobotContainer();

  frc2::Command* GetAutonomousCommand();

  private:
  // The robot's subsystems and commands are defined here...

  mainAuto m_MainAuto;
  Taxi m_TaxiAuto;
  ScoreOne m_ScoreOneAuto;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

};
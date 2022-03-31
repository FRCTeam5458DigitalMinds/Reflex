// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

/**
 * Our main auto command that drives forward, grabs a second cargo, and then drives
 * backward and shoots two cargo.
 */
class mainAuto;

class mainAuto 
 : public frc2::CommandHelper<frc2::SequentialCommandGroup, mainAuto> {
 
 public:
  /**
   * Creates a new ComplexAuto.
   *
   **/
  void mainMainAuto();
};
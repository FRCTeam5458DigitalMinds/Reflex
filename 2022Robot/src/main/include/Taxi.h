#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>


/**
 * Our main auto command that shoots one cargo and drives forward to taxi.
 * 
 */
class Taxi;

class Taxi
 : public frc2::CommandHelper<frc2::SequentialCommandGroup, Taxi> {
 public:
  /**
   * Creates a new Taxi Auto.
   *
   **/
  void taxiAuto();
};


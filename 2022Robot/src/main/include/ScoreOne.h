
#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

/**
 * Our main auto command that shoots one cargo and drives forward to taxi.
 * 
 */
class ScoreOne;

class ScoreOne  
 : public frc2::CommandHelper<frc2::SequentialCommandGroup, ScoreOne> {
 public:
  
  /**
   * Creates a new ScoreOne Auto.
   *
   **/
 void ScoreOneAuto();
};
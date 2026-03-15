package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
  private Climber climber;

  public enum ClimbStage {
    STAGE_1, STAGE_2
  }

  private ClimbStage climbStage;
  private int levelsToClimb = 3;
  private int numLevelsClimbed;

  /** Create a climb command for L3 */
  public Climb(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  /** Create a climb command for a specific number of levels */
  public Climb(Climber climber, int levelsToClimb) {
    this(climber);
    this.levelsToClimb = levelsToClimb;
  }

  private void handleClimbStage1() {
    if (climber.primaryAtUpperLimit()) {
      climbStage = ClimbStage.STAGE_2;
    } else {
      climber.setPosition(climber.ENCODER_POSITION_TOP);
    }
  }

  private void handleClimbStage2() {
    if (climber.primaryAtLowerLimit()) {
      climbStage = ClimbStage.STAGE_1;
      numLevelsClimbed++;
    } else {
      climber.setPosition(climber.ENCODER_POSITION_BOTTOM);
    }
  }

  @Override
  public void initialize() {
    climbStage = ClimbStage.STAGE_1;
    numLevelsClimbed = 0;
  }

  @Override
  public void execute() {
    switch (climbStage) {
      case STAGE_1 :
        handleClimbStage1();
        break;
      case STAGE_2 :
        handleClimbStage2();
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return numLevelsClimbed == levelsToClimb;
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  public ClimbStage getClimbStage() {
    return climbStage;
  }

  public int getNumLevelsClimbed() {
    return numLevelsClimbed;
  }
}

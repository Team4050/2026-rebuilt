package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Homeable;

/**
 * Reusable stall-homing command. Drives a {@link Homeable} subsystem toward its home position until a current spike and
 * low velocity are sustained, indicating a mechanical hard stop.
 *
 * <p>
 * On successful completion, {@link Homeable#onHomeComplete()} is called to reset the encoder. If interrupted (e.g. by
 * another command), the motor is stopped but the encoder is not reset.
 */
public class Home extends Command {
  private final Homeable subsystem;
  private final double stallCurrentAmps;
  private final double stallVelocityRpm;
  private final double stallTimeSec;

  private final Timer stallTimer = new Timer();

  /**
   * Creates a new HomeCommand.
   *
   * @param subsystem
   *          The subsystem to home. Will be added as a requirement.
   * @param stallCurrentAmps
   *          Current threshold (amps) above which the motor is considered stalled.
   * @param stallVelocityRpm
   *          Velocity threshold (RPM) below which the motor is considered stalled.
   * @param stallTimeSec
   *          Duration (seconds) the stall condition must be sustained before homing is complete.
   */
  public Home(Homeable subsystem, double stallCurrentAmps, double stallVelocityRpm, double stallTimeSec) {
    this.subsystem = subsystem;
    this.stallCurrentAmps = stallCurrentAmps;
    this.stallVelocityRpm = stallVelocityRpm;
    this.stallTimeSec = stallTimeSec;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    stallTimer.stop();
    stallTimer.reset();
  }

  @Override
  public void execute() {
    subsystem.driveToHome();

    boolean stalled = subsystem.getHomingCurrent() > stallCurrentAmps
        && subsystem.getHomingVelocity() < stallVelocityRpm;

    if (stalled) {
      if (!stallTimer.isRunning()) {
        stallTimer.start();
      }
    } else {
      stallTimer.stop();
      stallTimer.reset();
    }
  }

  @Override
  public boolean isFinished() {
    return stallTimer.hasElapsed(stallTimeSec);
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.stopHoming();
    if (!interrupted) {
      subsystem.onHomeComplete();
    }
  }
}

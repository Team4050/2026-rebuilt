package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Interface for subsystems that support stall-based homing. Implementing this interface allows the subsystem to be used
 * with {@link frc.robot.commands.Home}.
 */
public interface Homeable extends Subsystem {
  /** Drive the mechanism toward the home position at homing speed. */
  void driveToHome();

  /** Stop the homing motor. */
  void stopHoming();

  /** Get the current draw of the homing motor in amps. */
  double getHomingCurrent();

  /** Get the absolute velocity of the homing motor in RPM. */
  double getHomingVelocity();

  /** Called when homing completes successfully to reset the encoder position. */
  void onHomeComplete();
}

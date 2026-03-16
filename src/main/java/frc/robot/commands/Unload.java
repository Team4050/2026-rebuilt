package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Unloader;

public class Unload {
  private final Unloader unloaderLeft;
  private final Unloader unloaderRight;

  private boolean shooterMotorActive = false;

  public Unload(Unloader unloaderLeft, Unloader unloaderRight) {
    this.unloaderLeft = unloaderLeft;
    this.unloaderRight = unloaderRight;
  }

  private void primeShooter() {
    unloaderLeft.primeShooter();
    unloaderRight.primeShooter();

    shooterMotorActive = true;
  }

  /**
   * Runs the shooter. Only active if the unloader has a shooter. If the unloader doesn't have a shooter, runs the
   * kicker in reverse to eject the game piece.
   */
  private void shoot() {
    if (!unloaderLeft.hasShooter() && !unloaderRight.hasShooter()) {
      return;
    }

    if (unloaderLeft.hasShooter()) {
      unloaderLeft.shoot();
    } else {
      unloaderLeft.reverse();
    }

    if (unloaderRight.hasShooter()) {
      unloaderRight.shoot();
    } else {
      unloaderRight.reverse();
    }
  }

  /**
   * Runs the outtake. Only active if the unloader doesn't have a shooter. If the unloader has a shooter, runs the
   * kicker in reverse to eject the game piece.
   */
  private void outtake() {
    if (unloaderLeft.hasShooter() && unloaderRight.hasShooter()) {
      return;
    }

    if (!unloaderLeft.hasShooter()) {
      unloaderLeft.runOuttake();
    } else {
      unloaderLeft.reverse();
    }

    if (!unloaderRight.hasShooter()) {
      unloaderRight.runOuttake();
    } else {
      unloaderRight.reverse();
    }
  }

  public Command primeCommand() {
    // this command omits dependencies to avoid the command being canceled (shooter motors are independent)
    return new RunCommand(this::primeShooter).finallyDo(() -> {
      unloaderLeft.stopShooter();
      unloaderRight.stopShooter();
      shooterMotorActive = false;
    }).withName("Unloaders: Prime Shooter(s)");
  }

  public Command shootCommand() {
    return new RunCommand(this::shoot, unloaderLeft, unloaderRight).finallyDo(() -> {
      unloaderLeft.stopKicker();
      unloaderRight.stopKicker();
    }).withName("Unloaders: Shoot");
  }

  public Command outtakeCommand() {
    return new RunCommand(this::outtake, unloaderLeft, unloaderRight).finallyDo(() -> {
      unloaderLeft.stopKicker();
      unloaderRight.stopKicker();
    }).withName("Unloaders: Run Outtake");
  }

  public boolean shooterIsActive() {
    return shooterMotorActive;
  }
}

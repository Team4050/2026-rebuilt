package frc.robot.util;

import frc.robot.Constants;

public record Preset(String name, boolean drivetrainEnabled, boolean climberEnabled, boolean intakeEnabled,
    boolean outtakeEnabled, double mainSpeed, double secondarySpeed, double rotationalRate, double shooterSpeed) {

  public static Preset noPreset() {
    return new Preset("Custom Settings", false, false, false, false, 0, 0, 0, 0);
  }

  public static Preset competitive() {
    return new Preset("Competitive", Constants.Drivetrain.DEFAULT_ENABLED, Constants.Drivetrain.DEFAULT_ENABLED,
        Constants.Intake.DEFAULT_ENABLED, true, Constants.Drivetrain.DEFAULT_MAIN_SPEED,
        Constants.Drivetrain.DEFAULT_SECONDARY_SPEED, Constants.Drivetrain.DEFAULT_ROTATIONAL_RATE,
        Constants.Unloader.DEFAULT_SHOOTER_SPEED);
  }

  public static Preset noDrivetrain() {
    return new Preset("No Drivetrain", false, true, true, true, 0.50, 0.15, 0.5, 0.65);
  }

  public static Preset onlyDrivetrain() {
    return new Preset("Drivetrain Only", true, false, false, false, 0.30, 0.10, 0.5, 0.65);
  }

  public static Preset safeMode() {
    return new Preset("Safe Mode", true, false, true, true, 0.20, 0.10, 0.5, 0.30);
  }
}

package frc.robot.util;

public record Preset(boolean drivetrainEnabled, boolean climberEnabled, boolean intakeEnabled, boolean outtakeEnabled,
    double mainSpeed, double secondarySpeed, double rotationalRate, double shooterSpeed) {

  public static Preset competitive() {
    return new Preset(true, true, true, true, 0.50, 0.15, 0.5, 0.65);
  }

  // TODO add other presets
}

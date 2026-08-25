package frc.robot.util;

public record Preset(String name, boolean drivetrainEnabled, boolean climberEnabled, boolean intakeEnabled,
    boolean outtakeEnabled, double mainSpeed, double secondarySpeed, double rotationalRate, double shooterSpeed) {

  public static Preset noPreset() {
    return new Preset("Custom Settings", false, false, false, false, 0, 0, 0, 0);
  }

  public static Preset competitive() {
    return new Preset("Competitive", true, true, true, true, 0.50, 0.15, 0.5, 0.65);
  }

  // TODO add other presets
}

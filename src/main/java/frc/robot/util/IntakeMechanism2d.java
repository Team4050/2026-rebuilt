package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** A Mechanism2d helper that visualizes the intake deploy arm and rollers. */
public final class IntakeMechanism2d extends Mechanism2d {
  private static final double CANVAS_WIDTH = 40;
  private static final double CANVAS_HEIGHT = 24;
  private static final double ARM_LENGTH = 16;
  private static final double ROLLER_BASE_LENGTH = 2;
  private static final double ROLLER_LENGTH_VARIATION = 3.5;
  private static final double ARM_LINE_WEIGHT = 6;
  private static final double ROLLER_LINE_WEIGHT = 5;
  private static final double BASE_LINE_WEIGHT = 4;
  private static final double ROLLER_IDLE_THRESHOLD = 0.08;

  private static final Color8Bit BACKGROUND_COLOR = new Color8Bit(15, 20, 30);
  private static final Color8Bit ARM_COLOR = new Color8Bit(235, 137, 52);
  private static final Color8Bit BASE_COLOR = new Color8Bit(100, 120, 140);
  private static final Color8Bit ROLLER_IDLE_COLOR = new Color8Bit(192, 192, 192);
  private static final Color8Bit ROLLER_IN_COLOR = new Color8Bit(0, 190, 120);
  private static final Color8Bit ROLLER_OUT_COLOR = new Color8Bit(0, 160, 255);

  private final MechanismLigament2d intakeArm;
  private final MechanismLigament2d rollerIndicator;

  public IntakeMechanism2d() {
    super(CANVAS_WIDTH, CANVAS_HEIGHT);
    setBackgroundColor(BACKGROUND_COLOR);

    MechanismRoot2d pivot = getRoot("Intake Pivot", CANVAS_WIDTH * 0.25, CANVAS_HEIGHT / 2.0);
    var mount = pivot.append(new MechanismLigament2d("Intake Mount", 5, 0));
    mount.setLineWeight(BASE_LINE_WEIGHT);
    mount.setColor(BASE_COLOR);

    intakeArm = pivot.append(new MechanismLigament2d("Intake Arm", ARM_LENGTH, 180));
    intakeArm.setLineWeight(ARM_LINE_WEIGHT);
    intakeArm.setColor(ARM_COLOR);

    rollerIndicator = intakeArm.append(new MechanismLigament2d("Roller Indicator", ROLLER_BASE_LENGTH, 0));
    rollerIndicator.setLineWeight(ROLLER_LINE_WEIGHT);
    rollerIndicator.setColor(ROLLER_IDLE_COLOR);
  }

  /**
   * Aligns the visual arm with the reported deploy angle from the intake subsystem.
   *
   * @param degrees the reported position (degrees), typically the encoder position.
   */
  public void setDeployAngleDegrees(double degrees) {
    intakeArm.setAngle(normalizeDegrees(degrees));
  }

  /**
   * Highlights roller activity (in/out/idle) with length + color cues.
   *
   * @param speed negative for out, positive for in, zero to stop.
   */
  public void setRollerSpeed(double speed) {
    double clamped = clamp(speed, -1, 1);
    rollerIndicator.setLength(ROLLER_BASE_LENGTH + Math.abs(clamped) * ROLLER_LENGTH_VARIATION);
    if (Math.abs(clamped) < ROLLER_IDLE_THRESHOLD) {
      rollerIndicator.setColor(ROLLER_IDLE_COLOR);
    } else if (clamped > 0) {
      rollerIndicator.setColor(ROLLER_IN_COLOR);
    } else {
      rollerIndicator.setColor(ROLLER_OUT_COLOR);
    }
    rollerIndicator.setAngle(clamped >= 0 ? 0 : 180);
  }

  private static double normalizeDegrees(double degrees) {
    double normalized = degrees % 360;
    if (normalized < 0) {
      normalized += 360;
    }
    return normalized;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}

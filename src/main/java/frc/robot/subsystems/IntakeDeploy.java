package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeDeploy extends SubsystemBase {

  private final SparkMax motor = new SparkMax(Constants.Subsystems.intakeDeployId, SparkMax.MotorType.kBrushless);

  private final SparkMaxConfig softLimitsEnabled = new SparkMaxConfig();
  private final SparkMaxConfig softLimitsDisabled = new SparkMaxConfig();

  private final AbsoluteEncoder encoder = motor.getAbsoluteEncoder();
  private final SparkClosedLoopController controller = motor.getClosedLoopController();

  private final double deployPositionMin = 130;
  private final double deployPositionMax = 258;

  private final double deployPositionRetracted = 250;
  private final double deployPositionDeployed = 135;

  // How to calibrate zero offset:
  // 1. Set zero offset to 0
  // 2. Manually move the intake to the desired "retracted" position
  // 3. Read the absolute encoder position from the dashboard and set that as the zero offset
  private final double zeroOffset = 0;
  private final double conversionFactor = 360;

  // Must be a value between 0 and 1.
  private final double maxOutput = 0.75;

  public IntakeDeploy() {
    final SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(true);

    config.closedLoop
        .pid(0.008, 0.0, 0.001, ClosedLoopSlot.kSlot0)
        .outputRange(maxOutput * -1, maxOutput)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    config.softLimit
        .forwardSoftLimit(deployPositionMax)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(deployPositionMin)
        .reverseSoftLimitEnabled(true);

    config.absoluteEncoder
        .setSparkMaxDataPortConfig()
        .positionConversionFactor(conversionFactor)
        .inverted(false)
        .zeroOffset(zeroOffset / conversionFactor);

    if (motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring Intake Deploy Motor", false);
    }

    softLimitsEnabled.softLimit.forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
    softLimitsDisabled.softLimit.forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false);
  }

  private void setPosition(double position) {
    controller.setSetpoint(position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void deploy() {
    setPosition(deployPositionDeployed);
  }

  public void retract() {
    setPosition(deployPositionRetracted);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  // Safe bounds for override — well away from the 0°/360° wrap point
  private final double overrideMin = 2;
  private final double overrideMax = 350;

  private double deployOverrideCurrentPosition = 0;
  private double deployOverridePreviousPosition = 0;
  private boolean deployOverrideRollover = false;

  public Command deployOverrideCommand(boolean out) {
    return new FunctionalCommand(
        // Init
        () -> {
          motor.configure(softLimitsDisabled, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
          deployOverrideCurrentPosition = getPosition();
          deployOverridePreviousPosition = deployOverrideCurrentPosition;
          deployOverrideRollover = false;
        },
        // Execute
        () -> {
          // Detect rollover: a large sudden jump means the encoder wrapped
          double currentPosition = getPosition();
          if (Math.abs(currentPosition - deployOverridePreviousPosition) > 180) {
            deployOverrideRollover = true;
            motor.stopMotor();
            DriverStation.reportWarning("Intake deploy override: encoder rollover detected, stopping", false);
            return;
          }
          deployOverridePreviousPosition = currentPosition;

          deployOverrideCurrentPosition += out ? -1 : 1;
          deployOverrideCurrentPosition = MathUtil.clamp(deployOverrideCurrentPosition, overrideMin, overrideMax);
          setPosition(deployOverrideCurrentPosition);
        },
        // End
        interrupted -> {
          motor.configure(softLimitsEnabled, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }, () -> deployOverrideRollover, this).withName("Intake Deploy: Override " + (out ? "Out" : "In"));
  }
}

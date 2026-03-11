package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeDeploy extends SubsystemBase {

  private final double mechanismMinAngle = 48;
  private final double mechanismMaxAngle = 180;

  private final double retractedAngle = 50;
  private final double deployedAngle = 175;

  // Note: We should not use our zero offset to indicate resting position (either deployed or not).
  // This opens us up to the risk of potentially rolling over (past 0, or past 360) which
  // can result in the control loop telling the motor to turn in the opposite direction to get back to the setpoint.
  //
  // How to calibrate zero offset:
  // 1. Set zero offset to 0
  // 2. Manually move the intake to the desired "retracted" position
  // 3. Read the absolute encoder position from the dashboard and set that as the zero offset
  private final double zeroOffset = 0;
  private final double conversionFactor = 360;

  // The maximum output speed (percentage). Must be between 0 and 1.
  private final double maxOutput = 0.3;

  private final SparkMax motor = new SparkMax(Constants.Subsystems.intakeDeployId, SparkMax.MotorType.kBrushless);

  private final SparkMaxConfig softLimitsEnabled = new SparkMaxConfig();
  private final SparkMaxConfig softLimitsDisabled = new SparkMaxConfig();

  private final AbsoluteEncoder encoder = motor.getAbsoluteEncoder();
  private final SparkClosedLoopController controller = motor.getClosedLoopController();

  public IntakeDeploy() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(true);

    config.closedLoop
        .pid(
            // P: Proportional gain, how aggressively the controller responds.
            0.005,
            // I: Integral gain, how much the controller responds based on accumulated error over time.
            // Should likely remain 0 for the current mechanism, since there is some slop in the gearing.
            0.0,
            // D: Derivative gain, how much the controller responds based on the rate of change of the error.
            // Can help reduce overshoot and improve stability.
            0.001)
        .outputRange(-maxOutput, maxOutput)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    config.softLimit
        .forwardSoftLimit(mechanismMaxAngle)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(mechanismMinAngle)
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
    controller.setSetpoint(position, SparkMax.ControlType.kPosition);
  }

  private boolean deployed = false;

  private void deploy() {
    deployed = true;
    setPosition(deployedAngle);
  }

  private void retract() {
    deployed = false;
    setPosition(retractedAngle);
  }

  public Command toggleDeployCommand() {
    return runOnce(() -> {
      if (deployed) {
        retract();
      } else {
        deploy();
      }
    }).withName("Intake Deploy: Toggle");
  }

  private double deployOverrideCurrentPosition = 0;

  public Command deployOverrideCommand(boolean out) {
    return new FunctionalCommand(() -> {
      motor.configure(softLimitsDisabled, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      deployOverrideCurrentPosition = getPosition();
    }, () -> {
      deployOverrideCurrentPosition += out ? -1 : 1;
      controller.setSetpoint(deployOverrideCurrentPosition, SparkMax.ControlType.kPosition);
    }, interrupted -> {
      motor.configure(softLimitsEnabled, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }, () -> false, this).withName("Intake Deploy: Override " + (out ? "Out" : "In"));
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getOutputCurrent() {
    return motor.getOutputCurrent();
  }

  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }
}

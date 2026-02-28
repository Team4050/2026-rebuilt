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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  // default units are rotations
  private double encoderPositionMin = 0.0;
  private double encoderPositionMax = 140;

  private final SparkMax intake = new SparkMax(Constants.Subsystems.intakeRollerId, SparkMax.MotorType.kBrushless);
  private final SparkMax intakeDeploy = new SparkMax(Constants.Subsystems.intakeDeployId,
      SparkMax.MotorType.kBrushless);

  private final AbsoluteEncoder encoder = intakeDeploy.getAbsoluteEncoder();
  private final SparkClosedLoopController controller = intakeDeploy.getClosedLoopController();

  public Intake() {
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    SparkMaxConfig deployConfig = new SparkMaxConfig();

    intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    deployConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    deployConfig.closedLoop
        .pid(0.008, 0.0, 0.001, ClosedLoopSlot.kSlot0)
        .outputRange(-0.5, 0.5)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    deployConfig.softLimit
        .forwardSoftLimit(encoderPositionMax)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(encoderPositionMin)
        .reverseSoftLimitEnabled(true);
    deployConfig.absoluteEncoder
        .setSparkMaxDataPortConfig()
        .positionConversionFactor(360)
        .inverted(true)
        .zeroOffset(0.508333333);

    if (intake
        .configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring Intake Motor", false);
    }

    if (intakeDeploy
        .configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring Intake Deploy Motor", false);
    }
  }

  private void setPosition(double position) {
    controller.setSetpoint(position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  private double clampPosition(double position) {
    if (position > encoderPositionMax) {
      return encoderPositionMax;
    } else if (position < encoderPositionMin) {
      return encoderPositionMin;
    } else {
      return position;
    }
  }

  /**
   * Set a position for the climber to move to.
   */
  public void setTargetPosition(double position) {
    setPosition(clampPosition(position));
  }

  public void stop() {
    intakeStop();
  }

  public void intakeStop() {
    intake.set(0.0);
  }

  public void intakeForward() {
    intake.set(0.25);
  }

  public void intakeReverse() {
    intake.set(-0.25);
  }

  public void deployOut() {
    setPosition(encoderPositionMax);
  }

  /**
   *
   */
  public void deployIn() {
    setPosition(encoderPositionMin);
  }

  /**
   * Stop the climber.
   */
  public void deployStop() {
    controller.setSetpoint(encoder.getPosition(), SparkMax.ControlType.kPosition);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  private double deployPosition = 0.0;

  public void deployForward() {
    setPosition(deployPosition += 1);
  }
}

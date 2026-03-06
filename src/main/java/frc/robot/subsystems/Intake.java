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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  // default units are rotations
  private double encoderPositionMin = 0.0;
  private double encoderPositionMax = 140;

  // This stets the deploy override position.
  private double deployOverrideCurrentPosition = 0.0;

  private final SparkMax rollers = new SparkMax(Constants.Subsystems.intakeRollerId, SparkMax.MotorType.kBrushless);
  private final SparkMax deploy = new SparkMax(Constants.Subsystems.intakeDeployId, SparkMax.MotorType.kBrushless);

  private final AbsoluteEncoder deployEncoder = deploy.getAbsoluteEncoder();
  private final SparkClosedLoopController deployController = deploy.getClosedLoopController();

  public Intake() {
    SparkMaxConfig rollersConfig = new SparkMaxConfig();
    rollersConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    if (rollers
        .configure(rollersConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring Intake Motor", false);
    }

    SparkMaxConfig deployConfig = new SparkMaxConfig();
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

    if (deploy
        .configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring Intake Deploy Motor", false);
    }
  }

  // ===== Rollers =====

  private void stop() {
    rollers.stopMotor();
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("Intake: Stop");
  }

  public void intakeForward() {
    rollers.set(0.25);
  }

  public void intakeReverse() {
    rollers.set(-0.25);
  }

  // ===== Deploy =====

  private void setPosition(double position) {
    deployController.setSetpoint(position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setTargetPosition(double position) {
    setPosition(MathUtil.clamp(position, encoderPositionMin, encoderPositionMax));
  }

  public void deployOut() {
    setPosition(encoderPositionMax);
  }

  public void deployIn() {
    setPosition(encoderPositionMin);
  }

  public double getPosition() {
    return deployEncoder.getPosition();
  }

  // These methods are alternit/override controls for deplyment and are used to  manuly set the defalt deploy position.
  public void deployOverrideOut() {
    setPosition(deployOverrideCurrentPosition += 1);
  }

  public void deployOverrideIn() {
    setPosition(deployOverrideCurrentPosition -= 1);
  }
}

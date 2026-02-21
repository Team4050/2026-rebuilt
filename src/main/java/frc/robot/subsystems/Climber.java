package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  // default units are rotations
  private double encoderPositionMin = 0.0;
  // TODO at least max position must be calibrated manually and refactored here
  private double encoderPositionMax = 10.0;

  private final SparkMax leaderMotor = new SparkMax(Constants.Subsystems.climberPrimaryId, MotorType.kBrushless);
  private final SparkMax followerMotor = new SparkMax(Constants.Subsystems.climberFollowerId, MotorType.kBrushless);
  private final RelativeEncoder encoder = leaderMotor.getEncoder();
  private final SparkClosedLoopController controller = leaderMotor.getClosedLoopController();

  public Climber() {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
    leaderConfig.closedLoop.pid(0.3, 0.0, 0.0, ClosedLoopSlot.kSlot0).outputRange(-0.5, 0.5);
    leaderConfig
        .softLimit
        .forwardSoftLimit(encoderPositionMax)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(encoderPositionMin)
        .reverseSoftLimitEnabled(true);

    if (leaderMotor
        .configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      throw new IllegalStateException("Error configuring Climber Leader Motor");
    }

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).follow(leaderMotor, false);

    if (followerMotor
        .configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning(
          "WARNING: Climber follower motor failed to configure. Running leader only.", false);
    }

    // on startup, assume climber is in the "down" position
    encoder.setPosition(encoderPositionMin);
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

  /**
   * Move the climber up at full speed
   */
  public void up() {
    setPosition(encoderPositionMax);
  }

  /**
   * Move the climber down at full speed
   */
  public void down() {
    setPosition(encoderPositionMin);
  }

  /**
   * Stop the climber.
   */
  public void stop() {
    controller.setSetpoint(encoder.getPosition(), SparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    /* currently unused */
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }
}

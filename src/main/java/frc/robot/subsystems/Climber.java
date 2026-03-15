package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Home;

public class Climber extends SubsystemBase implements Homeable {
  public final double ENCODER_POSITION_TOP = 0.0;

  // manually calibrated 3/14/2026 during climber testing
  public final double ENCODER_POSITION_BOTTOM = 55.0;

  public final double ENCODER_POSITION_L1 = 30.0;

  // The maximum output speed (percentage) of the closed loop controller.
  // Must be between 0 and 1.
  private final double MAX_OUTPUT = 0.3;

  private final SparkMax leaderMotor = new SparkMax(Constants.Subsystems.climberPrimaryId, MotorType.kBrushless);
  private final SparkMax followerMotor = new SparkMax(Constants.Subsystems.climberFollowerId, MotorType.kBrushless);
  private final RelativeEncoder encoder = leaderMotor.getEncoder();

  private final SparkClosedLoopController pidController = leaderMotor.getClosedLoopController();

  private final double LIMIT_THRESHOLD = 0.2;

  public Climber() {
    var leaderConfig = new SparkMaxConfig();
    leaderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).inverted(true);
    leaderConfig.closedLoop.pid(0.1, 0.0, 0.0).outputRange(-MAX_OUTPUT, MAX_OUTPUT);

    if (leaderMotor
        .configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("WARNING: Climber Leader Motor failed to configure. Climber may not work.", false);
    }

    var followerConfig = new SparkMaxConfig();
    followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).follow(leaderMotor, false);

    if (followerMotor
        .configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("WARNING: Climber Follower Motor failed to configure. Running leader only.", false);
    }
  }

  public void setPosition(double position) {
    pidController.setSetpoint(position, SparkMax.ControlType.kPosition);
  }

  public void stop() {
    leaderMotor.stopMotor();
  }

  /** Get the leader motor's encoder position. */
  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  /** Get whether the primary climber is at its upper limit. */
  public boolean primaryAtUpperLimit() {
    return Math.abs(encoder.getPosition() - ENCODER_POSITION_TOP) < LIMIT_THRESHOLD;
  }

  /** Get the output current of the climber's leader motor */
  public double getLeaderCurrent() {
    return leaderMotor.getOutputCurrent();
  }

  /** Get the output current of the climber's follower motor */
  public double getFollowerCurrent() {
    return followerMotor.getOutputCurrent();
  }

  /** Get whether the primary climber is at its lower limit. */
  public boolean primaryAtLowerLimit() {
    return Math.abs(encoder.getPosition() - ENCODER_POSITION_BOTTOM) < LIMIT_THRESHOLD;
  }

  private void overridePrimaryUp() {
    setPosition(ENCODER_POSITION_TOP);
  }

  private void overridePrimaryDown() {
    setPosition(ENCODER_POSITION_BOTTOM);
  }

  /** Move the climber so that the robot has climbed to L1 */
  public void climbToLevel1() {
    setPosition(ENCODER_POSITION_L1);
  }

  // ===================== Control Commands =====================

  /** Command that drives the climber up while active, stops on end. */
  public Command overridePrimaryUpCommand() {
    return startEnd(this::overridePrimaryUp, this::stop).withName("Climber: Override Up");
  }

  /** Command that drives the climber down while active, stops on end. */
  public Command overridePrimaryDownCommand() {
    return startEnd(this::overridePrimaryDown, this::stop).withName("Climber: Override Down");
  }

  // ========================== Homing ==========================

  private static final double HOMING_SPEED = 0.1;
  private static final double STALL_CURRENT_AMPS = 10.0;
  private static final double STALL_VELOCITY_RPM = 5.0;
  private static final double STALL_TIME_SEC = 0.1;

  /**
   * Command for stall homing the climber.
   *
   * @return The generated command.
   */
  public Command homeCommand() {
    return new Home(this, STALL_CURRENT_AMPS, STALL_VELOCITY_RPM, STALL_TIME_SEC).withName("Climber: Home");
  }

  @Override
  public void driveToHome() {
    leaderMotor.set(-HOMING_SPEED);
  }

  @Override
  public void stopHoming() {
    stop();
  }

  @Override
  public double getHomingCurrent() {
    return leaderMotor.getOutputCurrent();
  }

  @Override
  public double getHomingVelocity() {
    return Math.abs(encoder.getVelocity());
  }

  @Override
  public void onHomeComplete() {
    encoder.setPosition(ENCODER_POSITION_TOP);
  }
}

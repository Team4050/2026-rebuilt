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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Home;

public class Climber extends SubsystemBase implements Homeable {
  private final double ENCODER_POSITION_MIN = 0.0;

  // manually calibrated 2/26/2026 for climber rev. 2
  private final double ENCODER_POSITION_MAX = 54.5;

  // The maximum output speed (percentage) of the closed loop controller.
  // Must be between 0 and 1.
  private final double MAX_OUTPUT = 0.1;

  private final SparkMax leaderMotor = new SparkMax(Constants.Subsystems.climberPrimaryId, MotorType.kBrushless);
  private final SparkMax followerMotor = new SparkMax(Constants.Subsystems.climberFollowerId, MotorType.kBrushless);
  private final RelativeEncoder encoder = leaderMotor.getEncoder();

  private final SparkClosedLoopController pidController = leaderMotor.getClosedLoopController();

  public Climber() {
    var leaderConfig = new SparkMaxConfig();
    leaderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).inverted(true);
    leaderConfig.closedLoop.pid(0.3, 0.0, 0.0).outputRange(-MAX_OUTPUT, MAX_OUTPUT);

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

  private void setPosition(double position) {
    pidController.setSetpoint(position, SparkMax.ControlType.kPosition);
  }

  private void handleClimbStage1() {
    if (levelsClimbed == 3) {
      return;
    }

    /* TODO make sure auto is handeled (do not pass L1) */

    if (primaryAtUpperLimit()) {
      /* TODO maybe pause here to settle before running stage 2 */
      climbStage = ClimbStage.STAGE_2;
    } else {
      primaryUp();
    }
  }

  private void handleClimbStage2() {
    if (levelsClimbed == 3) {
      return;
    }

    /* TODO make sure auto is handeled (do not pass L1) */

    if (primaryAtUpperLimit()) {
      /* TODO maybe pause here to settle before running stage 1 again */
      climbStage = ClimbStage.STAGE_1;

    } else {
      primaryDown();
    }
  }

  /**
   * Set a position for the climber to move to.
   */
  public void setTargetPosition(double position) {
    setPosition(MathUtil.clamp(position, ENCODER_POSITION_TOP, ENCODER_POSITION_BOTTOM));
  }

  /**
   * Manually move the primary climber up at full speed
   */
  private void up() {
    // "up" refers to climber primary moving up, and encoder values change in opposite direction
    setPosition(ENCODER_POSITION_TOP);
  }

  /**
   * Manually ove the primary climber down at full speed
   */
  private void down() {
    // "down" refers to climber primary down, and encoder values change in opposite direction
    setPosition(ENCODER_POSITION_BOTTOM);
  }

  /**
   * Stop the climber.
   */
  private void stop() {
    leaderMotor.stopMotor();

    // TODO: Determine if we need to use the setpoint for stopping.
    // - One reason not to use it is that it will keep the PID loop running, sending power to the motor and eating up CPU
    // - One reason to use it is that it will hold the climber in place when stopped, using the motor's power. This can
    //   can partially be negated with brake mode.
    // pidController.setSetpoint(encoder.getPosition(), SparkMax.ControlType.kPosition);
  }

  /** Command that drives the climber up while active, stops on end. */
  public Command upCommand() {
    return startEnd(this::up, this::stop).withName("Climber: Up");
  }

  /** Command that drives the climber down while active, stops on end. */
  public Command downCommand() {
    return startEnd(this::down, this::stop).withName("Climber: Down");
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("Climber: Stop");
  }

  // ===================== Homing =====================

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
    encoder.setPosition(ENCODER_POSITION_MIN);
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }
}

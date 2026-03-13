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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Home;

public class Climber extends SubsystemBase implements Homeable {
  private final double ENCODER_POSITION_TOP = 0.0;

  // manually calibrated 2/26/2026 for climber rev. 2
  private final double ENCODER_POSITION_BOTTOM = 54.5;

  // The maximum output speed (percentage) of the closed loop controller.
  // Must be between 0 and 1.
  private final double MAX_OUTPUT = 0.1;

  private final SparkMax leaderMotor = new SparkMax(Constants.Subsystems.climberPrimaryId, MotorType.kBrushless);
  private final SparkMax followerMotor = new SparkMax(Constants.Subsystems.climberFollowerId, MotorType.kBrushless);
  private final RelativeEncoder encoder = leaderMotor.getEncoder();

  private final SparkClosedLoopController pidController = leaderMotor.getClosedLoopController();

  public enum ClimbStage {
    STAGE_1, STAGE_2
  }

  private ClimbStage climbStage = ClimbStage.STAGE_1;

  private int numLevelsClimbed = 0;

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
    if (primaryAtUpperLimit()) {
      climbStage = ClimbStage.STAGE_2;
    } else {
      overridePrimaryUp();
    }
  }

  private void handleClimbStage2() {
    if (primaryAtLowerLimit()) {
      numLevelsClimbed++;
      climbStage = ClimbStage.STAGE_1;
    } else {
      overridePrimaryDown();
    }
  }

  /**
   * Manually move the primary climber up at full speed
   */
  private void overridePrimaryUp() {
    // "up" refers to climber primary moving up, and encoder values change in opposite direction
    setPosition(ENCODER_POSITION_TOP);
  }

  /**
   * Manually ove the primary climber down at full speed
   */
  private void overridePrimaryDown() {
    // "down" refers to climber primary down, and encoder values change in opposite direction
    setPosition(ENCODER_POSITION_BOTTOM);
  }

  /**
   * Stop the climber.
   */
  private void stop() {
    leaderMotor.stopMotor();

    // TODO: Determine if we need to use the setpoint for stopping.
    // stopMotor() is ideal, if we might require setSetpoint() to keep us at L3 max height (test this)
    // pidController.setSetpoint(encoder.getPosition(), SparkMax.ControlType.kPosition);
  }

  private void climb() {
    if (climbStage == ClimbStage.STAGE_1) {
      handleClimbStage1();
    } else {
      handleClimbStage2();
    }
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  public ClimbStage getClimbStage() {
    return climbStage;
  }

  public int getNumLevelsClimbed() {
    return numLevelsClimbed;
  }

  public boolean primaryAtUpperLimit() {
    return Math.abs(encoder.getPosition() - ENCODER_POSITION_TOP) < 0.1;
  }

  public boolean primaryAtLowerLimit() {
    return Math.abs(encoder.getPosition() - ENCODER_POSITION_BOTTOM) < 0.1;
  }

  /** Command that drives the climber up while active, stops on end. */
  public Command overridePrimaryUpCommand() {
    return startEnd(this::overridePrimaryUp, this::stop).withName("Climber: Up");
  }

  /** Command that drives the climber down while active, stops on end. */
  public Command overridePrimaryDownCommand() {
    return startEnd(this::overridePrimaryDown, this::stop).withName("Climber: Down");
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("Climber: Stop");
  }

  public Command climbCommand() {
    return new RunCommand(this::climb, this).finallyDo(this::stop).withName("Climber: Auto L3 Climb");
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
    encoder.setPosition(ENCODER_POSITION_TOP);
  }
}

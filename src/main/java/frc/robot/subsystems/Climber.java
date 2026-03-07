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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  // default units are rotations
  private final double ENCODER_POSITION_TOP = 0.0;

  // manually calibrated 2/26/2026 for climber rev. 2
  private final double ENCODER_POSITION_BOTTOM = 54.5;

  private final SparkMax leaderMotor = new SparkMax(Constants.Subsystems.climberPrimaryId, MotorType.kBrushless);
  private final SparkMax followerMotor = new SparkMax(Constants.Subsystems.climberFollowerId, MotorType.kBrushless);
  private final RelativeEncoder encoder = leaderMotor.getEncoder();

  private final SparkClosedLoopController pidController = leaderMotor.getClosedLoopController();

  public enum ClimbStage {

    STAGE_1("Stage 1 (Primary Climber Extending)"), STAGE_2("Stage 2 (Secondary Climber Returning)");

    private String climbStageString;

    ClimbStage(String climbStageString) {
      this.climbStageString = climbStageString;
    }
  }

  private ClimbStage climbStage = ClimbStage.STAGE_1;
  private int levelsClimbed = 0;

  // stall homing constants
  private static final double HOMING_SPEED = 0.1;
  private static final double STALL_CURRENT_AMPS = 10.0;
  private static final double STALL_VELOCITY_RPM = 5.0;
  private static final double STALL_TIME_SEC = 0.1;

  private boolean abort_homing = false;

  public Climber() {
    final SparkMaxConfig leaderMotorConfig = new SparkMaxConfig();
    leaderMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).inverted(true);
    leaderMotorConfig.closedLoop.pid(0.3, 0.0, 0.0, ClosedLoopSlot.kSlot0).outputRange(-0.1, 0.1);

    if (leaderMotor
        .configure(
            leaderMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("WARNING: Climber Leader Motor failed to configure. Climber may not work.", false);
    }

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).follow(leaderMotor, false);

    if (followerMotor
        .configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("WARNING: Climber Follower Motor failed to configure. Running leader only.", false);
    }
  }

  private void setPosition(double position) {
    pidController.setSetpoint(position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  private boolean primaryAtUpperLimit() {
    return Math.abs(encoder.getPosition() - ENCODER_POSITION_TOP) < 0.1;
  }

  private boolean primaryAtLowerLimit() {
    return Math.abs(encoder.getPosition() - ENCODER_POSITION_BOTTOM) < 0.1;
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
  public void primaryUp() {
    abort_homing = true;

    // "up" refers to climber primary moving up, and encoder values change in opposite direction
    setPosition(ENCODER_POSITION_TOP);
  }

  /**
   * Manually ove the primary climber down at full speed
   */
  public void primaryDown() {
    abort_homing = true;

    // "down" refers to climber primary down, and encoder values change in opposite direction
    setPosition(ENCODER_POSITION_BOTTOM);
  }

  /**
   * Stop the climber.
   */
  public void stop() {
    pidController.setSetpoint(encoder.getPosition(), SparkMax.ControlType.kPosition);
  }

  /**
   * Automatically climb to L3.
   */
  public void climb() {

  }

  /**
   * Command for stall homing the climber.
   *
   * @return The generated command.
   */
  public Command homeCommand() {
    return new Command() {
      private final Timer stallTimer = new Timer();

      @Override
      public void initialize() {
        System.out.println("Climber - stall homing start.");

        stallTimer.stop();
        stallTimer.reset();

        abort_homing = false;
      }

      @Override
      public void execute() {
        if (abort_homing) {
          System.out.println("Climber - stall homing aborted.");
          leaderMotor.stopMotor();
          return;
        }

        leaderMotor.set(-HOMING_SPEED);

        boolean stalled = leaderMotor.getOutputCurrent() > STALL_CURRENT_AMPS
            && Math.abs(encoder.getVelocity()) < STALL_VELOCITY_RPM;

        if (stalled) {
          if (!stallTimer.isRunning()) {
            stallTimer.start();
            System.out.println("Climber - stalled.");
          }
        } else {
          stallTimer.stop();
          stallTimer.reset();
        }
      }

      @Override
      public boolean isFinished() {
        return abort_homing || stallTimer.hasElapsed(STALL_TIME_SEC);
      }

      @Override
      public void end(boolean interrupted) {
        if (abort_homing)
          return;
        stop();
        encoder.setPosition(ENCODER_POSITION_TOP);
      }
    }.withName("ClimberHome");
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public String getClimberStateString() {
    return climbStage.climbStageString;
  }

  public int getLevelsClimbed() {
    return levelsClimbed;
  }

  @Override
  public void periodic() {
    /* currently unused */
  }
}

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  // default units are rotations
  private double encoderPositionMin = 0.0;

  // calibrated 2/26/2026
  private double encoderPositionMax = 54.5;

  private final SparkMax leaderMotor = new SparkMax(Constants.Subsystems.climberPrimaryId, MotorType.kBrushless);
  private final SparkMax followerMotor = new SparkMax(Constants.Subsystems.climberFollowerId, MotorType.kBrushless);
  private final RelativeEncoder encoder = leaderMotor.getEncoder();
  private final SparkClosedLoopController controller = leaderMotor.getClosedLoopController();
  private final SparkMaxConfig leaderMotorConfig = new SparkMaxConfig();

  // stall homing constants
  private static final double HOMING_SPEED = 0.1;
  private static final double STALL_CURRENT_AMPS = 10.0;
  private static final double STALL_VELOCITY_RPM = 5.0;
  private static final double STALL_TIME_SEC = 0.1;

  private boolean abort_homing = false;

  public Climber() {
    leaderMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).inverted(true);
    leaderMotorConfig.closedLoop.pid(0.3, 0.0, 0.0, ClosedLoopSlot.kSlot0).outputRange(-0.1, 0.1);
    leaderMotorConfig.softLimit.forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false);

    if (leaderMotor
        .configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
            != REVLibError.kOk) {
      throw new IllegalStateException("Climber Leader Motor failed to configure.");
    }

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).follow(leaderMotor, false);

    if (followerMotor
        .configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
            != REVLibError.kOk) {
      DriverStation.reportWarning(
          "WARNING: Climber Follower Motor failed to configure. Running leader only.", false);
    }

    // on startup, assume climber is in the "down" position
    // TODO fix this
    // encoder.setPosition(encoderPositionMin + 10);
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
    abort_homing = true;
    setPosition(encoderPositionMax);
  }

  /**
   * Move the climber down at full speed
   */
  public void down() {
    abort_homing = true;
    setPosition(encoderPositionMin);
  }

  /**
   * Stop the climber.
   */
  public void stop() {
    controller.setSetpoint(encoder.getPosition(), SparkMax.ControlType.kPosition);
  }

  public void runOpenLoop(double speed) {
    leaderMotor.set(speed);
  }

  public double getCurrent() {
    return leaderMotor.getOutputCurrent();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public void zeroEncoder(double position) {
    encoder.setPosition(position);
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
        System.out.println("Stall homing");

        abort_homing = false;
      }

      @Override
      public void execute() {
        if (abort_homing) {
          System.out.println("Aborted!");
          leaderMotor.stopMotor();
          return;
        }

        runOpenLoop(-HOMING_SPEED);

        boolean stalled = getCurrent() > STALL_CURRENT_AMPS && Math.abs(getVelocity()) < STALL_VELOCITY_RPM;

        if (stalled) {
          if (!stallTimer.isRunning()) {
            stallTimer.start();
            System.out.println("Stalled!");
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
        if (abort_homing) return;
        stop();
        encoder.setPosition(encoderPositionMin);
      }
    }.withName("ClimberHome");
  }

  @Override
  public void periodic() {
    /* currently unused */
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }
}

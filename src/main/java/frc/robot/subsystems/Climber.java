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
  // TODO at least max position must be calibrated manually and refactored here
  private double encoderPositionMax = 10.0;

  private final SparkMax leaderMotor = new SparkMax(Constants.Subsystems.climberPrimaryId, MotorType.kBrushless);
  private final SparkMax followerMotor = new SparkMax(Constants.Subsystems.climberFollowerId, MotorType.kBrushless);
  private final RelativeEncoder encoder = leaderMotor.getEncoder();
  private final SparkClosedLoopController controller = leaderMotor.getClosedLoopController();
  private final SparkMaxConfig leaderMotorConfig = new SparkMaxConfig();

  // stall homing constants
  private static final double HOMING_SPEED = 0.2;
  private static final double STALL_CURRENT_AMPS = 40.0;
  private static final double STALL_VELOCITY_RPM = 5.0;
  private static final double STALL_TIME_SEC = 0.25;

  public Climber() {
    leaderMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
    leaderMotorConfig.closedLoop.pid(0.3, 0.0, 0.0, ClosedLoopSlot.kSlot0).outputRange(-0.5, 0.5);
    leaderMotorConfig.softLimit.forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);

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

  private Command getHomeCommand(boolean direction) {
    return new Command() {
      private final Timer stallTimer = new Timer();
      private boolean aborted = false;

      @Override
      public void initialize() {
        stallTimer.stop();
        stallTimer.reset();

        if (direction) {
          leaderMotorConfig.softLimit.forwardSoftLimitEnabled(false);
        } else {
          leaderMotorConfig.softLimit.reverseSoftLimitEnabled(false);
        }

        if (leaderMotor.configure(
                leaderMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters)
                != REVLibError.kOk) {
          DriverStation.reportWarning(
              "WARNING: Failed to reconfigure Leader Motor during homing " + (direction ? "up" : "down")
                  + " command. Aborting command.",
              false);
          aborted = true;
        }
      }

      @Override
      public void execute() {
        if (aborted) return;

        runOpenLoop(direction ? HOMING_SPEED : -HOMING_SPEED);

        boolean stalled = getCurrent() > STALL_CURRENT_AMPS && Math.abs(getVelocity()) < STALL_VELOCITY_RPM;

        if (stalled) {
          if (!stallTimer.isRunning()) {
            stallTimer.start();
          }
        } else {
          stallTimer.stop();
          stallTimer.reset();
        }
      }

      @Override
      public boolean isFinished() {
        return aborted || stallTimer.hasElapsed(STALL_TIME_SEC);
      }

      @Override
      public void end(boolean interrupted) {
        stop();

        if (aborted) {
          return;
        }

        if (direction) {
          leaderMotorConfig
              .softLimit
              .forwardSoftLimit(encoder.getPosition())
              .forwardSoftLimitEnabled(true);
        } else {
          leaderMotorConfig
              .softLimit
              .reverseSoftLimit(encoder.getPosition())
              .reverseSoftLimitEnabled(true);
        }

        if (leaderMotor.configure(
                leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
                != REVLibError.kOk) {
          throw new IllegalStateException("Climber Leader Motor failed to reconfigure after home "
              + (direction ? "up" : "down") + " command.");
        }
      }
    }.withName(direction ? "ClimberHomeUp" : "ClimberHomeDown");
  }

  /**
   * Command for stall homing the maximum position of the Climber.
   *
   * @return The home up Command.
   */
  public Command homeUpCommand() {
    return getHomeCommand(true);
  }

  /**
   * Command for stall homing the maximum position of the Climber.
   *
   * @return The home down Command.
   */
  public Command homeDownCommand() {
    return getHomeCommand(false);
  }

  @Override
  public void periodic() {
    /* currently unused */
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }
}

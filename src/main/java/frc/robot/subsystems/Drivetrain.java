package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily be used in
 * command-based projects.
 */
public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private static final double simLoopPeriod = 0.004; // 4 ms
  private Notifier simNotifier = null;
  private double lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d redAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  private final Field2d fieldPosition = new Field2d();

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>
   * This constructs the underlying hardware devices, so users should not construct the devices themselves. If they need
   * the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants
   *          Drivetrain-wide constants for the swerve drive
   * @param modules
   *          Constants for each specific module
   */
  public Drivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>
   * This constructs the underlying hardware devices, so users should not construct the devices themselves. If they need
   * the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants
   *          Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency
   *          The frequency to run the odometry loop. If unspecified or set to 0 Hz, this is 250 Hz on CAN FD, and 100
   *          Hz on CAN 2.0.
   * @param modules
   *          Constants for each specific module
   */
  public Drivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>
   * This constructs the underlying hardware devices, so users should not construct the devices themselves. If they need
   * the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants
   *          Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency
   *          The frequency to run the odometry loop. If unspecified or set to 0 Hz, this is 250 Hz on CAN FD, and 100
   *          Hz on CAN 2.0.
   * @param odometryStandardDeviation
   *          The standard deviation for odometry calculation in the form [x, y, theta]ᵀ, with units in meters and
   *          radians
   * @param visionStandardDeviation
   *          The standard deviation for vision calculation in the form [x, y, theta]ᵀ, with units in meters and radians
   * @param modules
   *          Constants for each specific module
   */
  public Drivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request
   *          Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> request) {
    return run(() -> this.setControl(request.get()));
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective. If we haven't applied the operator perspective before, then
     * we should apply it regardless of DS state. This allows us to correct the perspective in case the robot code
     * restarts mid-match. Otherwise, only check and apply the operator perspective if the DS is disabled. This ensures
     * driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent(allianceColor -> {
        setOperatorPerspectiveForward(
            allianceColor == Alliance.Red ? redAlliancePerspectiveRotation : blueAlliancePerspectiveRotation);
        hasAppliedOperatorPerspective = true;
      });
    }

    fieldPosition.setRobotPose(getPose());
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - lastSimTime;
      lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    simNotifier.startPeriodic(simLoopPeriod);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate while still accounting
   * for measurement noise.
   *
   * @param visionRobotPoseMeters
   *          The pose of the robot as measured by the vision camera.
   * @param timestampSeconds
   *          The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate while still accounting
   * for measurement noise.
   *
   * <p>
   * Note that the vision measurement standard deviations passed into this method will continue to apply to future
   * measurements until a subsequent call to {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters
   *          The pose of the robot as measured by the vision camera.
   * @param timestampSeconds
   *          The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs
   *          Standard deviations of the vision pose measurement in the form [x, y, theta]ᵀ, with units in meters and
   *          radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters,
        Utils.fpgaToCurrentTime(timestampSeconds),
        visionMeasurementStdDevs);
  }

  /**
   * Return the pose at a given timestamp, if the buffer is not empty.
   *
   * @param timestampSeconds
   *          The timestamp of the pose in seconds.
   * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
   */
  @Override
  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
  }

  // ===================== SysID =====================

  private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

  private double lastRotationOutputVoltage;

  private final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
      new SysIdRoutine.Config(null, Volts.of(4), null),
      new SysIdRoutine.Mechanism(output -> setControl(translationCharacterization.withVolts(output)), log -> {
        logDriveMotor(log, "FL", 0);
        logDriveMotor(log, "FR", 1);
        logDriveMotor(log, "BL", 2);
        logDriveMotor(log, "BR", 3);
      }, this));

  private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(new SysIdRoutine.Config(null, Volts.of(7), null),
      new SysIdRoutine.Mechanism(volts -> setControl(steerCharacterization.withVolts(volts)), log -> {
        logSteerMotor(log, "FL", 0);
        logSteerMotor(log, "FR", 1);
        logSteerMotor(log, "BL", 2);
        logSteerMotor(log, "BR", 3);
      }, this));

  private final SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(new SysIdRoutine.Config(
      /* This is in radians per second², but SysId only supports "volts per second" */
      Volts.of(Math.PI / 6).per(Second),
      /* This is in radians per second, but SysId only supports "volts" */
      Volts.of(Math.PI), null), new SysIdRoutine.Mechanism(output -> {
        lastRotationOutputVoltage = output.in(Volts);
        setControl(rotationCharacterization.withRotationalRate(lastRotationOutputVoltage));
      }, log -> {
        var pigeon = getPigeon2();
        var yaw = pigeon.getYaw();
        var angularVelocity = pigeon.getAngularVelocityZWorld();

        yaw.refresh();
        angularVelocity.refresh();

        log
            .motor("Rotation")
            .voltage(Volts.of(lastRotationOutputVoltage))
            .angularPosition(yaw.getValue())
            .angularVelocity(angularVelocity.getValue());
      }, this));

  private void logDriveMotor(SysIdRoutineLog log, String name, int moduleIndex) {
    var motor = getModule(moduleIndex).getDriveMotor();
    var voltage = motor.getMotorVoltage();
    var position = motor.getPosition();
    var velocity = motor.getVelocity();
    var current = motor.getTorqueCurrent();

    voltage.refresh();
    position.refresh();
    velocity.refresh();
    current.refresh();

    log
        .motor(name)
        .voltage(voltage.getValue())
        .current(current.getValue())
        .angularPosition(position.getValue())
        .angularVelocity(velocity.getValue());
  }

  private void logSteerMotor(SysIdRoutineLog log, String name, int moduleIndex) {
    var motor = getModule(moduleIndex).getSteerMotor();
    var voltage = motor.getMotorVoltage();
    var position = motor.getPosition();
    var velocity = motor.getVelocity();
    var current = motor.getTorqueCurrent();

    voltage.refresh();
    position.refresh();
    velocity.refresh();
    current.refresh();

    log
        .motor(name)
        .voltage(voltage.getValue())
        .current(current.getValue())
        .angularPosition(position.getValue())
        .angularVelocity(velocity.getValue());
  }

  public SendableChooser<Command> buildSysIdChooser() {
    SendableChooser<Command> chooser = new SendableChooser<Command>();
    chooser.setDefaultOption("Translation QS Fwd", sysIdRoutineTranslation.quasistatic(Direction.kForward));
    chooser.addOption("Translation QS Rev", sysIdRoutineTranslation.quasistatic(Direction.kReverse));
    chooser.addOption("Translation Dyn Fwd", sysIdRoutineTranslation.dynamic(Direction.kForward));
    chooser.addOption("Translation Dyn Rev", sysIdRoutineTranslation.dynamic(Direction.kReverse));
    chooser.addOption("Steer QS Fwd", sysIdRoutineSteer.quasistatic(Direction.kForward));
    chooser.addOption("Steer QS Rev", sysIdRoutineSteer.quasistatic(Direction.kReverse));
    chooser.addOption("Steer Dyn Fwd", sysIdRoutineSteer.dynamic(Direction.kForward));
    chooser.addOption("Steer Dyn Rev", sysIdRoutineSteer.dynamic(Direction.kReverse));
    chooser.addOption("Rotation QS Fwd", sysIdRoutineRotation.quasistatic(Direction.kForward));
    chooser.addOption("Rotation QS Rev", sysIdRoutineRotation.quasistatic(Direction.kReverse));
    chooser.addOption("Rotation Dyn Fwd", sysIdRoutineRotation.dynamic(Direction.kForward));
    chooser.addOption("Rotation Dyn Rev", sysIdRoutineRotation.dynamic(Direction.kReverse));
    return chooser;
  }

  // ===================== Getters for RobotState =====================

  public Field2d getFieldPosition() {
    return fieldPosition;
  }

  public Pose2d getPose() {
    return getState().Pose;
  }

  public ChassisSpeeds getSpeeds() {
    return getState().Speeds;
  }

  public SwerveModuleState[] getModuleStates() {
    return getState().ModuleStates;
  }

  public SwerveModuleState[] getModuleTargets() {
    return getState().ModuleTargets;
  }

  public Rotation2d getHeading() {
    return getState().Pose.getRotation();
  }
}

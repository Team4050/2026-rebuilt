package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Optional;

public class RobotState {
    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private RobotState() {}

    // ===================================================================================
    // Drivetrain State
    // ===================================================================================

    private Pose2d currentPose = new Pose2d();
    private Rotation2d gyroHeading = new Rotation2d();
    private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
    private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
    private boolean isMoving = false;
    private double angularVelocityDegreesPerSec = 0.0;

    // Movement thresholds
    private static final double LINEAR_VELOCITY_THRESHOLD = 0.1; // m/s
    private static final double ANGULAR_VELOCITY_THRESHOLD = 5.0; // deg/s

    /** Set the vision-fused pose estimate from the drivetrain */
    public void setCurrentPose(Pose2d pose) {
        this.currentPose = pose;
    }

    /** Get the current vision-fused pose estimate */
    public Pose2d getCurrentPose() {
        return currentPose;
    }

    /** Set gyro heading from Pigeon */
    public void setGyroHeading(Rotation2d heading) {
        this.gyroHeading = heading;
    }

    /** Get raw gyro heading (useful for vision and targeting) */
    public Rotation2d getGyroHeading() {
        return gyroHeading;
    }

    /** Set robot-relative chassis speeds */
    public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
        this.robotRelativeSpeeds = speeds;
        this.angularVelocityDegreesPerSec = Math.toDegrees(speeds.omegaRadiansPerSecond);
        this.isMoving = getLinearVelocity() > LINEAR_VELOCITY_THRESHOLD
                || Math.abs(angularVelocityDegreesPerSec) > ANGULAR_VELOCITY_THRESHOLD;
    }

    /** Get robot-relative chassis speeds */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return robotRelativeSpeeds;
    }

    /** Set field-relative chassis speeds */
    public void setFieldRelativeSpeeds(ChassisSpeeds speeds) {
        this.fieldRelativeSpeeds = speeds;
    }

    /** Get field-relative chassis speeds (for shoot-on-move calculations) */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return fieldRelativeSpeeds;
    }

    /** Check if robot is currently moving (linear or rotational) */
    public boolean isMoving() {
        return isMoving;
    }

    /** Get linear velocity magnitude in m/s */
    public double getLinearVelocity() {
        return Math.hypot(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond);
    }

    /** Get angular velocity in degrees per second */
    public double getAngularVelocityDegreesPerSec() {
        return angularVelocityDegreesPerSec;
    }

    // /** Get the difference between fused pose and raw odometry (for tuning/debugging) */
    // public double getVisionOdometryDifference() {
    //     return currentPose.getTranslation().getDistance(odometryPose.getTranslation());
    // }

    // ===================================================================================
    // VISION STATE
    // Producer: VisionSubsystem
    // Consumer: CommandSwerveDrivetrain
    // ===================================================================================

    public record VisionMeasurement(
            Pose2d pose,
            double timestampSeconds,
            int tagCount,
            double avgTagDistance,
            String source // Which camera/limelight
            ) {}

    private VisionMeasurement latestVisionMeasurement = null;
    private long visionUpdateCount = 0;
    private long visionAcceptedCount = 0;
    private long visionRejectedCount = 0;

    /** Add a new vision measurement to be consumed by the drivetrain */
    public void addVisionMeasurement(VisionMeasurement measurement) {
        this.latestVisionMeasurement = measurement;
        this.visionUpdateCount++;
    }

    /**
     * Consume the latest vision measurement (returns and clears it).
     * Use this in the drivetrain to avoid double-applying measurements.
     */
    public Optional<VisionMeasurement> consumeVisionMeasurement() {
        VisionMeasurement measurement = latestVisionMeasurement;
        latestVisionMeasurement = null;
        return Optional.ofNullable(measurement);
    }

    /** Peek at the latest vision measurement without consuming it */
    public Optional<VisionMeasurement> peekVisionMeasurement() {
        return Optional.ofNullable(latestVisionMeasurement);
    }

    /** Record that a vision measurement was accepted by the pose estimator */
    public void recordVisionAccepted() {
        visionAcceptedCount++;
    }

    /** Record that a vision measurement was rejected */
    public void recordVisionRejected() {
        visionRejectedCount++;
    }

    public long getVisionUpdateCount() {
        return visionUpdateCount;
    }

    public long getVisionAcceptedCount() {
        return visionAcceptedCount;
    }

    public long getVisionRejectedCount() {
        return visionRejectedCount;
    }

    // Examples

    // // ===================================================================================
    // // TARGETING STATE
    // // Producer: Auto routines, driver input, game piece detection
    // // Consumers: Drivetrain (auto-aim), Shooter, Turret, LEDs
    // // ===================================================================================

    // private Translation2d targetPosition = null;
    // private boolean hasActiveTarget = false;
    // private String targetName = "";

    // /** Set the current field-relative target position */
    // public void setTarget(Translation2d target, String name) {
    //     this.targetPosition = target;
    //     this.hasActiveTarget = true;
    //     this.targetName = name;
    // }

    // /** Set target without a name */
    // public void setTarget(Translation2d target) {
    //     setTarget(target, "");
    // }

    // /** Clear the current target */
    // public void clearTarget() {
    //     this.targetPosition = null;
    //     this.hasActiveTarget = false;
    //     this.targetName = "";
    // }

    // /** Get the current target position if one is set */
    // public Optional<Translation2d> getTarget() {
    //     return Optional.ofNullable(targetPosition);
    // }

    // /** Check if there's an active target */
    // public boolean hasActiveTarget() {
    //     return hasActiveTarget;
    // }

    // /** Get the name/description of the current target */
    // public String getTargetName() {
    //     return targetName;
    // }

    // /** Get the field-relative angle from robot to target */
    // public Optional<Rotation2d> getAngleToTarget() {
    //     if (targetPosition == null) {
    //         return Optional.empty();
    //     }
    //     Translation2d robotToTarget = targetPosition.minus(currentPose.getTranslation());
    //     return Optional.of(robotToTarget.getAngle());
    // }

    // /** Get the robot-relative angle to turn to face target */
    // public Optional<Rotation2d> getRotationToTarget() {
    //     return getAngleToTarget().map(angle -> angle.minus(currentPose.getRotation()));
    // }

    // /** Get distance from robot to target in meters */
    // public Optional<Double> getDistanceToTarget() {
    //     if (targetPosition == null) {
    //         return Optional.empty();
    //     }
    //     return Optional.of(currentPose.getTranslation().getDistance(targetPosition));
    // }

    // /** Check if robot is aimed at target within tolerance */
    // public boolean isAimedAtTarget(Rotation2d tolerance) {
    //     return getRotationToTarget()
    //             .map(rot -> Math.abs(rot.getDegrees()) < tolerance.getDegrees())
    //             .orElse(false);
    // }

    // // ===================================================================================
    // // ALLIANCE & MATCH STATE
    // // Producer: DriverStation wrapper (set once at match start)
    // // Consumers: Auto paths, vision filtering, target selection, LEDs
    // // ===================================================================================

    // private Alliance alliance = Alliance.Blue;
    // private boolean allianceKnown = false;

    // /** Update alliance from DriverStation (call periodically until known) */
    // public void updateAlliance() {
    //     DriverStation.getAlliance().ifPresent(a -> {
    //         this.alliance = a;
    //         this.allianceKnown = true;
    //     });
    // }

    // /** Get the current alliance */
    // public Alliance getAlliance() {
    //     return alliance;
    // }

    // /** Check if we've received alliance data from FMS/DS */
    // public boolean isAllianceKnown() {
    //     return allianceKnown;
    // }

    // /** Check if we're on red alliance */
    // public boolean isRedAlliance() {
    //     return alliance == Alliance.Red;
    // }

    // /** Check if we're on blue alliance */
    // public boolean isBlueAlliance() {
    //     return alliance == Alliance.Blue;
    // }

    // // ===================================================================================
    // // GAME PIECE STATE (STUB - customize for your game)
    // // Producer: Intake subsystem, sensors
    // // Consumers: Auto logic, LEDs, driver feedback, scoring commands
    // // ===================================================================================

    // // Example for 2025 Reefscape - adjust for your game
    // public enum GamePiece {
    //     NONE,
    //     CORAL,
    //     ALGAE
    // }

    // private GamePiece heldGamePiece = GamePiece.NONE;
    // private boolean intakeSensorTriggered = false;
    // private boolean indexerSensorTriggered = false;
    // private boolean shooterSensorTriggered = false;

    // public void setHeldGamePiece(GamePiece piece) {
    //     this.heldGamePiece = piece;
    // }

    // public GamePiece getHeldGamePiece() {
    //     return heldGamePiece;
    // }

    // public boolean hasGamePiece() {
    //     return heldGamePiece != GamePiece.NONE;
    // }

    // public boolean hasCoral() {
    //     return heldGamePiece == GamePiece.CORAL;
    // }

    // public boolean hasAlgae() {
    //     return heldGamePiece == GamePiece.ALGAE;
    // }

    // public void setIntakeSensorTriggered(boolean triggered) {
    //     this.intakeSensorTriggered = triggered;
    // }

    // public boolean isIntakeSensorTriggered() {
    //     return intakeSensorTriggered;
    // }

    // public void setIndexerSensorTriggered(boolean triggered) {
    //     this.indexerSensorTriggered = triggered;
    // }

    // public boolean isIndexerSensorTriggered() {
    //     return indexerSensorTriggered;
    // }

    // public void setShooterSensorTriggered(boolean triggered) {
    //     this.shooterSensorTriggered = triggered;
    // }

    // public boolean isShooterSensorTriggered() {
    //     return shooterSensorTriggered;
    // }

    // // ===================================================================================
    // // MECHANISM STATE (STUBS - add your mechanisms)
    // // Producer: Individual mechanism subsystems
    // // Consumers: Collision avoidance, targeting calculations, state machines
    // // ===================================================================================

    // // --- Arm Example ---
    // private Rotation2d armAngle = new Rotation2d();
    // private Rotation2d armGoalAngle = new Rotation2d();
    // private boolean armAtGoal = true;

    // public void setArmAngle(Rotation2d angle) {
    //     this.armAngle = angle;
    // }

    // public Rotation2d getArmAngle() {
    //     return armAngle;
    // }

    // public void setArmGoalAngle(Rotation2d angle) {
    //     this.armGoalAngle = angle;
    // }

    // public Rotation2d getArmGoalAngle() {
    //     return armGoalAngle;
    // }

    // public void setArmAtGoal(boolean atGoal) {
    //     this.armAtGoal = atGoal;
    // }

    // public boolean isArmAtGoal() {
    //     return armAtGoal;
    // }

    // // --- Elevator Example ---
    // private double elevatorHeightMeters = 0.0;
    // private double elevatorGoalMeters = 0.0;
    // private boolean elevatorAtGoal = true;

    // public void setElevatorHeight(double meters) {
    //     this.elevatorHeightMeters = meters;
    // }

    // public double getElevatorHeight() {
    //     return elevatorHeightMeters;
    // }

    // public void setElevatorGoal(double meters) {
    //     this.elevatorGoalMeters = meters;
    // }

    // public double getElevatorGoal() {
    //     return elevatorGoalMeters;
    // }

    // public void setElevatorAtGoal(boolean atGoal) {
    //     this.elevatorAtGoal = atGoal;
    // }

    // public boolean isElevatorAtGoal() {
    //     return elevatorAtGoal;
    // }

    // // --- Shooter Example ---
    // private double shooterVelocityRPM = 0.0;
    // private double shooterGoalRPM = 0.0;
    // private boolean shooterAtGoal = false;

    // public void setShooterVelocity(double rpm) {
    //     this.shooterVelocityRPM = rpm;
    // }

    // public double getShooterVelocity() {
    //     return shooterVelocityRPM;
    // }

    // public void setShooterGoal(double rpm) {
    //     this.shooterGoalRPM = rpm;
    // }

    // public double getShooterGoal() {
    //     return shooterGoalRPM;
    // }

    // public void setShooterAtGoal(boolean atGoal) {
    //     this.shooterAtGoal = atGoal;
    // }

    // public boolean isShooterAtGoal() {
    //     return shooterAtGoal;
    // }

    // // --- Climber Example ---
    // private boolean climberDeployed = false;
    // private double climberPositionMeters = 0.0;

    // public void setClimberDeployed(boolean deployed) {
    //     this.climberDeployed = deployed;
    // }

    // public boolean isClimberDeployed() {
    //     return climberDeployed;
    // }

    // public void setClimberPosition(double meters) {
    //     this.climberPositionMeters = meters;
    // }

    // public double getClimberPosition() {
    //     return climberPositionMeters;
    // }

    // // ===================================================================================
    // // SHOT READINESS (combines multiple subsystem states)
    // // Useful for triggers, LEDs, and driver feedback
    // // ===================================================================================

    // /** Check if all systems are ready to shoot */
    // public boolean isReadyToShoot() {
    //     return hasGamePiece()
    //             && shooterAtGoal
    //             && isAimedAtTarget(Rotation2d.fromDegrees(3.0))
    //             && !isMoving(); // Or remove this for shoot-on-move
    // }

    // /** Check if systems are ready but waiting for aim */
    // public boolean isWaitingForAim() {
    //     return hasGamePiece() && shooterAtGoal && !isAimedAtTarget(Rotation2d.fromDegrees(3.0));
    // }

    // // ===================================================================================
    // // ADVANTAGEKIT LOGGING
    // // Call this from Robot.robotPeriodic()
    // // ===================================================================================

    // public void logTelemetry() {
    //     // Drivetrain state
    //     Logger.recordOutput("RobotState/Drivetrain/CurrentPose", currentPose);
    //     Logger.recordOutput("RobotState/Drivetrain/OdometryPose", odometryPose);
    //     Logger.recordOutput("RobotState/Drivetrain/GyroHeading", gyroHeading.getDegrees());
    //     Logger.recordOutput("RobotState/Drivetrain/GyroPitch", gyroPitchDegrees);
    //     Logger.recordOutput("RobotState/Drivetrain/GyroRoll", gyroRollDegrees);
    //     Logger.recordOutput("RobotState/Drivetrain/LinearVelocity", getLinearVelocity());
    //     Logger.recordOutput("RobotState/Drivetrain/AngularVelocity", angularVelocityDegreesPerSec);
    //     Logger.recordOutput("RobotState/Drivetrain/IsMoving", isMoving);
    //     Logger.recordOutput("RobotState/Drivetrain/VisionOdometryDiff", getVisionOdometryDifference());

    //     // Vision state
    //     Logger.recordOutput("RobotState/Vision/UpdateCount", visionUpdateCount);
    //     Logger.recordOutput("RobotState/Vision/AcceptedCount", visionAcceptedCount);
    //     Logger.recordOutput("RobotState/Vision/RejectedCount", visionRejectedCount);
    //     peekVisionMeasurement().ifPresent(vm -> {
    //         Logger.recordOutput("RobotState/Vision/LatestPose", vm.pose());
    //         Logger.recordOutput("RobotState/Vision/TagCount", vm.tagCount());
    //         Logger.recordOutput("RobotState/Vision/AvgTagDistance", vm.avgTagDistance());
    //         Logger.recordOutput("RobotState/Vision/Source", vm.source());
    //     });

    //     // Targeting state
    //     Logger.recordOutput("RobotState/Targeting/HasActiveTarget", hasActiveTarget);
    //     Logger.recordOutput("RobotState/Targeting/TargetName", targetName);
    //     if (targetPosition != null) {
    //         Logger.recordOutput("RobotState/Targeting/TargetPosition",
    //             new Pose2d(targetPosition, new Rotation2d()));
    //         getDistanceToTarget().ifPresent(d ->
    //             Logger.recordOutput("RobotState/Targeting/Distance", d));
    //         getAngleToTarget().ifPresent(a ->
    //             Logger.recordOutput("RobotState/Targeting/FieldAngle", a.getDegrees()));
    //         getRotationToTarget().ifPresent(r ->
    //             Logger.recordOutput("RobotState/Targeting/RotationNeeded", r.getDegrees()));
    //     }

    //     // Alliance
    //     Logger.recordOutput("RobotState/Alliance/Color", alliance.name());
    //     Logger.recordOutput("RobotState/Alliance/Known", allianceKnown);

    //     // Game piece state
    //     Logger.recordOutput("RobotState/GamePiece/Held", heldGamePiece.name());
    //     Logger.recordOutput("RobotState/GamePiece/HasPiece", hasGamePiece());
    //     Logger.recordOutput("RobotState/GamePiece/IntakeSensor", intakeSensorTriggered);
    //     Logger.recordOutput("RobotState/GamePiece/IndexerSensor", indexerSensorTriggered);
    //     Logger.recordOutput("RobotState/GamePiece/ShooterSensor", shooterSensorTriggered);

    //     // Mechanisms
    //     Logger.recordOutput("RobotState/Arm/Angle", armAngle.getDegrees());
    //     Logger.recordOutput("RobotState/Arm/GoalAngle", armGoalAngle.getDegrees());
    //     Logger.recordOutput("RobotState/Arm/AtGoal", armAtGoal);

    //     Logger.recordOutput("RobotState/Elevator/Height", elevatorHeightMeters);
    //     Logger.recordOutput("RobotState/Elevator/Goal", elevatorGoalMeters);
    //     Logger.recordOutput("RobotState/Elevator/AtGoal", elevatorAtGoal);

    //     Logger.recordOutput("RobotState/Shooter/VelocityRPM", shooterVelocityRPM);
    //     Logger.recordOutput("RobotState/Shooter/GoalRPM", shooterGoalRPM);
    //     Logger.recordOutput("RobotState/Shooter/AtGoal", shooterAtGoal);

    //     Logger.recordOutput("RobotState/Climber/Deployed", climberDeployed);
    //     Logger.recordOutput("RobotState/Climber/Position", climberPositionMeters);

    //     // Composite states
    //     Logger.recordOutput("RobotState/ReadyToShoot", isReadyToShoot());
    //     Logger.recordOutput("RobotState/WaitingForAim", isWaitingForAim());
    // }
}

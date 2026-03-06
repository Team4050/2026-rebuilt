package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.util.LimelightHelpers;

@Logged(defaultNaming = Logged.Naming.USE_HUMAN_NAME)
public class RobotState {
  private static RobotState instance;

  @NotLogged
  private boolean needGameDataCheck = true;

  @Logged(name = "PDH")
  private final PowerDistribution pdh = new PowerDistribution();

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private RobotState() {
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
  }

  public void periodic() {
    updateDrivetrainPeriodic();
    updateMatchDataPeriodic();
    updateVisionPeriodic();
  }

  // ===================== Drivetrain =====================

  private Drivetrain drivetrain;

  @NotLogged
  private SwerveModuleState[] cachedModuleStates = new SwerveModuleState[] { new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState() };

  public void addDrivetrain(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    // Epilogue doesn't support logging complex objects, so add it as a Sendable instead
    SmartDashboard.putData("Field", drivetrain.getFieldPosition());
    SmartDashboard.putData("Swerve Drive", (SendableBuilder builder) -> {
      builder.setSmartDashboardType("SwerveDrive");
      builder.addDoubleProperty("Front Left Angle", () -> cachedModuleStates[0].angle.getRadians(), null);
      builder.addDoubleProperty("Front Left Velocity", () -> cachedModuleStates[0].speedMetersPerSecond, null);
      builder.addDoubleProperty("Front Right Angle", () -> cachedModuleStates[1].angle.getRadians(), null);
      builder.addDoubleProperty("Front Right Velocity", () -> cachedModuleStates[1].speedMetersPerSecond, null);
      builder.addDoubleProperty("Back Left Angle", () -> cachedModuleStates[2].angle.getRadians(), null);
      builder.addDoubleProperty("Back Left Velocity", () -> cachedModuleStates[2].speedMetersPerSecond, null);
      builder.addDoubleProperty("Back Right Angle", () -> cachedModuleStates[3].angle.getRadians(), null);
      builder.addDoubleProperty("Back Right Velocity", () -> cachedModuleStates[3].speedMetersPerSecond, null);
      builder.addDoubleProperty("Robot Angle", () -> drivetrain.getHeading().getRadians(), null);
    });
  }

  private void updateDrivetrainPeriodic() {
    if (drivetrain == null) {
      return;
    }
    cachedModuleStates = drivetrain.getModuleStates();
  }

  public Pose2d getChassisPose() {
    if (drivetrain == null) {
      return new Pose2d();
    }
    return drivetrain.getPose();
  }

  public ChassisSpeeds getChassisSpeeds() {
    if (drivetrain == null) {
      return new ChassisSpeeds();
    }
    return drivetrain.getSpeeds();
  }

  public SwerveModuleState[] getChassisModuleStates() {
    if (drivetrain == null) {
      return new SwerveModuleState[0];
    }
    return drivetrain.getModuleStates();
  }

  public SwerveModuleState[] getChassisModuleTargets() {
    if (drivetrain == null) {
      return new SwerveModuleState[0];
    }
    return drivetrain.getModuleTargets();
  }

  public Rotation2d getChassisHeading() {
    if (drivetrain == null) {
      return new Rotation2d();
    }
    return drivetrain.getHeading();
  }

  // ===================== Climber =====================

  private Climber climber;

  public void addClimber(Climber climber) {
    this.climber = climber;
  }

  public double getClimberPosition() {
    if (climber == null) {
      return 0.0;
    }
    return climber.getEncoderPosition();
  }

  // ===================== Intake =====================

  private Intake intake;

  public void addIntake(Intake intake) {
    this.intake = intake;
  }

  public double getIntakePosition() {
    if (intake == null) {
      return 0.0;
    }
    return intake.getPosition();
  }

  // ===================== Vision =====================

  private Pose2d visionPose = new Pose2d();

  @SuppressWarnings("unused") // Logged by Epilogue
  private int visionTagCount = 0;

  @SuppressWarnings("unused") // Logged by Epilogue
  private double visionAvgTagDist = 0;

  @SuppressWarnings("unused") // Logged by Epilogue
  private double visionLatencyMs = 0;

  @SuppressWarnings("unused") // Logged by Epilogue
  private boolean visionValid = false;

  private void updateVisionPeriodic() {
    if (!Constants.Vision.VISION_ENABLED || drivetrain == null) {
      return;
    }

    String ll = Constants.Vision.LIMELIGHT_NAME;

    // Provide gyro heading for MegaTag2 localization
    LimelightHelpers.SetRobotOrientation(ll, drivetrain.getHeading().getDegrees(), 0, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);

    if (!LimelightHelpers.validPoseEstimate(estimate)) {
      visionValid = false;
      return;
    }

    visionPose = estimate.pose;
    visionTagCount = estimate.tagCount;
    visionAvgTagDist = estimate.avgTagDist;
    visionLatencyMs = estimate.latency;
    visionValid = true;

    // Show vision estimate on field widget
    drivetrain.getFieldPosition().getObject("Vision").setPose(visionPose);

    // Scale standard deviations with distance — farther tags = less trustworthy
    double xyStdDev = 0.7 * estimate.avgTagDist;
    drivetrain
        .addVisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds,
            VecBuilder.fill(xyStdDev, xyStdDev, Double.MAX_VALUE));
  }

  // ===================== Match data =====================

  public enum MatchPeriod {
    DISABLED, AUTONOMOUS, TELEOP, ENDGAME, TEST
  }

  private static final double ALLIANCE_SHIFTS_START = 130.0; // after 10s transition
  private static final double SHIFT_DURATION = 25.0;

  private double matchRemainingTime = -1;

  @NotLogged
  private DriverStation.Alliance alliance;

  @NotLogged
  private DriverStation.Alliance firstAllianceInactive;

  public MatchPeriod getMatchPeriod() {
    if (DriverStation.isAutonomous()) {
      return MatchPeriod.AUTONOMOUS;
    } else if (DriverStation.isTeleop()) {
      return matchRemainingTime >= 0 && matchRemainingTime <= 30 ? MatchPeriod.ENDGAME : MatchPeriod.TELEOP;
    } else if (DriverStation.isTest()) {
      return MatchPeriod.TEST;
    }
    return MatchPeriod.DISABLED;
  }

  public boolean isOurScoringPeriod() {
    int shift = getAllianceShiftNumber();
    if (shift == 0 || firstAllianceInactive == null || alliance == null) {
      return false;
    }
    // Our alliance is inactive first → we score on even shifts (2, 4)
    // Otherwise → we score on odd shifts (1, 3)
    boolean evenShift = shift % 2 == 0;
    return (firstAllianceInactive == alliance) == evenShift;
  }

  @NotLogged
  private int getAllianceShiftNumber() {
    if (getMatchPeriod() != MatchPeriod.TELEOP) {
      return 0;
    }
    if (matchRemainingTime > ALLIANCE_SHIFTS_START) {
      return 0;
    }
    return (int) ((ALLIANCE_SHIFTS_START - matchRemainingTime) / SHIFT_DURATION) + 1;
  }

  private void updateMatchDataPeriodic() {
    if (alliance == null || DriverStation.isDisabled()) {
      alliance = DriverStation.getAlliance().orElse(null);
    }

    matchRemainingTime = DriverStation.getMatchTime();

    if (needGameDataCheck) {
      String gameData = DriverStation.getGameSpecificMessage();
      if (!gameData.isEmpty()) {
        firstAllianceInactive = switch (gameData.charAt(0)) {
          case 'B' -> DriverStation.Alliance.Blue;
          case 'R' -> DriverStation.Alliance.Red;
          default -> null;
        };
        needGameDataCheck = false;
      }
    }
  }
}

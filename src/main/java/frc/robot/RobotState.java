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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Climb;
import frc.robot.commands.Climb.ClimbStage;
import frc.robot.commands.Unload;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Unloader;
import frc.robot.subsystems.Intake.IntakeDeploy;
import frc.robot.subsystems.Intake.IntakeRollers;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Preset;

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
      instance.configurePresetChooser();
      instance.configurePresetChooser();
    }
    return instance;
  }

  private RobotState() {
    if (Constants.DEV_MODE) {
      SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    }
  }

  public void periodic() {
    updateDrivetrainPeriodic();
    updateMatchDataPeriodic();
    updateVisionPeriodic();
  }

  // ===================== Drivetrain =====================

  private Drivetrain drivetrain;

  private final String DRIVERTRAIN_ENABLE_KEY = "Drivetrain Enabled";
  private final boolean DEFAULT_DRIVETRAIN_ENABLE = true;

  @NotLogged
  private SwerveModuleState[] cachedModuleStates = new SwerveModuleState[] { new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState() };

  public void addDrivetrain(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    SmartDashboard.putBoolean(DRIVERTRAIN_ENABLE_KEY, DEFAULT_DRIVETRAIN_ENABLE);

    if (Constants.DEV_MODE) {
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
  }

  public boolean getDrivetrainEnable() {
    return SmartDashboard.getBoolean(DRIVERTRAIN_ENABLE_KEY, DEFAULT_DRIVETRAIN_ENABLE);
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
  private Climb climbCommand;

  private final String CLIMBER_ENABLE_KEY = "Climber Enabled";
  private final boolean DEFAULT_CLIMBER_ENABLE = true;

  public void addClimber(Climber climber, Climb climbCommand) {
    this.climber = climber;
    this.climbCommand = climbCommand;

    SmartDashboard.putBoolean(CLIMBER_ENABLE_KEY, DEFAULT_CLIMBER_ENABLE);
  }

  public boolean getClimberEnable() {
    return SmartDashboard.getBoolean(CLIMBER_ENABLE_KEY, DEFAULT_CLIMBER_ENABLE);
  }

  public double getClimberPosition() {
    if (climber == null) {
      return 0.0;
    }
    return climber.getEncoderPosition();
  }

  public double getClimberLeaderCurrent() {
    if (climber == null) {
      return 0.0;
    }
    return climber.getLeaderCurrent();
  }

  public double getClimberFollowerCurrent() {
    if (climber == null) {
      return 0.0;
    }
    return climber.getFollowerCurrent();
  }

  public boolean primaryAtUpperLimit() {
    return climber != null && climber.primaryAtUpperLimit();
  }

  public boolean primaryAtLowerLimit() {
    return climber != null && climber.primaryAtLowerLimit();
  }

  public Climb.ClimbStage getClimbStage() {
    if (climbCommand == null) {
      return ClimbStage.UNKNOWN;
    }
    return climbCommand.getClimbStage();
  }

  public int getNumLevelsClimbed() {
    if (climbCommand == null) {
      return 0;
    }
    return climbCommand.getNumLevelsClimbed();
  }

  // ===================== Intake =====================

  private IntakeDeploy intakeDeploy;

  private final String INTAKE_ENABLE_KEY = "Intake Enable";
  private final boolean DEFAULT_INTAKE_ENABLE = true;

  public void addIntakeDeploy(IntakeDeploy intakeDeploy) {
    this.intakeDeploy = intakeDeploy;

    SmartDashboard.putBoolean(INTAKE_ENABLE_KEY, DEFAULT_INTAKE_ENABLE);
  }

  public boolean getIntakeEnable() {
    return SmartDashboard.getBoolean(INTAKE_ENABLE_KEY, DEFAULT_INTAKE_ENABLE);
  }

  public double getIntakePosition() {
    if (intakeDeploy == null) {
      return 0.0;
    }
    return intakeDeploy.getPosition();
  }

  public double getIntakeDeployVelocity() {
    if (intakeDeploy == null) {
      return 0.0;
    }
    return intakeDeploy.getEncoderVelocity();
  }

  @SuppressWarnings("unused") // TODO: May be used in the future
  private IntakeRollers intakeRollers;

  public void addIntakeRollers(IntakeRollers intakeRollers) {
    this.intakeRollers = intakeRollers;
  }

  // ===================== Outtake =====================

  private Unloader unloaderLeft;
  private Unloader unloaderRight;
  private Unload unloadCommand;
  private final String OUTTAKE_ENABLE_KEY = "Intake Enable";
  private final boolean DEFAULT_OUTTAKE_ENABLE = true;

  public void addUnloaders(Unloader left, Unloader right, Unload command) {
    this.unloaderLeft = left;
    this.unloaderRight = right;
    this.unloadCommand = command;

    SmartDashboard.putBoolean(OUTTAKE_ENABLE_KEY, DEFAULT_OUTTAKE_ENABLE);
  }

  public boolean getOuttakeEnable() {
    return SmartDashboard.getBoolean(OUTTAKE_ENABLE_KEY, DEFAULT_OUTTAKE_ENABLE);
  }

  public boolean getShooterActive() {
    if (unloadCommand == null) {
      return false;
    }
    return unloadCommand.shooterIsActive();
  }

  public boolean getShooterAtSpeed() {
    if (unloaderLeft == null || unloaderRight == null) {
      return false;
    }
    return unloaderLeft.isAtSpeed() || unloaderRight.isAtSpeed();
  }

  // ===================== Vision =====================

  private Pose2d visionPose = new Pose2d();

  @SuppressWarnings("unused") // Logged by Epilogue
  private int visionTagCount = 0;

  @SuppressWarnings("unused") // Logged by Epilogue
  private double visionAvgTagDist = 0;

  @SuppressWarnings("unused") // Logged by Epilogue
  private boolean visionValid = false;

  private void updateVisionPeriodic() {
    if (!Constants.Vision.VISION_ENABLED || drivetrain == null) {
      return;
    }

    var ll = Constants.Vision.LIMELIGHT_NAME;

    LimelightHelpers.SetRobotOrientation(ll, drivetrain.getHeading().getDegrees(), 0, 0, 0, 0, 0);

    var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);

    if (!LimelightHelpers.validPoseEstimate(estimate)) {
      visionValid = false;
      return;
    }

    visionPose = estimate.pose;
    visionTagCount = estimate.tagCount;
    visionAvgTagDist = estimate.avgTagDist;
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

  public double matchRemainingTime = -1;

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

  // ================== Settings Presets ==================

  private SendableChooser<Preset> presetChooser = new SendableChooser<Preset>();

  private void applyPreset(Preset preset) {
    // if no preset is selected, assume custom settings
    // therefore, do not apply preset settings
    if (preset == null) {
      return;
    }

    SmartDashboard.putBoolean(DRIVERTRAIN_ENABLE_KEY, preset.drivetrainEnabled());
    SmartDashboard.putBoolean(CLIMBER_ENABLE_KEY, preset.climberEnabled());
    SmartDashboard.putBoolean(INTAKE_ENABLE_KEY, preset.intakeEnabled());
    // TODO SmartDashboard.putBoolean(OUTTAKE_ENABLE_KEY, preset.outtakeEnabled());
    // TODO SmartDashboard.putNumber(MAIN_SPEED_KEY, preset.mainSpeed());
    // TODO SmartDashboard.putNumber(SECONDARY_SPEED_KEY, preset.secondarySpeed());
    // TODO SmartDashboard.putNumber(ROTATIONAL_RATE_KEY, preset.rotationalRate());
  }

  // ====== Presets ======
  private void configurePresetChooser() {
    presetChooser.onChange(this::applyPreset);

    presetChooser.addOption("Competitive", Preset.competitive());
    // TODO add other presets

    SmartDashboard.putData("Presets", presetChooser);

  }
}

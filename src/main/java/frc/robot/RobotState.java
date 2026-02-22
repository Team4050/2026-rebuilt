package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import java.util.Set;

@Logged
public class RobotState {
    private static RobotState instance;

    @NotLogged
    private boolean needGameDataCheck = true;

    @SuppressWarnings("unused")
    private final PowerDistribution pdh = new PowerDistribution();

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private RobotState() {}

    public void periodic() {
        updateMatchDataPeriodic();
    }

    // ===================== Drivetrain =====================

    private Drivetrain drivetrain;

    public void addDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        SendableChooser<Command> sysIdChooser = drivetrain.buildSysIdChooser();
        SmartDashboard.putData("SysId Routine", sysIdChooser);
        SmartDashboard.putData(
                "Run SysId",
                Commands.defer(sysIdChooser::getSelected, Set.of(drivetrain)).withName("Run SysId"));
        SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

        // Epilogue doesn't support logging complex objects, so add it as a Sendable instead
        SmartDashboard.putData("Field", drivetrain.getFieldPosition());
        SmartDashboard.putData("Swerve Drive", (SendableBuilder builder) -> {
            builder.setSmartDashboardType("SwerveDrive");
            builder.addDoubleProperty(
                    "Front Left Angle", () -> drivetrain.getModuleStates()[0].angle.getRadians(), null);
            builder.addDoubleProperty(
                    "Front Left Velocity", () -> drivetrain.getModuleStates()[0].speedMetersPerSecond, null);
            builder.addDoubleProperty(
                    "Front Right Angle", () -> drivetrain.getModuleStates()[1].angle.getRadians(), null);
            builder.addDoubleProperty(
                    "Front Right Velocity", () -> drivetrain.getModuleStates()[1].speedMetersPerSecond, null);
            builder.addDoubleProperty(
                    "Back Left Angle", () -> drivetrain.getModuleStates()[2].angle.getRadians(), null);
            builder.addDoubleProperty(
                    "Back Left Velocity", () -> drivetrain.getModuleStates()[2].speedMetersPerSecond, null);
            builder.addDoubleProperty(
                    "Back Right Angle", () -> drivetrain.getModuleStates()[3].angle.getRadians(), null);
            builder.addDoubleProperty(
                    "Back Right Velocity", () -> drivetrain.getModuleStates()[3].speedMetersPerSecond, null);
            builder.addDoubleProperty(
                    "Robot Angle", () -> drivetrain.getHeading().getRadians(), null);
        });
    }

    @Logged
    public Pose2d getChassisPose() {
        return drivetrain.getPose();
    }

    @Logged
    public ChassisSpeeds getChassisSpeeds() {
        return drivetrain.getSpeeds();
    }

    @Logged
    public SwerveModuleState[] getChassisModuleStates() {
        return drivetrain.getModuleStates();
    }

    @Logged
    public SwerveModuleState[] getChassisModuleTargets() {
        return drivetrain.getModuleTargets();
    }

    @Logged
    public Rotation2d getChassisHeading() {
        return drivetrain.getHeading();
    }

    // ===================== Climber =====================

    private Climber climber;

    public void addClimber(Climber climber) {
        this.climber = climber;
    }

    @Logged
    public double getClimberPosition() {
        return climber.getEncoderPosition();
    }

    // ===================== Intake =====================

    private Intake intake;

    public void addIntake(Intake intake) {
        this.intake = intake;
    }

    @Logged
    public double getIntakeSpeed() {
        return intake.getSpeed();
    }

    // ===================== Match data =====================

    public enum MatchPeriod {
        DISABLED,
        AUTONOMOUS,
        TELEOP,
        ENDGAME,
        TEST
    }

    private static final double ALLIANCE_SHIFTS_START = 130.0; // after 10s transition
    private static final double SHIFT_DURATION = 25.0;

    private double matchRemainingTime = -1;

    @NotLogged
    private DriverStation.Alliance alliance;

    @NotLogged
    private DriverStation.Alliance firstAllianceInactive;

    @Logged
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

    @Logged
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
        if (alliance == null) {
            alliance = DriverStation.getAlliance().orElse(null);
        }

        matchRemainingTime = DriverStation.getMatchTime();

        if (needGameDataCheck) {
            String gameData = DriverStation.getGameSpecificMessage();
            if (gameData.length() > 0) {
                firstAllianceInactive = switch (gameData.charAt(0)) {
                    case 'B' -> DriverStation.Alliance.Blue;
                    case 'R' -> DriverStation.Alliance.Red;
                    default -> null;};
                needGameDataCheck = false;
            }
        }
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  private final double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Drivetrain drivetrain = TunerConstants.createDrivetrain();

  public final Intake intakeSub = new Intake();
  public final Climber climber = new Climber();

  private final CommandXboxController joystickPrimary = new CommandXboxController(0);
  private final CommandXboxController joystickSecondary = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    registerNamedCommands();
    initRobotState();
    configureBindings();
    autoChooser = buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void initRobotState() {
    RobotState rs = RobotState.getInstance();
    rs.addDrivetrain(drivetrain);
    rs.addIntake(intakeSub);
    rs.addClimber(climber);
  }

  private void configureBindings() {
    SendableChooser<Command> sysIdChooser = drivetrain.buildSysIdChooser();
    SmartDashboard.putData("SysId Routine", sysIdChooser);
    joystickPrimary
        .start()
        .and(joystickPrimary.back())
        .whileTrue(Commands.defer(sysIdChooser::getSelected, Set.of(drivetrain)).withName("Run SysId"));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers
        .disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true).withName("Idle Swerve Drive"));

    var teleopDrive = drivetrain
        .applyRequest(
            () -> drive
                .withVelocityX(-joystickPrimary.getLeftY() * MaxSpeed)
                .withVelocityY(-joystickPrimary.getLeftX() * MaxSpeed)
                .withRotationalRate(-joystickPrimary.getRightX() * MaxAngularRate))
        .withName("Teleop Swerve Drive");
    drivetrain.setDefaultCommand(teleopDrive);

    joystickPrimary.a().whileTrue(drivetrain.applyRequest(() -> brake).withName("Brake Swerve Drive"));
    var pointWheels = drivetrain
        .applyRequest(
            () -> point.withModuleDirection(new Rotation2d(-joystickPrimary.getLeftY(), -joystickPrimary.getLeftX())));
    joystickPrimary.b().whileTrue(pointWheels);

    // Reset the field-centric heading on left bumper press.
    joystickPrimary
        .leftBumper()
        .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).withName("Reset Field Centric Heading"));

    intakeSub.setDefaultCommand(new RunCommand(intakeSub::stop, intakeSub));
    joystickSecondary.leftBumper().toggleOnTrue(intakeSub.run(intakeSub::intakeForward));
    joystickSecondary.rightBumper().toggleOnTrue(intakeSub.run(intakeSub::intakeReverse));
    // joystickSecondary.y().toggleOnTrue(intakeSub.run(intakeSub::deployOut));
    // joystickSecondary.a().toggleOnTrue(intakeSub.run(intakeSub::deployIn));

    // climber.setDefaultCommand(new RunCommand(climber::stop, climber));
    joystickSecondary.povUp().whileTrue(new RunCommand(climber::up, climber));
    joystickSecondary.povDown().whileTrue(new RunCommand(climber::down, climber));
    joystickSecondary
        .povUp()
        .negate()
        .and(joystickSecondary.povDown().negate())
        .whileTrue(new RunCommand(climber::stop, climber));
    joystickSecondary.povLeft().onTrue(new RunCommand(() -> climber.setTargetPosition(0.0), climber));
    joystickSecondary.povRight().onTrue(new RunCommand(() -> climber.setTargetPosition(5.0), climber));
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("intake", intakeSub.run(intakeSub::intakeForward));
    NamedCommands.registerCommand("outtake", intakeSub.run(intakeSub::intakeReverse));
  }

  private SendableChooser<Command> buildAutoChooser() {
    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
    chooser.addOption("Drive 6ft Straight", drive6ftStraight());
    chooser.addOption("Drive 6ft, Turn 90, Return", drive6ftTurn90Return());
    chooser.addOption("Vision Navigate to Fixed Pose", visionNavigateToFixedPose());
    return chooser;
  }

  // ===================== Autonomous Routines =====================

  private static final PathConstraints slowConstraints = new PathConstraints(1.0, 1.0, Math.toRadians(540),
      Math.toRadians(720));

  private Command drive6ftStraight() {
    return Commands.defer(() -> {
      Pose2d currentPose = drivetrain.getPose();
      Rotation2d heading = currentPose.getRotation();
      // 6ft = 1.8288m forward from current pose
      Translation2d forward = new Translation2d(1.8288, heading);
      Pose2d endPose = new Pose2d(currentPose.getTranslation().plus(forward), heading);

      var waypoints = PathPlannerPath.waypointsFromPoses(currentPose, endPose);
      PathPlannerPath path = new PathPlannerPath(waypoints, slowConstraints, null, new GoalEndState(0.0, heading));
      path.preventFlipping = true;

      return AutoBuilder.followPath(path);
    }, Set.of(drivetrain)).withName("Drive 6ft Straight");
  }

  private Command drive6ftTurn90Return() {
    return Commands.defer(() -> {
      Pose2d startPose = drivetrain.getPose();
      Rotation2d heading = startPose.getRotation();
      Translation2d forward = new Translation2d(1.8288, heading);
      Pose2d midPose = new Pose2d(startPose.getTranslation().plus(forward), heading);

      // Path 1: drive 6ft forward
      var waypoints1 = PathPlannerPath.waypointsFromPoses(startPose, midPose);
      PathPlannerPath path1 = new PathPlannerPath(waypoints1, slowConstraints, null, new GoalEndState(0.0, heading));
      path1.preventFlipping = true;

      // Path 2: turn 90 degrees and drive back to start
      Rotation2d turnedHeading = heading.plus(Rotation2d.fromDegrees(90));
      Rotation2d returnTravelHeading = heading.plus(Rotation2d.fromDegrees(180));
      Pose2d midForReturn = new Pose2d(midPose.getTranslation(), returnTravelHeading);
      Pose2d endPose = new Pose2d(startPose.getTranslation(), returnTravelHeading);

      var waypoints2 = PathPlannerPath.waypointsFromPoses(midForReturn, endPose);
      PathPlannerPath path2 = new PathPlannerPath(waypoints2, slowConstraints, null,
          new GoalEndState(0.0, turnedHeading));
      path2.preventFlipping = true;

      return AutoBuilder.followPath(path1).andThen(AutoBuilder.followPath(path2));
    }, Set.of(drivetrain)).withName("Drive 6ft, Turn 90, Return");
  }

  private Command visionNavigateToFixedPose() {
    // Placeholder target: center field. Update coordinates for your actual target.
    Pose2d targetPose = new Pose2d(8.27, 4.01, Rotation2d.fromDegrees(0));
    PathConstraints constraints = new PathConstraints(2.0, 2.0, Math.toRadians(540), Math.toRadians(720));
    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0).withName("Vision Navigate to Fixed Pose");
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

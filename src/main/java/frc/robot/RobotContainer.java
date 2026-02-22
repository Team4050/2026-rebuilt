// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class RobotContainer {
    private final double MaxSpeed =
            0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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

    public RobotContainer() {
        initRobotState();
        configureBindings();
    }

    private void initRobotState() {
        RobotState rs = RobotState.getInstance();
        rs.addDrivetrain(drivetrain);
        rs.addIntake(intakeSub);
        rs.addClimber(climber);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive.withVelocityX(
                                        -joystickPrimary.getLeftY() * MaxSpeed) // Drive forward with negative Y
                                // (forward)
                                .withVelocityY(-joystickPrimary.getLeftX() * MaxSpeed) // Drive left with negative X
                                // (left)
                                .withRotationalRate(
                                        -joystickPrimary.getRightX() * MaxAngularRate) // Drive counterclockwise
                        // with negative X (left)
                        ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled()
                .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystickPrimary.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystickPrimary
                .b()
                .whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(
                        new Rotation2d(-joystickPrimary.getLeftY(), -joystickPrimary.getLeftX()))));

        // Reset the field-centric heading on left bumper press.
        joystickPrimary.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

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

    public Command getAutonomousCommand() {
        // TODO: Autonomous code
        return null;
    }
}

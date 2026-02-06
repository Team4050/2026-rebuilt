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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

public class RobotContainer {
    // TODO: Relocate constants to dedicated constants file
    private double MaxSpeed =
            0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandXboxController joystickPrimary = new CommandXboxController(0);
    private final CommandXboxController joystickSecondary = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intakeSub = new Intake();

    public final Climber climber = new Climber();

    public RobotContainer() {
        configureBindings();
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

        climber.setDefaultCommand(new RunCommand(climber::stop, climber));

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

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystickPrimary.back().and(joystickPrimary.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystickPrimary.back().and(joystickPrimary.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystickPrimary.start().and(joystickPrimary.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystickPrimary.start().and(joystickPrimary.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystickPrimary.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // RUN INTAKE COMMANDS
        // TODO: RUMBLE WHEN INTAKE swithcing directions
        joystickSecondary.leftBumper().toggleOnTrue(intakeSub.run(intakeSub::runIntake));
        joystickSecondary.rightBumper().toggleOnTrue(intakeSub.run(intakeSub::reverseIntake));

        intakeSub.setDefaultCommand(new RunCommand(intakeSub::stopIntake, intakeSub));
        joystickPrimary.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystickPrimary.povUp().whileTrue(new RunCommand(() -> climber.setSpeed(1.0), climber));
        joystickPrimary.povDown().whileTrue(new RunCommand(() -> climber.setSpeed(-1.0), climber));
    }

    public Command getAutonomousCommand() {
        // TODO: Autonomous code

        // Simple drive forward auton example
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //         // Reset our field centric heading to match the robot
        //         // facing away from our alliance station wall (0 deg).
        //         drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //         // Then slowly drive forward (away from us) for 5 seconds.
        //         drivetrain
        //                 .applyRequest(
        //                         () -> drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
        //                 .withTimeout(5.0),
        //         // Finally idle for the rest of auton
        //         drivetrain.applyRequest(() -> idle));
        return null;
    }
}

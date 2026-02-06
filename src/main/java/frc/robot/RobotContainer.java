// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

public class RobotContainer {
    private final Drive driveSubsystem;

    // // TODO: Relocate constants to dedicated constants file
    // private double MaxSpeed =
    //         0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate =
    //         RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // /* Setting up bindings for necessary control of the swerve drive platform */
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         .withDeadband(MaxSpeed * 0.1)
    //         .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandXboxController primaryController = new CommandXboxController(0);

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                driveSubsystem = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                break;

            case SIM:
                driveSubsystem = new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
                break;

            default:
                driveSubsystem = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                break;
        }

        configureBindings();
    }

    private void configureBindings() {
        driveSubsystem.setDefaultCommand(DriveCommands.joystickDrive(
                driveSubsystem,
                () -> -primaryController.getLeftY(),
                () -> -primaryController.getLeftX(),
                () -> -primaryController.getRightX()));

        // Lock to 0° when A button is held
        primaryController
                .a()
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        driveSubsystem,
                        () -> -primaryController.getLeftY(),
                        () -> -primaryController.getLeftX(),
                        () -> Rotation2d.kZero));

        // Switch to X pattern when X button is pressed
        primaryController.x().onTrue(Commands.runOnce(driveSubsystem::stopWithX, driveSubsystem));

        // Reset gyro to 0° when B button is pressed
        primaryController
                .b()
                .onTrue(Commands.runOnce(
                                () -> driveSubsystem.setPose(
                                        new Pose2d(driveSubsystem.getPose().getTranslation(), Rotation2d.kZero)),
                                driveSubsystem)
                        .ignoringDisable(true));
    }

    public Command getAutonomousCommand() {
        // TODO: Autonomous code
        return null;
    }
}

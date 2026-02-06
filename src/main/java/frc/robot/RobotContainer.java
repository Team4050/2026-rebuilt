// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

public class RobotContainer {
    private final Drive driveSubsystem;
    public final Intake intakeSub = new Intake();
    public final Climber climber = new Climber();

    private final CommandXboxController joystickPrimary = new CommandXboxController(0);
    private final CommandXboxController joystickSecondary = new CommandXboxController(1);

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
        // Drivetrain
        driveSubsystem.setDefaultCommand(DriveCommands.joystickDrive(
                driveSubsystem,
                () -> -joystickPrimary.getLeftY(),
                () -> -joystickPrimary.getLeftX(),
                () -> -joystickPrimary.getRightX()));

        // Lock to 0° when A button is held
        joystickPrimary
                .a()
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        driveSubsystem,
                        () -> -joystickPrimary.getLeftY(),
                        () -> -joystickPrimary.getLeftX(),
                        () -> Rotation2d.kZero));

        // Switch to X pattern when X button is pressed
        joystickPrimary.x().onTrue(Commands.runOnce(driveSubsystem::stopWithX, driveSubsystem));

        // Reset gyro to 0° when B button is pressed
        joystickPrimary
                .b()
                .onTrue(Commands.runOnce(
                                () -> driveSubsystem.setPose(
                                        new Pose2d(driveSubsystem.getPose().getTranslation(), Rotation2d.kZero)),
                                driveSubsystem)
                        .ignoringDisable(true));

        climber.setDefaultCommand(new RunCommand(climber::stop, climber));
        joystickSecondary.povUp().onTrue(climber.runOnce(climber::up));
        joystickSecondary.povDown().whileTrue(climber.runOnce(climber::down));

        intakeSub.setDefaultCommand(new RunCommand(intakeSub::stop, intakeSub));
        joystickSecondary.leftBumper().toggleOnTrue(intakeSub.run(intakeSub::intakeForward));
        joystickSecondary.rightBumper().toggleOnTrue(intakeSub.run(intakeSub::intakeReverse));
        joystickSecondary.y().toggleOnTrue(intakeSub.run(intakeSub::deployOut));
        joystickSecondary.a().toggleOnTrue(intakeSub.run(intakeSub::deployIn));
    }

    public Command getAutonomousCommand() {
        // TODO: Autonomous code
        return null;
    }
}

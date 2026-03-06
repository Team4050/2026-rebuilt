// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

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
  private final Drivetrain drivetrain = TunerConstants.createDrivetrain();

  public final Intake intake = new Intake();
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
    rs.addIntake(intake);
    rs.addClimber(climber);
  }

  private void configureBindings() {
    configureDefaultCommands();
    configureRobotTriggers();
    configurePrimaryBindings();
    configureSecondaryBindings();

    // Diagnostic commands
    SendableChooser<Command> sysIdChooser = drivetrain.buildSysIdChooser();
    SmartDashboard.putData("SysId Routine", sysIdChooser);
    joystickPrimary
        .povUp()
        .whileTrue(Commands.defer(sysIdChooser::getSelected, Set.of(drivetrain)).withName("DT: Run SysId"));

    SmartDashboard.putData("Climber: Home", climber.homeCommand());
  }

  private void configureDefaultCommands() {
    intake.setDefaultCommand(intake.stopCommand());
    climber.setDefaultCommand(climber.stopCommand());
  }

  private void configurePrimaryBindings() {
    // ===== Driving =====

    var maxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    var maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    var fieldCentricSwerveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * 0.1)
        .withRotationalDeadband(maxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    var robotCentricSwerveRequest = new SwerveRequest.RobotCentric()
        .withDeadband(maxSpeed * 0.1)
        .withRotationalDeadband(maxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    var brakeDriveRequest = new SwerveRequest.SwerveDriveBrake();

    var isRobotCentric = new AtomicBoolean(false);

    drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
      double vx = -joystickPrimary.getLeftY() * maxSpeed;
      double vy = -joystickPrimary.getLeftX() * maxSpeed;
      double rot = -joystickPrimary.getRightX() * maxAngularRate;
      if (isRobotCentric.get()) {
        return robotCentricSwerveRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(rot);
      }
      return fieldCentricSwerveRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(rot);
    }).withName("DT: Drive"));

    // Apply Brakes while holding X
    joystickPrimary.x().whileTrue(drivetrain.applyRequest(() -> brakeDriveRequest).withName("DT: Brake"));

    // Reset the field-centric heading on Y button press.
    joystickPrimary
        .y()
        .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).withName("DT: Reset Field Centric Heading"));

    // Toggle to robot-relative driving on back button
    joystickPrimary
        .back()
        .onTrue(drivetrain.runOnce(() -> isRobotCentric.set(true)).withName("DT: Toggle Robot Centric"));

    // Toggle to field-relative driving on start button
    joystickPrimary
        .start()
        .onTrue(drivetrain.runOnce(() -> isRobotCentric.set(false)).withName("DT: Toggle Field Centric"));

    // ===== Misc =====

    joystickPrimary.leftTrigger().whileTrue(new RunCommand(intake::intakeReverse, intake).withName("Intake: Reverse"));
    joystickPrimary.rightTrigger().whileTrue(new RunCommand(intake::intakeForward, intake).withName("Intake: Forward"));

  }

  private void configureSecondaryBindings() {
    // TODO: Technically up and down are misleading. We should consider different names
    joystickSecondary.leftTrigger().whileTrue(new RunCommand(climber::up, climber).withName("Climber: Up"));
    joystickSecondary.rightTrigger().whileTrue(new RunCommand(climber::down, climber).withName("Climber: Down"));
    // TODO: Climber deploy
    // joystickSecondary.povUp()
    // joystickSecondary.povDown()

    joystickSecondary.leftBumper().toggleOnTrue(intake.run(intake::intakeForward));
    joystickSecondary.rightBumper().toggleOnTrue(intake.run(intake::intakeReverse));
    joystickSecondary.x().toggleOnTrue(intake.run(intake::deployOut));
    joystickSecondary.b().toggleOnTrue(intake.run(intake::deployIn));

    joystickSecondary.a().whileTrue(intake.run(intake::deployOverrideOut));
    joystickSecondary.y().whileTrue(intake.run(intake::deployOverrideIn));
  }

  private void configureRobotTriggers() {
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    RobotModeTriggers
        .disabled()
        .whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true).withName("DT: Idle"));
  }

  public Command getAutonomousCommand() {
    // TODO: Autonomous code
    return null;
  }
}

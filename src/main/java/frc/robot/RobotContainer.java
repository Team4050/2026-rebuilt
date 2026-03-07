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
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.Unload;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Unloader;
import frc.robot.subsystems.Intake.IntakeDeploy;
import frc.robot.subsystems.Intake.IntakeRollers;

public class RobotContainer {
  private final Drivetrain drivetrain = TunerConstants.createDrivetrain();

  public final IntakeRollers intakeRollers = new IntakeRollers();
  public final IntakeDeploy intakeDeploy = new IntakeDeploy();

  public final Climber climber = new Climber();

  public final Unloader unloaderLeft = new Unloader(Constants.Subsystems.kickerLeftId, false,
      Constants.Subsystems.shooterLeftId, false);

  public final Unloader unloaderRight = new Unloader(Constants.Subsystems.kickerRightId, true);

  public final Unload unloadCommand = new Unload(unloaderLeft, unloaderRight);

  private final CommandXboxController joystickPrimary = new CommandXboxController(0);
  private final CommandXboxController joystickSecondary = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    initRobotState();
    configureBindings();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void initRobotState() {
    RobotState rs = RobotState.getInstance();
    rs.addDrivetrain(drivetrain);
    rs.addIntakeDeploy(intakeDeploy);
    rs.addIntakeRollers(intakeRollers);
    rs.addClimber(climber);
    rs.addUnloaders(unloaderLeft, unloaderRight);
  }

  private void configureBindings() {
    configureRobotTriggers();
    configurePrimaryBindings();
    configureSecondaryBindings();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // ===== Diagnostic commands =====

    SendableChooser<Command> sysIdChooser = drivetrain.buildSysIdChooser();
    SmartDashboard.putData("SysId Routine", sysIdChooser);
    joystickPrimary
        .povUp()
        .whileTrue(Commands.defer(sysIdChooser::getSelected, Set.of(drivetrain)).withName("DT: Run SysId"));

    SmartDashboard.putData("Climber: Home", climber.homeCommand());
  }

  private void configurePrimaryBindings() {
    // ===== Driving =====

    var maxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    var crawlSpeed = 0.15 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
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
      var speed = joystickPrimary.getHID().getRightBumperButton() ? crawlSpeed : maxSpeed;
      var vx = -joystickPrimary.getLeftY() * speed;
      var vy = -joystickPrimary.getLeftX() * speed;
      var rot = -joystickPrimary.getRightX() * maxAngularRate;
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

    // ===== Intake =====

    joystickPrimary.leftTrigger().whileTrue(intakeRollers.outCommand());
    joystickPrimary.rightTrigger().whileTrue(intakeRollers.inCommand());
  }

  private void configureSecondaryBindings() {
    // ===== Intake =====

    // X: Toggle intake deploy (press to deploy, press again to retract)
    joystickSecondary.x().onTrue(intakeDeploy.toggleDeployCommand());

    // B: Run intake rollers while held
    joystickSecondary.b().whileTrue(intakeRollers.inCommand());

    // ===== Unloaders =====

    joystickSecondary.leftTrigger().whileTrue(unloadCommand.outtakeCommand());

    joystickSecondary.rightBumper().toggleOnTrue(unloadCommand.primeCommand());
    joystickSecondary.rightTrigger().whileTrue(unloadCommand.shootCommand());

    // ===== Climber =====

    // PovUp (hold): Climb up
    joystickSecondary.povUp().whileTrue(climber.upCommand());

    // PovDown (hold): Climb down
    joystickSecondary.povDown().whileTrue(climber.downCommand());

    // TODO: implement climber deploy/retract toggle
    // PovRight (tap): Toggle climber deploy / retract
    joystickSecondary
        .povRight()
        .onTrue(Commands.print("[STUB] Climber: Toggle Deploy").withName("Climber: Toggle Deploy"));

    // ===== Overrides =====

    joystickSecondary.start().whileTrue(intakeDeploy.deployOverrideCommand(false));
    joystickSecondary.back().whileTrue(intakeDeploy.deployOverrideCommand(true));
  }

  private void configureRobotTriggers() {
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    RobotModeTriggers
        .disabled()
        .whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true).withName("DT: Idle"));
  }

  // ===================== Autonomous Routines =====================
  private SendableChooser<Command> buildAutoChooser() {
    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
    return chooser;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

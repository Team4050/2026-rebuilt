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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeDeploy;
import frc.robot.subsystems.IntakeRollers;

public class RobotContainer {
  private final Drivetrain drivetrain = TunerConstants.createDrivetrain();

  public final IntakeRollers intakeRollers = new IntakeRollers();
  public final IntakeDeploy intakeDeploy = new IntakeDeploy();
  // public final Climber climber = new Climber();

  private final CommandXboxController joystickPrimary = new CommandXboxController(0);
  private final CommandXboxController joystickSecondary = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    initRobotState();
    configureBindings();
    autoChooser = buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void initRobotState() {
    RobotState rs = RobotState.getInstance();
    rs.addDrivetrain(drivetrain);
    rs.addIntakeDeploy(intakeDeploy);
    rs.addIntakeRollers(intakeRollers);
    // rs.addClimber(climber);
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

    // SmartDashboard.putData("Climber: Home", climber.homeCommand());
  }

  private void configureDefaultCommands() {
    intakeRollers.setDefaultCommand(intakeRollers.stopCommand());
    // climber.setDefaultCommand(climber.stopCommand());
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
      var vx = -joystickPrimary.getLeftY() * maxSpeed;
      var vy = -joystickPrimary.getLeftX() * maxSpeed;
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

    // ===== Mechanisms =====

    joystickPrimary
        .leftTrigger()
        .whileTrue(new RunCommand(intakeRollers::intakeReverse, intakeRollers).withName("Intake: Reverse"));
    joystickPrimary
        .rightTrigger()
        .whileTrue(new RunCommand(intakeRollers::intakeForward, intakeRollers).withName("Intake: Forward"));
  }

  private void configureSecondaryBindings() {
    // TODO: Technically up and down are misleading. We should consider different names
    // joystickSecondary.leftBumper().whileTrue(new RunCommand(climber::up, climber).withName("Climber: Up"));
    // joystickSecondary.rightBumper().whileTrue(new RunCommand(climber::down, climber).withName("Climber: Down"));
    // TODO: Climber deploy
    // joystickSecondary.povUp()
    // joystickSecondary.povDown()

    joystickSecondary.leftTrigger().whileTrue(intakeRollers.run(intakeRollers::intakeReverse));
    joystickSecondary.rightTrigger().whileTrue(intakeRollers.run(intakeRollers::intakeForward));

    joystickSecondary.x().onTrue(intakeDeploy.runOnce(intakeDeploy::deploy));
    joystickSecondary.b().onTrue(intakeDeploy.runOnce(intakeDeploy::retract));

    // TODO: If it can be avoided, we should not have override commands on the secondary driver's joystick
    joystickSecondary.a().whileTrue(intakeDeploy.deployOverrideCommand(false));
    joystickSecondary.y().whileTrue(intakeDeploy.deployOverrideCommand(true));
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

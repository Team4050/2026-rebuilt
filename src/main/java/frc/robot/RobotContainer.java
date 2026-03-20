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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.Climb;
import frc.robot.commands.Unload;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Agitate;
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
  public final Climb climbCommand = new Climb(climber);

  public final Unloader unloaderLeft = new Unloader(Constants.Subsystems.kickerLeftId, false,
      Constants.Subsystems.shooterLeftId, false);

  public final Unloader unloaderRight = new Unloader(Constants.Subsystems.kickerRightId, true,
      Constants.Subsystems.shooterRightId, true);

  public final Agitate agitate = new Agitate();
  public final Unload unloadCommand = new Unload(unloaderLeft, unloaderRight, agitate);

  private final CommandXboxController joystickPrimary = new CommandXboxController(0);
  private final CommandXboxController joystickSecondary = new CommandXboxController(1);

  public RobotContainer() {
    initRobotState();
    configureBindings();
  }

  private void initRobotState() {
    RobotState rs = RobotState.getInstance();
    rs.addDrivetrain(drivetrain);
    rs.addIntakeDeploy(intakeDeploy);
    rs.addIntakeRollers(intakeRollers);
    rs.addClimber(climber, climbCommand);
    rs.addUnloaders(unloaderLeft, unloaderRight, unloadCommand);
  }

  private void configureBindings() {
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

  private void configurePrimaryBindings() {
    // ===== Driving =====

    var theoreticalMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    var crawlSpeed = 0.15 * theoreticalMaxSpeed;
    var theoreticalMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    var fieldCentricSwerveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(theoreticalMaxSpeed * 0.05)
        .withRotationalDeadband(theoreticalMaxAngularRate * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    var robotCentricSwerveRequest = new SwerveRequest.RobotCentric()
        .withDeadband(theoreticalMaxSpeed * 0.05)
        .withRotationalDeadband(theoreticalMaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    var brakeDriveRequest = new SwerveRequest.SwerveDriveBrake();

    var isRobotCentric = new AtomicBoolean(false);

    drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
      var maxSpeed = drivetrain.getSpeedMultiplier() * theoreticalMaxSpeed;
      var maxAngularRate = drivetrain.getRotSpeedMultiplier() * theoreticalMaxAngularRate;
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

    //Command agitateandShoot = new ParallelCommandGroup(unloadCommand.primeCommand(), agitate.agitateCommand());

    // ===== Unloaders =====

    joystickSecondary.leftBumper().whileTrue(agitate.agitateCommand());

    joystickSecondary.rightBumper().toggleOnTrue(unloadCommand.primeCommand());
    joystickSecondary.rightTrigger().whileTrue(unloadCommand.shootCommand());

    // ===== Climber =====

    // PovUp (hold): Climb up
    joystickSecondary.povUp().whileTrue(climber.overridePrimaryUpCommand().withName("Climber: Override Primary Up"));

    // PovDown (hold): Climb down
    joystickSecondary
        .povDown()
        .whileTrue(climber.overridePrimaryDownCommand().withName("Climber: Override Primary Down"));

    // ===== Overrides =====

    // Start (hold): Override intake deploy - ignores soft limits
    joystickSecondary.start().whileTrue(intakeDeploy.deployOverrideNoLimitsCommand(false));

    // Back (hold): Override intake deploy - ignores soft limits
    joystickSecondary.back().whileTrue(intakeDeploy.deployOverrideNoLimitsCommand(true));
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

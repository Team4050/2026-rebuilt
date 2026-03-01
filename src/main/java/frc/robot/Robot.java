// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.revrobotics.util.StatusLogger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Elastic;
import frc.robot.util.LimelightHelpers;

@Logged(defaultNaming = Logged.Naming.USE_HUMAN_NAME)
public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final RobotState robotState = RobotState.getInstance();

  private final RobotContainer robotContainer;

  public Robot() {
    CameraServer.startAutomaticCapture();

    // Serve Elastic dashboard config file
    // https://frc-elastic.gitbook.io/docs/additional-features-and-references/remote-layout-downloading#on-robot-configuration
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // Default dashboard to diagnostics tab when not on main or event branch
    if (Constants.DEV_MODE) {
      Elastic.selectTab("Diagnostics");
    }

    configureLogging();
    configureLimeLight();

    // Warm up PathPlanner to avoid first-run lag during auto
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    robotContainer = new RobotContainer();
  }

  private void configureLimeLight() {
    // Set IMU mode to mode 2, to use pose data from drivetrain IMU (using SetRobotOrientation)
    LimelightHelpers.SetIMUMode(Constants.Vision.LIMELIGHT_NAME, 2);

    // All pose measurements are in the robot's coordinate space, with the origin at
    // the center of the robot, +X forward, +Y left, and +Z up. Height is measured from the ground.
    LimelightHelpers
        .setCameraPose_RobotSpace(
            Constants.Vision.LIMELIGHT_NAME,
            // Forward: Negative since LL is on back of robot
            Units.inchesToMeters(-8.25),
            // Side: LL is centered
            Units.inchesToMeters(0),
            // Up
            Units.inchesToMeters(8),
            // Roll
            180,
            // Pitch: LL is tilted up slightly
            15,
            // Yaw: LL is pointed backwards, so 180 degrees
            180);
  }

  private void configureLogging() {
    // Disable Rev logs
    StatusLogger.disableAutoLogging();

    // Silence joystick warnings in development (overridden when connected to FMS)
    DriverStation.silenceJoystickConnectionWarning(true);

    // Suppress .hoot file generation from CTRE SignalLogger - we will do our own logging
    SignalLogger.enableAutoLogging(false);

    // Start data logging — mirrors all NetworkTables to .wpilog on disk
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    Epilogue.configure(config -> {
      if (isSimulation()) {
        config.errorHandler = ErrorHandler.crashOnError();
      }
      config.minimumImportance = Logged.Importance.DEBUG;
    });
    Epilogue.bind(this);
  }

  @Override
  public void robotPeriodic() {
    robotState.periodic();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}

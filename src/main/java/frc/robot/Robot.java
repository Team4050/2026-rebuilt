// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.MatchData;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;
    private final MatchData matchData = new MatchData();

    public Robot() {
        // Silence joystick warnings in development (overridden when connected to FMS)
        DriverStation.silenceJoystickConnectionWarning(true);

        // Serve Elastic dashboard config file
        // https://frc-elastic.gitbook.io/docs/additional-features-and-references/remote-layout-downloading#on-robot-configuration
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        robotContainer = new RobotContainer();

        matchData.init();
    }

    @Override
    public void robotPeriodic() {
        matchData.periodic();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}

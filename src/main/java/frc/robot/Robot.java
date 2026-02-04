// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.MatchData;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer robotContainer;

    private final MatchData matchData = new MatchData();

    private final SparkMax intake = new SparkMax(5, SparkMax.MotorType.kBrushless);

    private final Joystick intakeJoystick = new Joystick(1);

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
        m_autonomousCommand = robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {

        // intake test
        if (intakeJoystick.getRawButton(4)) {
            intake.set(1);
            System.out.println("intake forward");
        } else if (intakeJoystick.getRawButton(1)) {
            intake.set(-1);
            System.out.println("intake reverse");
        } else {
            intake.set(0.0);
        }
    }

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

    @Override
    public void simulationPeriodic() {}
}

package frc.robot;

import frc.robot.generated.BuildConstants;

public final class Constants {

  public static final boolean DEV_MODE = !BuildConstants.GIT_BRANCH.equals("main")
      && !BuildConstants.GIT_BRANCH.startsWith("event/");

  public static final String canivoreCanBusName = "Drivetrain";

  public static final int pdhId = 1;

  public final class Drivetrain {
    public static final int pigeonId = 10;

    // =========================================================================================================================
    // CAN device IDs - Schema is <module number: 1-4><device: drive = 1, steer = 2, encoder = 3>
    // =========================================================================================================================

    // Front left - Module 1
    public static final int frontLeftDriveId = 11;
    public static final int frontLeftSteerId = 12;
    public static final int frontLeftEncoderId = 13;

    // Front right - Module 4
    public static final int frontRightDriveId = 41;
    public static final int frontRightSteerId = 42;
    public static final int frontRightEncoderId = 43;

    // Back left - Module 2
    public static final int backLeftDriveId = 21;
    public static final int backLeftSteerId = 22;
    public static final int backLeftEncoderId = 23;

    // Back right - Module 3
    public static final int backRightDriveId = 31;
    public static final int backRightSteerId = 32;
    public static final int backRightEncoderId = 33;

    // Speed Settings
    public static final String MAIN_SPEED_KEY = "Main Drive Speed";
    public static final String SECONDARY_SPEED_KEY = "Secondary Drive Speed";
    public static final String ROTATIONAL_RATE_KEY = "Drive Rotational Rate";
  }

  public final class Subsystems {
    public static final int intakeRollerId = 51;
    public static final int intakeDeployId = 52;

    public static final int climberPrimaryId = 53;
    public static final int climberFollowerId = 54;
    public static final int climberDeployId = 55; // Unused / Reserved for future use

    public static final int kickerLeftId = 56;
    public static final int shooterLeftId = 57;

    public static final int kickerRightId = 58;
    public static final int shooterRightId = 59;

    public static final int agitateId = 60;
  }

  public final class Unloader {
    // Unloader is designed to support N unloaders, but having a fixed SmartDashboard
    // key for shooter speed means that all shooters will run at the same speed.
    // TODO: We should do this differently someday.
    public static final String SHOOTER_SPEED_KEY = "Shooter Speed";
  }

  public final class Vision {
    public static final boolean VISION_ENABLED = false;
    public static final String LIMELIGHT_NAME = "limelight";
  }
}

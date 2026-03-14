package frc.robot;

import edu.wpi.first.math.util.Units;
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
  }

  public final class Vision {
    public static final boolean VISION_ENABLED = true;

    public static final String LIMELIGHT_NAME = "limelight";

    public static final int IMU_MODE = 2; // Use drivetrain IMU for pose estimation

    // Camera position relative to robot center (meters and degrees).
    // Measure from the robot center (floor level, between the 4 swerve modules)
    // to the camera lens center.
    // +X = forward, +Y = left, +Z = up
    public static final double CAMERA_FORWARD = -0.36065;
    public static final double CAMERA_SIDE = 0.17534;
    public static final double CAMERA_UP = 0.51911;
    public static final double CAMERA_ROLL = 0;
    public static final double CAMERA_PITCH = 10; // Camera is angled slightly upwards
    public static final double CAMERA_YAW = 190; // Camera is facing backwards
  }

  public final class Targeting {
    public static final double ROTATION_KP = 0.03;
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.001;

    public static final double FORWARD_KP = 1.0;
    public static final double FORWARD_KI = 0.0;
    public static final double FORWARD_KD = 0.05;

    public static final double STRAFE_KP = 1.0;
    public static final double STRAFE_KI = 0.0;
    public static final double STRAFE_KD = 0.05;

    public static final double MAX_DRIVE_SPEED = 0.5;
    public static final double MAX_ROTATION_SPEED = 0.5;
  }

  public final class Tower {
    // Two AprilTags per tower wall, per alliance (one centered, one offset).
    // Verify which ID is centered vs offset on your field — set the centered one as primary.
    public static final int RED_PRIMARY_TAG_ID = 15;
    public static final int RED_SECONDARY_TAG_ID = 16;
    public static final int BLUE_PRIMARY_TAG_ID = 31;
    public static final int BLUE_SECONDARY_TAG_ID = 32;

    // Target offsets from tag (meters)
    public static final double TARGET_DISTANCE = Units.inchesToMeters(55);
    public static final double TARGET_LATERAL_OFFSET = 0.0;

    // Tolerances
    public static final double ROTATION_TOLERANCE_DEG = 1.5;
    public static final double POSITION_TOLERANCE_M = 0.03;

    // Timeout for auto alignment commands (seconds)
    public static final double ALIGN_TIMEOUT_SEC = 5.0;

    // Approach pose for autonomous pathfinding — update with real coordinates
    public static final double APPROACH_X = 4.0;
    public static final double APPROACH_Y = 4.0;
    public static final double APPROACH_HEADING_DEG = 0.0;
  }

  public final class Home {
    public static final int PRIMARY_TAG_ID = 30;
    public static final int SECONDARY_TAG_ID = 29;

    public static final double TARGET_DISTANCE = Units.inchesToMeters(48);
    public static final double TARGET_LATERAL_OFFSET = Units.inchesToMeters(12);
  }
}

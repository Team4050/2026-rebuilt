package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionMeasurement;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

/**
 * Vision subsystem that processes Limelight pose estimates and publishes
 * them to RobotState for consumption by the drivetrain.
 *
 * This subsystem is a PRODUCER of vision data - it doesn't apply the data
 * itself, it just validates and publishes it.
 */
public class VisionSubsystem extends SubsystemBase {
    private final RobotState robotState = RobotState.getInstance();
    private final String limelightName;

    // Filtering configuration
    private static final int MIN_TAG_COUNT = 1;
    private static final double MAX_TAG_DISTANCE = 4.0; // meters
    private static final double MAX_ANGULAR_VELOCITY_FOR_VISION = 120.0; // deg/s

    // Field dimensions (in meters) for bounds checking
    private static final double FIELD_LENGTH = 16.54;
    private static final double FIELD_WIDTH = 8.07;

    // Debug/tuning
    private boolean visionEnabled = true;
    private int consecutiveRejects = 0;
    private String lastRejectReason = "";

    /**
     * Create a VisionSubsystem with a specific Limelight name.
     * @param limelightName NetworkTables name of the Limelight (e.g., "limelight", "limelight-front")
     */
    public VisionSubsystem(String limelightName) {
        this.limelightName = limelightName;
    }

    /**
     * Create a VisionSubsystem using the default "limelight" name.
     */
    public VisionSubsystem() {
        this("limelight");
    }

    @Override
    public void periodic() {
        if (!visionEnabled) {
            return;
        }

        updateRobotOrientation();
        processVisionEstimate();
        updateDashboard();
    }

    /**
     * Feed current robot heading to Limelight for MegaTag2.
     * This significantly improves pose estimation accuracy, especially with single tags.
     */
    private void updateRobotOrientation() {
        Rotation2d heading = robotState.getGyroHeading();
        double angularVelocity = robotState.getAngularVelocityDegreesPerSec();

        LimelightHelpers.SetRobotOrientation(
                limelightName,
                heading.getDegrees(),
                angularVelocity, // Angular velocity helps with latency compensation
                0,
                0,
                0,
                0 // Pitch, roll, and their rates (usually 0 for ground robots)
                );
    }

    /**
     * Get the latest pose estimate from Limelight, validate it, and publish to RobotState.
     */
    private void processVisionEstimate() {
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        String rejectReason = getRejectReason(estimate);
        if (rejectReason != null) {
            lastRejectReason = rejectReason;
            consecutiveRejects++;
            robotState.recordVisionRejected();
            return;
        }

        // Valid estimate - publish to RobotState
        consecutiveRejects = 0;
        lastRejectReason = "";

        robotState.addVisionMeasurement(new VisionMeasurement(
                estimate.pose, estimate.timestampSeconds, estimate.tagCount, estimate.avgTagDist, limelightName));
    }

    /**
     * Validate a pose estimate and return the reason for rejection, or null if valid.
     */
    private String getRejectReason(PoseEstimate estimate) {
        // No data
        if (estimate == null) {
            return "No estimate";
        }

        // Not enough tags
        if (estimate.tagCount < MIN_TAG_COUNT) {
            return "Tag count: " + estimate.tagCount;
        }

        // Tags too far away
        if (estimate.avgTagDist > MAX_TAG_DISTANCE) {
            return "Distance: " + String.format("%.2f", estimate.avgTagDist);
        }

        // Robot rotating too fast (causes motion blur)
        double angularVelocity = Math.abs(robotState.getAngularVelocityDegreesPerSec());
        if (angularVelocity > MAX_ANGULAR_VELOCITY_FOR_VISION) {
            return "Angular velocity: " + String.format("%.1f", angularVelocity);
        }

        // Pose outside field bounds
        Pose2d pose = estimate.pose;
        if (pose.getX() < -0.5
                || pose.getX() > FIELD_LENGTH + 0.5
                || pose.getY() < -0.5
                || pose.getY() > FIELD_WIDTH + 0.5) {
            return "Out of bounds: " + String.format("(%.2f, %.2f)", pose.getX(), pose.getY());
        }

        // Check for unreasonable pose jump (optional, more aggressive filtering)
        Pose2d currentPose = robotState.getCurrentPose();
        double poseDelta = currentPose.getTranslation().getDistance(pose.getTranslation());
        if (poseDelta > 1.0 && estimate.tagCount == 1) {
            // Large jump with single tag - suspicious
            return "Large jump with single tag: " + String.format("%.2f", poseDelta);
        }

        // All checks passed
        return null;
    }

    /**
     * Update SmartDashboard with vision status.
     */
    private void updateDashboard() {
        String prefix = "Vision/" + limelightName + "/";

        SmartDashboard.putBoolean(prefix + "Enabled", visionEnabled);
        SmartDashboard.putNumber(prefix + "ConsecutiveRejects", consecutiveRejects);
        SmartDashboard.putString(prefix + "LastRejectReason", lastRejectReason);
        SmartDashboard.putNumber(prefix + "TagCount", getVisibleTagCount());
        SmartDashboard.putNumber(prefix + "AvgTagDistance", getAverageTagDistance());
    }

    /**
     *  Get the number of AprilTags currently visible
     */
    public int getVisibleTagCount() {
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        return estimate != null ? estimate.tagCount : 0;
    }

    /**
     * Get the average distance to visible tags
     */
    public double getAverageTagDistance() {
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        return estimate != null ? estimate.avgTagDist : -1;
    }

    /**
     * Check if we have a valid vision lock (seeing tags)
     */
    public boolean hasVisionLock() {
        return getVisibleTagCount() >= MIN_TAG_COUNT;
    }

    /**
     * Get the name of this Limelight
     */
    public String getLimelightName() {
        return limelightName;
    }

    /**
     *Get the last reason a measurement was rejected
     */
    public String getLastRejectReason() {
        return lastRejectReason;
    }

    /**
     * Get consecutive reject count (for debugging)
     */
    public int getConsecutiveRejects() {
        return consecutiveRejects;
    }
}

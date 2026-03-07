package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.TowerAlignmentState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.LimelightHelpers;

public class AlignToTower extends Command {
  private final Drivetrain drivetrain;
  private final RobotState robotState = RobotState.getInstance();

  private final boolean finishWhenAligned;
  private int activeTagId = -1;

  private final PIDController rotationPid = new PIDController(Constants.Targeting.ROTATION_KP,
      Constants.Targeting.ROTATION_KI, Constants.Targeting.ROTATION_KD);
  private final PIDController forwardPid = new PIDController(Constants.Targeting.FORWARD_KP,
      Constants.Targeting.FORWARD_KI, Constants.Targeting.FORWARD_KD);
  private final PIDController strafePid = new PIDController(Constants.Targeting.STRAFE_KP,
      Constants.Targeting.STRAFE_KI, Constants.Targeting.STRAFE_KD);

  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

  public AlignToTower(Drivetrain drivetrain, boolean finishWhenAligned) {
    this.drivetrain = drivetrain;
    this.finishWhenAligned = finishWhenAligned;

    rotationPid.setTolerance(1.5);
    forwardPid.setTolerance(0.03);
    strafePid.setTolerance(0.03);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    activeTagId = robotState.getTowerPrimaryTagID();
    LimelightHelpers.setPriorityTagID(Constants.Vision.LIMELIGHT_NAME, activeTagId);
    rotationPid.reset();
    forwardPid.reset();
    strafePid.reset();
  }

  @Override
  public void execute() {
    if (!LimelightHelpers.getTV(Constants.Vision.LIMELIGHT_NAME)) {
      drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      robotState.setTowerAlignment(new TowerAlignmentState(false, false, activeTagId, 0, 0, 0, 0, 0, 0));
      return;
    }

    double tx = LimelightHelpers.getTX(Constants.Vision.LIMELIGHT_NAME);
    Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.Vision.LIMELIGHT_NAME);
    double distance = targetPose.getZ();
    double lateral = targetPose.getX();

    double rotOutput = MathUtil
        .clamp(
            rotationPid.calculate(tx, 0.0),
            -Constants.Targeting.MAX_ROTATION_SPEED,
            Constants.Targeting.MAX_ROTATION_SPEED);

    double fwdOutput = MathUtil
        .clamp(
            forwardPid.calculate(distance, Constants.Tower.TARGET_DISTANCE),
            -Constants.Targeting.MAX_DRIVE_SPEED,
            Constants.Targeting.MAX_DRIVE_SPEED);

    double strafeOutput = MathUtil
        .clamp(
            strafePid.calculate(lateral, Constants.Tower.TARGET_LATERAL_OFFSET),
            -Constants.Targeting.MAX_DRIVE_SPEED,
            Constants.Targeting.MAX_DRIVE_SPEED);

    drivetrain
        .setControl(robotCentric.withVelocityX(fwdOutput).withVelocityY(strafeOutput).withRotationalRate(-rotOutput));

    boolean aligned = rotationPid.atSetpoint() && forwardPid.atSetpoint() && strafePid.atSetpoint();
    robotState
        .setTowerAlignment(
            new TowerAlignmentState(true, aligned, activeTagId, tx, distance, lateral, fwdOutput, strafeOutput,
                rotOutput));
  }

  @Override
  public boolean isFinished() {
    if (!finishWhenAligned) {
      return false;
    }
    return rotationPid.atSetpoint() && forwardPid.atSetpoint() && strafePid.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    LimelightHelpers.setPriorityTagID(Constants.Vision.LIMELIGHT_NAME, -1);
    robotState.clearTowerAlignment();
  }
}

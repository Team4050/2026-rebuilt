package frc.robot.auto;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

public class SimpleClimb extends SequentialCommandGroup {

  public SimpleClimb(Drivetrain drivetrain, Climber climber) {
    double autoSpeed = 0.25 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    double driveTimeSec = 0.5;

    var robotCentric = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    addCommands(
        // 1. Drive left while moving climber up
        Commands
            .parallel(
                drivetrain
                    .applyRequest(() -> robotCentric.withVelocityX(0).withVelocityY(autoSpeed).withRotationalRate(0))
                    .withTimeout(driveTimeSec),
                Commands.runOnce(() -> climber.setPosition(climber.ENCODER_POSITION_TOP), climber)),
        // 2. Drive back
        drivetrain
            .applyRequest(() -> robotCentric.withVelocityX(-autoSpeed).withVelocityY(0).withRotationalRate(0))
            .withTimeout(driveTimeSec),
        // 3. Move climber down
        Commands.runOnce(() -> climber.setPosition(climber.ENCODER_POSITION_BOTTOM), climber));

    setName("Auto: Simple");
  }
}

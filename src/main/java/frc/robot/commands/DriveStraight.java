package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class DriveStraight extends PIDCommand {
    private final Drivetrain drivetrain;

    @SuppressWarnings("deprecation")
    public DriveStraight(Drivetrain drivetrain, double distance) {
      super(
        new PIDController(0.1, 0, 0),
        ()->{return drivetrain.getPose().getX();},
        distance,
        d -> drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(d).withVelocityY(0.0))
      );

      this.drivetrain = drivetrain;
      addRequirements(this.drivetrain);

      getController().setTolerance(0.01);
    }
}

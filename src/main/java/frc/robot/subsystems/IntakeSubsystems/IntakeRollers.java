package frc.robot.subsystems.IntakeSubsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollers extends SubsystemBase {

  private final double SPEED = 0.7;

  private final SparkMax motor = new SparkMax(Constants.Subsystems.intakeRollerId, SparkMax.MotorType.kBrushless);

  public IntakeRollers() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(50);

    if (motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring Intake Rollers Motor", false);
    }
  }

  private void stop() {
    motor.stopMotor();
  }

  private void intakeIn() {
    motor.set(SPEED);
  }

  private void intakeOut() {
    motor.set(-SPEED);
  }

  public Command inCommand() {
    return startEnd(this::intakeIn, this::stop).withName("Intake: In");
  }

  public Command outCommand() {
    return startEnd(this::intakeOut, this::stop).withName("Intake: Out");
  }

  public double motorCurrent() {
    return motor.getOutputCurrent();
  }
}

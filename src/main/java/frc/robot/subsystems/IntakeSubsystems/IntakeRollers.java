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

  private final SparkMax motor = new SparkMax(Constants.Subsystems.intakeRollerId, SparkMax.MotorType.kBrushless);

  private final double speed = 0.7;

  public IntakeRollers() {
    final SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(50);

    if (motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring Intake Rollers Motor", false);
    }
  }

  private void stop() {
    motor.stopMotor();
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("Intake Rollers: Stop");
  }

  public void intakeForward() {
    motor.set(speed);
  }

  public void intakeReverse() {
    motor.set(-speed);
  }

  public double motorCurrent() {
    return motor.getOutputCurrent();
  }
}

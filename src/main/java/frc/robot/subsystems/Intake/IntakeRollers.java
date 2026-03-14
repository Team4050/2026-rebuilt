package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollers extends SubsystemBase {

  private static final String SPEED_KEY = "Intake Roller Speed";
  private static final double DEFAULT_SPEED = 0.55;

  private final SparkMax motor = new SparkMax(Constants.Subsystems.intakeRollerId, SparkMax.MotorType.kBrushless);

  public IntakeRollers() {
    SmartDashboard.putNumber(SPEED_KEY, DEFAULT_SPEED);

    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(50);

    if (motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring Intake Rollers Motor", false);
    }
  }

  private double getSpeed() {
    return SmartDashboard.getNumber(SPEED_KEY, DEFAULT_SPEED);
  }

  private void stop() {
    motor.stopMotor();
  }

  private void intakeIn() {
    motor.set(getSpeed());
  }

  private void intakeOut() {
    motor.set(-getSpeed());
  }

  public Command inCommand() {
    return startEnd(this::intakeIn, this::stop).withName("Intake: In");
  }

  public Command outCommand() {
    return startEnd(this::intakeOut, this::stop).withName("Intake: Out");
  }
}

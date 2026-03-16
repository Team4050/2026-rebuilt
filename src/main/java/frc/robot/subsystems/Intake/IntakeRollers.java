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

  private static final String SPEED_IN_KEY = "Intake Roller Speed In";
  private static final double DEFAULT_SPEED_IN = 0.7;

  private static final String SPEED_OUT_KEY = "Intake Roller Speed Out";
  private static final double DEFAULT_SPEED_OUT = 0.7;

  private final SparkMax motor = new SparkMax(Constants.Subsystems.intakeRollerId, SparkMax.MotorType.kBrushless);

  public IntakeRollers() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(50);

    if (motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring Intake Rollers Motor", false);
    }

    SmartDashboard.putNumber(SPEED_IN_KEY, DEFAULT_SPEED_IN);
    SmartDashboard.putNumber(SPEED_OUT_KEY, DEFAULT_SPEED_OUT);
  }

  private double getInSpeed() {
    return SmartDashboard.getNumber(SPEED_IN_KEY, DEFAULT_SPEED_IN);
  }

  private double getOutSpeed() {
    return SmartDashboard.getNumber(SPEED_OUT_KEY, DEFAULT_SPEED_OUT);
  }

  private void stop() {
    motor.stopMotor();
  }

  private void intakeIn() {
    motor.set(getInSpeed());
  }

  private void intakeOut() {
    motor.set(-getOutSpeed());
  }

  public Command inCommand() {
    return startEnd(this::intakeIn, this::stop).withName("Intake: In");
  }

  public Command outCommand() {
    return startEnd(this::intakeOut, this::stop).withName("Intake: Out");
  }
}

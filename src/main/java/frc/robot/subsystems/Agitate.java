package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Agitate extends SubsystemBase {

  private static final String SPEED_KEY = "Agitate Speed";
  private static final double DEFAULT_SPEED = 1.0;

  private final SparkMax motor = new SparkMax(Constants.Subsystems.agitateId, SparkMax.MotorType.kBrushed);

  public Agitate() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(50);

    if (motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring Agitate Motor", false);
    }

    SmartDashboard.putNumber(SPEED_KEY, DEFAULT_SPEED);
  }

  private double getSpeed() {
    return SmartDashboard.getNumber(SPEED_KEY, DEFAULT_SPEED);
  }

  public void stop() {
    motor.stopMotor();
  }

  public void agitateOn() {
    motor.set(getSpeed());
  }

  public Command agitateCommand() {
    return new RunCommand(this::agitateOn).finallyDo(() -> this.stop()).withName("Agitate On");
  }

}

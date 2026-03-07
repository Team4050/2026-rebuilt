package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {

  public enum OuttakeMode {

    Outtake("Outtake Mode"), Shooter("Shooter Mode");

    private String outtakeModeString;

    OuttakeMode(String outtakeModeString) {
      this.outtakeModeString = outtakeModeString;
    }
  }

  private SparkMax motor;
  private OuttakeMode outtakeMode;

  private final double outtakeSpeed = 0.7;
  private final double shooterSpeed = 1;
  private final double outtakeRevSpeed = -0.5;

  public Outtake(int motorId, OuttakeMode mode) {
    outtakeMode = mode;
    motor = new SparkMax(motorId, SparkMax.MotorType.kBrushless);

    final SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(50);

    if (motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring Outtake Motor " + motorId, false);
    }
  }

  private void setOuttakeMode(OuttakeMode mode) {
    outtakeMode = mode;
  }

  private void stop() {
    motor.stopMotor();
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("Outtake: Stop");
  }

  public void Forward() {
    if (outtakeMode == OuttakeMode.Outtake) {
      motor.set(outtakeSpeed);
    } else if (outtakeMode == OuttakeMode.Shooter) {
      motor.set(shooterSpeed);
    }
  }

  public void Reverse() {
    motor.set(outtakeRevSpeed);
  }

  public double motorCurrent() {
    return motor.getOutputCurrent();
  }
}

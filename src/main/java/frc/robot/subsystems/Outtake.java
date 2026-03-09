package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
// For configuration
import com.revrobotics.RelativeEncoder; // For getting positions/velocities
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {

  public enum OuttakeMode {

    Unloader("Unloader Mode"), Shooter("Shooter Mode");

    private String outtakeModeString;

    OuttakeMode(String outtakeModeString) {
      this.outtakeModeString = outtakeModeString;
    }
  }

  private SparkMax motor; // motor is the unloader motor
  private SparkMax motor2; // motor2  is the shooter motor
  private RelativeEncoder encoder2; // encoder for the shooter motor
  private OuttakeMode outtakeMode;
  private int motorId;
  private int motorId2;

  private final double outtakeSpeed = -0.7;
  private final double shooterSpeed = -1;
  private final double outtakeRevSpeed = 0.5;

  public Outtake(int motorId, int motorId2, OuttakeMode mode) {
    //outtakeMode = OuttakeMode.Outtake;
    outtakeMode = mode;
    this.motorId = motorId;
    motor = new SparkMax(motorId, SparkMax.MotorType.kBrushless);

    final SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(20);

    if (motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring Outtake motor " + motorId, false);
    }

    if (outtakeMode == OuttakeMode.Shooter) {
      this.motorId2 = motorId2;

      motor2 = new SparkMax(motorId2, SparkMax.MotorType.kBrushless);

      encoder2 = motor2.getEncoder();

      final SparkMaxConfig config2 = new SparkMaxConfig();
      config2.idleMode(IdleMode.kCoast).smartCurrentLimit(20);

      if (motor2
          .configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
        DriverStation.reportWarning("Error configuring Outtake motor " + motorId2, false);
      }
    }
  }

  private void setOuttakeMode(OuttakeMode mode) {
    outtakeMode = mode;
  }

  public Command setOuttakeModeCommand(OuttakeMode mode) {
    return runOnce(() -> setOuttakeMode(mode))
        .withName("Set outtake " + motorId + " mode to " + mode.outtakeModeString);
  }

  private void stop() {
    if (outtakeMode == OuttakeMode.Shooter) {
      motor2.stopMotor();
      motor.stopMotor();
    } else if (outtakeMode == OuttakeMode.Unloader) {
      motor.stopMotor();
    }
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("Outtake motor " + motorId + " stopped");
  }

  public void Forward() {
    if (outtakeMode == OuttakeMode.Unloader) {
      motor.set(outtakeSpeed);
    } else if (outtakeMode == OuttakeMode.Shooter) {
      Shooting();
    }
  }

  public void Reverse() {
    motor.set(outtakeRevSpeed);
  }

  public void Shooting() {
    motor2.set(shooterSpeed);
    if (encoder2.getVelocity() > 2000.0) {
      motor.set(shooterSpeed);
    }
  }

  public double motorCurrent() {
    return motor.getOutputCurrent();
  }
}

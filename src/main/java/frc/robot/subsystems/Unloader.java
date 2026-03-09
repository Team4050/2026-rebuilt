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

public class Unloader extends SubsystemBase {
  private SparkMax kickerMotor;
  private SparkMax shooterMotor;

  public Unloader(int kickerMotorId) {
    kickerMotor = new SparkMax(kickerMotorId, SparkMax.MotorType.kBrushless);

    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(20);

    if (kickerMotor
        .configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring kicker motor " + kickerMotorId, false);
    }
  }

  public Unloader(int kickerMotorId, int shooterMotorId) {
    // Add kicker motor (with single param constructor)
    this(kickerMotorId);

    // Add shooter motor
    addShooter(shooterMotorId);
  }

  public void addShooter(int shooterMotorId) {
    if (shooterMotor != null) {
      // No-op if shooter is already configured
      return;
    }

    shooterMotor = new SparkMax(shooterMotorId, SparkMax.MotorType.kBrushless);

    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(50);

    if (shooterMotor
        .configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring shooter motor " + shooterMotorId, false);
    }
  }

  public boolean hasShooter() {
    return shooterMotor != null;
  }

  private void stop() {
    kickerMotor.stopMotor();

    if (shooterMotor != null) {
      shooterMotor.stopMotor();
    }
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("Unloader: Stop motors");
  }
}

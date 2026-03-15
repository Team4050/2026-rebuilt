package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Unloader extends SubsystemBase {
  private static final String SPEED_SHOOTER_KEY = "Shooter Speed";
  private static final double DEFAULT_SPEED_SHOOTER = 0.55;

  private final int KICKER_CURRENT_LIMIT = 20;
  private final int SHOOTER_CURRENT_LIMIT = 50;

  private final double KICKER_SPEED_OUT = 0.7;
  private final double KICKER_SPEED_IN = -0.5;

  private SparkMax kickerMotor;
  private SparkMax shooterMotor;
  private RelativeEncoder shooterEncoder;

  public Unloader(int kickerMotorId, boolean reverseKicker) {
    kickerMotor = new SparkMax(kickerMotorId, SparkMax.MotorType.kBrushless);

    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(KICKER_CURRENT_LIMIT).inverted(reverseKicker);

    if (kickerMotor
        .configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring kicker motor " + kickerMotorId, false);
    }
  }

  public Unloader(int kickerMotorId, boolean reverseKicker, int shooterMotorId, boolean reverseShooter) {
    // Add kicker motor (with single param constructor)
    this(kickerMotorId, reverseKicker);

    // Add shooter motor
    shooterMotor = new SparkMax(shooterMotorId, SparkMax.MotorType.kBrushless);

    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(SHOOTER_CURRENT_LIMIT).inverted(reverseShooter);

    if (shooterMotor
        .configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      DriverStation.reportWarning("Error configuring shooter motor " + shooterMotorId, false);
    }

    shooterEncoder = shooterMotor.getEncoder();

    SmartDashboard.putNumber(SPEED_SHOOTER_KEY, DEFAULT_SPEED_SHOOTER);
  }

  private double getShooterSpeed() {
    return SmartDashboard.getNumber(SPEED_SHOOTER_KEY, DEFAULT_SPEED_SHOOTER);
  }

  public boolean hasShooter() {
    return shooterMotor != null;
  }

  public void primeShooter() {
    if (hasShooter()) {
      shooterMotor.set(getShooterSpeed());
    }
  }

  public void shoot() {
    if (shooterEncoder.getVelocity() > 10.0) {
      kickerMotor.set(getShooterSpeed());
    }
  }

  public void runOuttake() {
    kickerMotor.set(KICKER_SPEED_OUT);
  }

  public void reverse() {
    kickerMotor.set(KICKER_SPEED_IN);
  }

  public void stopKicker() {
    kickerMotor.stopMotor();
  }

  public void stopShooter() {
    if (hasShooter()) {
      shooterMotor.stopMotor();
    }
  }

  public double getShooterRPM() {
    return hasShooter() ? shooterEncoder.getVelocity() : 0;
  }

  public boolean kickerIsRunning() {
    return kickerMotor.getEncoder().getVelocity() > 0.1;
  }
}

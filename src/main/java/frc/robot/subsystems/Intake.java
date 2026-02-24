package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final SparkMax intake = new SparkMax(Constants.Subsystems.intakeRollerId, SparkMax.MotorType.kBrushless);
  // private final SparkMax intakeDeploy = new SparkMax(Constants.Subsystems.intakeDeployId,
  // SparkMax.MotorType.kBrushless);

  public Intake() {
    SparkMaxConfig mainConfig = new SparkMaxConfig();

    mainConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    if (intake
        .configure(mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
      throw new IllegalStateException("Error configuring Intake Motor");
    }
  }

  public void stop() {
    intakeStop();
    // deployStop();
  }

  public void intakeForward() {
    intake.set(0.25);
  }

  public void intakeStop() {
    intake.set(0);
  }

  public void intakeReverse() {
    intake.set(-0.25);
  }

  // public void deployOut() {
  //     intakeDeploy.set(0.5);
  // }

  // public void deployStop() {
  //     intakeDeploy.set(0);
  // }

  // public void deployIn() {
  //     intakeDeploy.set(-0.5);
  // }

  public double getSpeed() {
    return intake.get();
  }
}

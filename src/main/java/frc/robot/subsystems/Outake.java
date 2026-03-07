package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outake extends SubsystemBase {
  // private final SparkMax motor = new SparkMax(Constants.Subsystems.outakeRollerId, SparkMax.MotorType.kBrushless);

  //   private final double speed = 0.7;

  //   public OutakeSub() {
  //     final SparkMaxConfig config = new SparkMaxConfig();
  //     config.idleMode(IdleMode.kCoast).smartCurrentLimit(50);

  //     if (motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) != REVLibError.kOk) {
  //       DriverStation.reportWarning("Error configuring Intake Rollers Motor", false);
  //     }
  //   }

  //   private void stop() {
  //     motor.stopMotor();
  //   }

  //   public Command stopCommand() {
  //     return runOnce(this::stop).withName("Intake Rollers: Stop");
  //   }

  //   public void intakeForward() {
  //     motor.set(speed);
  //   }

  //   public void intakeReverse() {
  //     motor.set(-speed);
  //   }

  //   public double motorCurrent() {
  //     return motor.getOutputCurrent();
  //   }
}

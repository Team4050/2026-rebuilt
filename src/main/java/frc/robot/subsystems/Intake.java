package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final SparkMax intake = new SparkMax(51, SparkMax.MotorType.kBrushless);
    private final SparkMax intakeDeploy = new SparkMax(52, SparkMax.MotorType.kBrushless);

    public void stop() {
        intakeStop();
        deployStop();
    }

    public void intakeForward() {
        intake.set(1);
    }

    public void intakeStop() {
        intake.set(0);
    }

    public void intakeReverse() {
        intake.set(-1);
    }

    public void deployOut() {
        intakeDeploy.set(0.5);
    }

    public void deployStop() {
        intakeDeploy.set(0);
    }

    public void deployIn() {
        intakeDeploy.set(-0.5);
    }
}

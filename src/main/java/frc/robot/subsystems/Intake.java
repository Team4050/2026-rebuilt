package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private final SparkMax intake = new SparkMax(Constants.Subsystems.intakeRollerId, SparkMax.MotorType.kBrushless);
    private final SparkMax intakeDeploy =
            new SparkMax(Constants.Subsystems.intakeDeployId, SparkMax.MotorType.kBrushless);

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

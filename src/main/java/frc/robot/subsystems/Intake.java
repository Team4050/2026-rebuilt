package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final SparkMax intake = new SparkMax(51, SparkMax.MotorType.kBrushless);
    private final SparkMax intakeDeply = new SparkMax(52, SparkMax.MotorType.kBrushless);

    public void runIntake() {
        intake.set(1);
    }

    public void stopIntake() {
        intake.set(0);
    }

    public void reverseIntake() {
        intake.set(-1);
    }

    /**
     * Set the direction of the intake deploy motor.
     * @param deployOut the maximum speed. This number should be between 0.1 and 1.0.
     */

    public void deployOut() {
        intakeDeply.set(0.5);
    }

    /**
     * Stop the intake deploy motor.
     * @param deployStop the maximum speed. This number should be 0.0.
     */

    public void deployStop() {
        intakeDeply.set(0);
    }

    /**
     * Set the direction of the intake deploy motor.
     * @param deployIn the maximum speed. This number should be between -0.1 and -1.0.
     */

    public void deployIn() {
        intakeDeply.set(-0.5);
    }
}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final SparkMax intake = new SparkMax(51, SparkMax.MotorType.kBrushless);
    private final SparkMax intakdeply = new SparkMax(52, SparkMax.MotorType.kBrushless);
    // rem to add deploy motor later

    public void runIntake() {
        intake.set(1);
    }

    public void stopIntake() {
        intake.set(0);
    }

    public void reverseIntake() {
        intake.set(-1);
    }

    public void deployIntakeout() {
        intakdeply.set(0.5);
    }

    public void deployIntakeStop() {
        intake.set(0);
        intakdeply.set(0);
    }

    public void deployIntakein() {
        intakdeply.set(-0.5);
    }
}

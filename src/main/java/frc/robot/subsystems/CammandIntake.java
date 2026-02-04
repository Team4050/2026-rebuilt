package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CammandIntake extends SubsystemBase {

        private final SparkMax intake = new SparkMax(5, SparkMax.MotorType.kBrushless);
        // private final SparkMax intakdeply = new SparkMax(5, SparkMax.MotorType.kBrushless); 
        //rem to add deploy motor later

        public void runIntake(){
            intake.set(1);
        }
        public void stopIntake(){
            intake.set(0);
        }

}
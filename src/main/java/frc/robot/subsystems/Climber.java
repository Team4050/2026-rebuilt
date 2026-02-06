package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private static final int kLeaderID = 6;
    private static final int kFollowerID = 7;

    private final SparkMax leaderMotor = new SparkMax(kLeaderID, MotorType.kBrushless);

    private final SparkMax followerMotor = new SparkMax(kFollowerID, MotorType.kBrushless);

    public Climber() {
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        leaderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);

        followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).follow(leaderMotor, true);

        // apply configs, check for errors
        if (leaderMotor.configure(leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters)
                != REVLibError.kOk) {
            System.err.println("Error configuring Climber Leader Motor");
        }
        if (followerMotor.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters)
                != REVLibError.kOk) {
            System.err.println("Error configuring Climber Follower Motor");
        }
        ;
    }

    /**
     * Run the climber with a fixed speed.
     * @param speed The speed to set. Value should be between -1.0 and 1.0
     */
    public void setSpeed(double speed) {
        System.out.println("Climber - speed set to " + speed);
        leaderMotor.set(speed);
    }

    /**
     * Stop the climber.
     */
    public void stop() {
        System.out.println("Climber - stopped");
        leaderMotor.stopMotor();
    }
}

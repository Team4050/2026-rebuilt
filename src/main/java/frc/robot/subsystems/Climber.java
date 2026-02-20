package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    // default units are rotations
    private double encoderPositionMin = 0.0;
    // TODO at least max position must be calibrated manually and refactored here
    private double encoderPositionMax = 5.0;

    private double speedFactor = 0.25;

    private final SparkMax leaderMotor = new SparkMax(Constants.Subsystems.climberPrimaryId, MotorType.kBrushless);

    // private final SparkMax followerMotor = new SparkMax(Constants.Subsystems.climberFollowerId,
    // MotorType.kBrushless);

    private final RelativeEncoder encoder = leaderMotor.getEncoder();

    public Climber() {
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        // SparkMaxConfig followerConfig = new SparkMaxConfig();

        leaderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);

        // followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).follow(leaderMotor, true);

        // apply configs, check for errors
        if (leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
                != REVLibError.kOk) {
            throw new IllegalStateException("Error configuring Climber Leader Motor");
        }
        // if (followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        //         != REVLibError.kOk) {
        //     throw new IllegalStateException("Error configuring Climber Follower Motor");
        // }

        // on startup, assume climber is in the "down" position
        encoder.setPosition(0.0);
    }

    private boolean isAtUpperLimit() {
        return encoder.getPosition() >= encoderPositionMax;
    }

    private boolean isAtLowerLimit() {
        return encoder.getPosition() <= encoderPositionMin;
    }

    private void setSpeed(double speed) {
        leaderMotor.set(speed * speedFactor);
    }

    /**
     * Set the maximum speed threshold for the climber.
     * @param speed the maximum speed. This number should be between 0 and 1.0.
     */
    public void setSpeedFactor(double speed) {
        if (speed > 1.0 || speed < 0.0) {
            speedFactor = 1.0;
        } else {
            speedFactor = speed;
        }
    }

    /**
     * Move the climber up at full speed
     */
    public void up() {
        if (!isAtUpperLimit()) {
            setSpeed(1.0);
        }
    }

    /**
     * Move the climber down at full speed
     */
    public void down() {
        if (!isAtLowerLimit()) {
            setSpeed(-1.0);
        }
    }

    /**
     * Stop the climber.
     */
    public void stop() {
        leaderMotor.stopMotor();
    }

    @Override
    public void periodic() {
        /* not yet used */
    }
}

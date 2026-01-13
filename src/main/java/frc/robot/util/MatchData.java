package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class MatchData {

    private enum Shift {
        TRANSITION,
        FIRST,
        SECOND,
        THIRD,
        FOURTH
    }

    private double matchRemainingTimeCache = -1;
    private DriverStation.Alliance alliance;
    private DriverStation.Alliance firstAllianceInactive;
    private boolean needGameDataCheck = true;

    /**
     * Initialize the MatchData instance. Must not be called until after the DriverStation is initialized.
     */
    public void init() {
        alliance = DriverStation.getAlliance().orElse(null);
    }

    /**
     * Periodic update method. Should be called regularly (e.g., in robotPeriodic).
     */
    public void periodic() {
        matchRemainingTimeCache = DriverStation.getMatchTime();

        if (needGameDataCheck) {
            String gameData = DriverStation.getGameSpecificMessage();
            if (gameData.length() > 0) {
                switch (gameData.charAt(0)) {
                    case 'B':
                        firstAllianceInactive = DriverStation.Alliance.Blue;
                        break;
                    case 'R':
                        firstAllianceInactive = DriverStation.Alliance.Red;
                        break;
                    default:
                        // Invalid data
                        firstAllianceInactive = null;
                        break;
                }
                needGameDataCheck = false;
            }
        }
    }

    /**
     * Get the cached match time remaining.
     * @return The cached match time remaining in seconds.
     */
    public double getMatchTimeRemaining() {
        return matchRemainingTimeCache;
    }

    /**
     * Check if the match time indicates we are in the transition shift period.
     * This may be slightly inaccurate due to time delays, so it should not be used
     * for critical decisions (like enabling or disabling mechanisms).
     * @return true if we're probably in the transition shift period (20-30 seconds remaining)
     */
    public boolean isProbablyTransitionShift() {
        return probablyGetCurrentShift() == Shift.TRANSITION;
    }

    /**
     * Check if the match time indicates we are in the endgame period.
     * This may be slightly inaccurate due to time delays, so it should not be used
     * for critical decisions (like enabling or disabling mechanisms).
     * @return true if we're probably in the endgame period (30 seconds or less remaining)
     */
    public boolean isProbablyEndGame() {
        matchRemainingTimeCache = DriverStation.getMatchTime();
        return matchRemainingTimeCache <= 30;
    }

    /**
     * Check if the match time indicates we are in our alliance's scoring period.
     * This may be slightly inaccurate due to time delays, so it should not be used
     * for critical decisions (like enabling or disabling mechanisms).
     * @return true if we're probably in our alliance's scoring period
     */
    public boolean isProbablyOurScoringPeriod() {
        Shift currentShift = probablyGetCurrentShift();
        if (firstAllianceInactive == null || alliance == null || currentShift == null) {
            return false;
        }

        if (firstAllianceInactive == alliance) {
            return currentShift == Shift.SECOND || currentShift == Shift.FOURTH;
        } else {
            return currentShift == Shift.FIRST || currentShift == Shift.THIRD;
        }
    }

    private Shift probablyGetCurrentShift() {
        if (!DriverStation.isTeleop()) {
            return null;
        }

        matchRemainingTimeCache = DriverStation.getMatchTime();

        if (matchRemainingTimeCache > 130 && matchRemainingTimeCache <= 140) {
            return Shift.TRANSITION;
        } else if (matchRemainingTimeCache > 105 && matchRemainingTimeCache <= 130) {
            return Shift.FIRST;
        } else if (matchRemainingTimeCache > 80 && matchRemainingTimeCache <= 105) {
            return Shift.SECOND;
        } else if (matchRemainingTimeCache > 55 && matchRemainingTimeCache <= 80) {
            return Shift.THIRD;
        } else if (matchRemainingTimeCache > 30 && matchRemainingTimeCache <= 55) {
            return Shift.FOURTH;
        }

        return null;
    }
}

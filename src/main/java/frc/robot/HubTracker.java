// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Tracks the HUB active/inactive status based on FMS game data and match time.
 * 
 * During TELEOP, the match is divided into shifts:
 *   TRANSITION SHIFT (2:20-2:10) - Both HUBs active
 *   SHIFT 1 (2:10-1:45) - Alternating, based on AUTO results
 *   SHIFT 2 (1:45-1:20) - Alternating
 *   SHIFT 3 (1:20-0:55) - Alternating
 *   SHIFT 4 (0:55-0:30) - Alternating
 *   END GAME (0:30-0:00) - Both HUBs active
 * 
 * The alliance that scored MORE fuel in AUTO has their HUB inactive in SHIFT 1.
 * FMS sends this info via getGameSpecificMessage() at the start of TELEOP.
 */
public class HubTracker {

    public enum HubStatus {
        ACTIVE,
        INACTIVE,
        UNKNOWN
    }

    public enum MatchShift {
        AUTO,
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        END_GAME,
        UNKNOWN
    }

    // Whether our alliance scored more in AUTO (meaning we're inactive in SHIFT 1)
    private boolean weWonAuto = false;
    private boolean gameDataReceived = false;
    private MatchShift lastShift = MatchShift.UNKNOWN;

    /**
     * Call once at the start of TELEOP to read FMS game data.
     * The game data indicates which alliance scored more FUEL in AUTO.
     * Check the 2026 FRC Control System docs for the exact format.
     */
    public void readGameData() {
        String gameData = DriverStation.getGameSpecificMessage();
        var alliance = DriverStation.getAlliance();

        if (gameData != null && !gameData.isEmpty() && alliance.isPresent()) {
            gameDataReceived = true;

            // Game data format: the alliance letter that scored more in AUTO
            // (or was randomly selected). E.g. "R" for Red, "B" for Blue.
            boolean redScoredMore = gameData.toUpperCase().startsWith("R");
            boolean weAreRed = alliance.get() == Alliance.Red;

            // If we scored more in AUTO, our HUB is INACTIVE in SHIFT 1
            weWonAuto = (weAreRed && redScoredMore) || (!weAreRed && !redScoredMore);

            DataLogManager.log("HubTracker: Game data = '" + gameData 
                + "', Alliance = " + alliance.get()
                + ", We won AUTO = " + weWonAuto);
        } else {
            gameDataReceived = false;
            DataLogManager.log("HubTracker: No game data received or alliance unknown");
        }
    }

    /**
     * Determines the current match shift based on the match timer.
     * DriverStation.getMatchTime() counts down during TELEOP from 2:20 (140s).
     */
    public MatchShift getCurrentShift(double matchTime) {
        if (matchTime > 130) {         // 2:20 - 2:10
            return MatchShift.TRANSITION;
        } else if (matchTime > 105) {  // 2:10 - 1:45
            return MatchShift.SHIFT_1;
        } else if (matchTime > 80) {   // 1:45 - 1:20
            return MatchShift.SHIFT_2;
        } else if (matchTime > 55) {   // 1:20 - 0:55
            return MatchShift.SHIFT_3;
        } else if (matchTime > 30) {   // 0:55 - 0:30
            return MatchShift.SHIFT_4;
        } else if (matchTime > 0) {    // 0:30 - 0:00
            return MatchShift.END_GAME;
        }
        return MatchShift.UNKNOWN;
    }

    /**
     * Returns whether our alliance's HUB is currently active.
     */
    public HubStatus getHubStatus(double matchTime) {
        if (!gameDataReceived) {
            return HubStatus.UNKNOWN;
        }

        MatchShift shift = getCurrentShift(matchTime);

        switch (shift) {
            case TRANSITION:
            case END_GAME:
                // Both HUBs active
                return HubStatus.ACTIVE;

            case SHIFT_1:
            case SHIFT_3:
                // If we won AUTO, we're INACTIVE in odd shifts
                return weWonAuto ? HubStatus.INACTIVE : HubStatus.ACTIVE;

            case SHIFT_2:
            case SHIFT_4:
                // If we won AUTO, we're ACTIVE in even shifts
                return weWonAuto ? HubStatus.ACTIVE : HubStatus.INACTIVE;

            default:
                return HubStatus.UNKNOWN;
        }
    }

    /**
     * Call periodically during TELEOP to update dashboard and log shift changes.
     */
    public void update(double matchTime) {
        HubStatus status = getHubStatus(matchTime);
        MatchShift currentShift = getCurrentShift(matchTime);

        // Update dashboard
        SmartDashboard.putBoolean("HUB Active", status == HubStatus.ACTIVE);
        SmartDashboard.putString("Current Shift", currentShift.toString());
        SmartDashboard.putString("HUB Status", status.toString());

        // Log shift transitions
        if (currentShift != lastShift && currentShift != MatchShift.UNKNOWN) {
            DataLogManager.log("HubTracker: Shift change -> " + currentShift 
                + " | HUB = " + status);
            lastShift = currentShift;
        }
    }

    /**
     * Reset state for a new match.
     */
    public void reset() {
        weWonAuto = false;
        gameDataReceived = false;
        lastShift = MatchShift.UNKNOWN;
    }

    public boolean isGameDataReceived() {
        return gameDataReceived;
    }
}

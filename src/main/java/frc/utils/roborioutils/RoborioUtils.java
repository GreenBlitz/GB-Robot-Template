package frc.utils.roborioutils;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class RoborioUtils {

    private static double lastTime;

    private static double currentTime;


    public static void updateRioUtils() {
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
        logStatus();
    }

    private static void logStatus() {
        Logger.recordOutput(RoborioUtilsConstants.LOG_PATH + "CycleTime", getCurrentRoborioCycleTime());
        reportAlertsToLog();
    }

    private static void reportAlertsToLog() {
        if (getCurrentRoborioCycleTime() > getDefaultRoborioCycleTime() + RoborioUtilsConstants.TIME_STEP_TOLERANCE) {
            Logger.recordOutput(RoborioUtilsConstants.ALERT_LOG_PATH + "CycleOverrunAt", currentTime);
        }
    }

    public static double getDefaultRoborioCycleTime() {
        return RoborioUtilsConstants.DEFAULT_ROBORIO_CYCLE_TIME;
    }

    public static double getCurrentRoborioCycleTime() {
        return currentTime - lastTime;
    }

}

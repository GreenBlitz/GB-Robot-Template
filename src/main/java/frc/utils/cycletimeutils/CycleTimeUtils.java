package frc.utils.cycletimeutils;

import edu.wpi.first.hal.HALUtil;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

public class CycleTimeUtils {

    public static final double DEFAULT_CYCLE_TIME_SECONDS = 0.02;

    private static double lastTime = 0;
    private static double currentTime = 0;


    public static void updateCycleTime() {
        lastTime = currentTime;
        currentTime = Conversions.microSecondsToSeconds(HALUtil.getFPGATime());

        logStatus();
        reportAlerts();
    }

    private static void logStatus() {
        Logger.recordOutput(CycleTimeConstants.LOG_PATH + "CycleTime", getCurrentCycleTime());
    }

    private static void reportAlerts() {
        if (getCurrentCycleTime() > DEFAULT_CYCLE_TIME_SECONDS + CycleTimeConstants.TIME_STEP_TOLERANCE_SECONDS) {
            Logger.recordOutput(CycleTimeConstants.ALERT_LOG_PATH + "CycleOverrunAt", currentTime);
        }
    }

    public static double getCurrentCycleTime() {
        return currentTime - lastTime;
    }

}

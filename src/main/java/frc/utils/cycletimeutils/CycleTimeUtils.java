package frc.utils.cycletimeutils;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class CycleTimeUtils {

    private static double lastTime = 0;
    private static double currentTime = 0;
    private static double sumOfTimeSteps = 0;
    private static int timeStepsCounter = 0;


    public static void updateCycleTime() {
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
        sumOfTimeSteps += getCurrentCycleTime();
        timeStepsCounter++;

        logStatus();
        reportAlertsToLog();
    }

    private static double getCurrentCycleTime() {
        return currentTime - lastTime;
    }

    private static void logStatus() {
        Logger.recordOutput(CycleTimeConstants.LOG_PATH + "CycleTime", getCurrentCycleTime());
    }

    private static void reportAlertsToLog() {
        if (getCurrentCycleTime() > getDefaultCycleTime() + CycleTimeConstants.TIME_STEP_TOLERANCE) {
            Logger.recordOutput(CycleTimeConstants.ALERT_LOG_PATH + "CycleOverrunAt", currentTime);
        }
    }

    public static double getDefaultCycleTime() {
        return CycleTimeConstants.DEFAULT_ROBORIO_CYCLE_TIME;
    }

    public static double getAverageCycleTime() {//todo - maybe remove (or other way that more accurate)
        return sumOfTimeSteps / timeStepsCounter;
    }

}

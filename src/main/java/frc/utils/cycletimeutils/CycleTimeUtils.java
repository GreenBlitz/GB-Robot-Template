package frc.utils.cycletimeutils;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class CycleTimeUtils {

    private static double lastTime = 0;
    private static double currentTime = 0;
    private static double sumOfTimeSteps = 0;
    private static int timeStepsCounter = 0;


    public static void updateRioUtils() {
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
        sumOfTimeSteps += getCurrentRoborioCycleTime();
        timeStepsCounter++;

        logStatus();
        reportAlertsToLog();
    }

    private static double getCurrentRoborioCycleTime() {
        return currentTime - lastTime;
    }

    private static void logStatus() {
        Logger.recordOutput(CycleTimeConstants.LOG_PATH + "CycleTime", getCurrentRoborioCycleTime());
    }

    private static void reportAlertsToLog() {
        if (getCurrentRoborioCycleTime() > getDefaultRoborioCycleTime() + CycleTimeConstants.TIME_STEP_TOLERANCE) {
            Logger.recordOutput(CycleTimeConstants.ALERT_LOG_PATH + "CycleOverrunAt", currentTime);
        }
    }

    public static double getDefaultRoborioCycleTime() {
        return CycleTimeConstants.DEFAULT_ROBORIO_CYCLE_TIME;
    }

    public static double getAverageRoborioCycleTime() {
        return sumOfTimeSteps / timeStepsCounter;
    }

}

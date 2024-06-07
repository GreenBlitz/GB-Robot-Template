package frc.utils.roborioutils;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class RoborioUtils {

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
    }

    private static double getCurrentRoborioCycleTime() {
        return currentTime - lastTime;
    }

    private static void logStatus() {
        Logger.recordOutput(RoborioUtilsConstants.LOG_PATH + "CanUtilization", getCANUtilizationPercent());
        Logger.recordOutput(RoborioUtilsConstants.LOG_PATH + "AverageCycleTime", getAverageRoborioCycleTime());
        Logger.recordOutput(RoborioUtilsConstants.LOG_PATH + "CurrentCycleTime", getCurrentRoborioCycleTime());
        reportAlertsToLog();
    }

    private static void reportAlertsToLog() {
        if (RobotConstants.ROBOT_TYPE.isReal()) {
            reportCANAlertsToLog();
        }
        boolean badCycleTime = getAverageRoborioCycleTime() > getDefaultRoborioCycleTime() + RoborioUtilsConstants.TIME_STEP_TOLERANCE;
        Logger.recordOutput(RoborioUtilsConstants.ALERT_LOG_PATH + "BadAverageCycleTime", badCycleTime);
        if (getAverageRoborioCycleTime() > getDefaultRoborioCycleTime() + RoborioUtilsConstants.TIME_STEP_TOLERANCE) {
            Logger.recordOutput(RoborioUtilsConstants.ALERT_LOG_PATH + "CycleOverrunAt", currentTime);
        }
    }

    private static void reportCANAlertsToLog() {
        if (!isCANConnectedToRoborio()) {
            Logger.recordOutput(RoborioUtilsConstants.ALERT_LOG_PATH + "CANDisconnectAt", currentTime);
        }
        else if (getCANUtilizationPercent() > RoborioUtilsConstants.MAX_CAN_UTILIZATION_PERCENT) {
            Logger.recordOutput(RoborioUtilsConstants.ALERT_LOG_PATH + "CANFloodedAt", currentTime);
        }
    }

    public static double getDefaultRoborioCycleTime() {
        return RoborioUtilsConstants.DEFAULT_ROBORIO_CYCLE_TIME;
    }

    public static double getAverageRoborioCycleTime() {
        return sumOfTimeSteps / timeStepsCounter;
    }

    public static boolean isCANConnectedToRoborio() {
        return getCANUtilizationPercent() > 0;
    }

    public static double getCANUtilizationPercent() {
        return RobotController.getCANStatus().percentBusUtilization * 100;
    }

}

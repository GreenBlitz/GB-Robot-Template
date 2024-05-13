package frc.utils.roborioutils;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.RobotConstants;
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
        Logger.recordOutput(RoborioUtilsConstants.LOG_PATH + "CanUtilization", getCANUtilizationPercent());
        Logger.recordOutput(RoborioUtilsConstants.LOG_PATH + "CycleTime", getCurrentRoborioCycleTime());
        reportAlertsToLog();
    }

    private static void reportAlertsToLog() {
        if (RobotConstants.ROBOT_TYPE.isReal()) {
            reportCANAlertsToLog();
        }
        if (getCurrentRoborioCycleTime() > getDefaultRoborioCycleTime() + RoborioUtilsConstants.TIME_STEP_TOLERANCE) {
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

    public static double getCurrentRoborioCycleTime() {
        return currentTime - lastTime;
    }

    public static boolean isCANConnectedToRoborio() {
        return getCANUtilizationPercent() > 0;
    }

    public static double getCANUtilizationPercent() {
        return RobotController.getCANStatus().percentBusUtilization * 100;
    }

}

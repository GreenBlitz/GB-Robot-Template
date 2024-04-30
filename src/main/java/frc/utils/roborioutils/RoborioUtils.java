package frc.utils.roborioutils;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class RoborioUtils {

    private static double lastTime;

    private static double currentTime;

    public static void updateRioUtils() {
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
        logInfo();
    }

    private static void logInfo(){
        Logger.recordOutput(RoborioUtilsConstants.LOG_PATH + "CanUtilization", getCANUtilizationPercent());
        Logger.recordOutput(RoborioUtilsConstants.LOG_PATH + "CycleTime", getCurrentRoborioCycle());
        logAlertsChecks();
    }

    private static void logAlertsChecks() {
        if (!isCANConnectedToRoborio()) {
            Logger.recordOutput(RoborioUtilsConstants.ALERT_LOG_PATH + "CanDisconnectAt", currentTime);
        }
        else if (getCANUtilizationPercent() > RoborioUtilsConstants.MAX_CAN_UTILIZATION_PERCENT) {
            Logger.recordOutput(RoborioUtilsConstants.ALERT_LOG_PATH + "CanFloodedAt", currentTime);
        }
        if (getCurrentRoborioCycle() > getDefaultTimeStep() + RoborioUtilsConstants.TIME_STEP_TOLERANCE) {
            Logger.recordOutput(RoborioUtilsConstants.ALERT_LOG_PATH + "CycleOverrunAt", currentTime);
        }
    }

    public static double getDefaultTimeStep() {
        return RoborioUtilsConstants.DEFAULT_TIME_STEP;
    }

    public static double getCurrentRoborioCycle() {
        return currentTime - lastTime;
    }

    public static boolean isCANConnectedToRoborio() {
        return getCANUtilizationPercent() > 0;
    }

    public static double getCANUtilizationPercent() {
        return RobotController.getCANStatus().percentBusUtilization * 100;
    }

}
package frc.utils;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class RoborioUtils {

    private static double lastTime;

    private static double currentTime;

    public static void updateCurrentCycleTime() {
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
    }

    public static double getCurrentRoborioCycle() {
        //Todo - add Alert Log - maybe in periodic maybe in call
        return currentTime - lastTime;
    }

    public static boolean isCANConnectedToRoborio() {
        //Todo - add Alert Log - maybe in periodic maybe in call
        return getCANUtilizationPercent() > 0;
    }

    public static double getCANUtilizationPercent() {
        //Todo - add Alert Log - maybe in periodic maybe in call
        return RobotController.getCANStatus().percentBusUtilization * 100;
    }

}
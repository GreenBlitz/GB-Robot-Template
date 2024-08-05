package frc.robot.subsystems.swerve.gyro;


import frc.robot.constants.LogPaths;

public record SwerveGyroConstants(String logPath, String alertLogPath) {

    public SwerveGyroConstants (String swerveLogPath) {
        this(swerveLogPath + "Gyro/", LogPaths.ALERT_LOG_PATH + swerveLogPath + "Gyro/");
    }

}

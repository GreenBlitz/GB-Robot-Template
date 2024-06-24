package frc.utils.loggerutils;

public enum LogSaveSpot {

    USB(LoggerConstants.USB_LOG_PATH),
    ROBORIO(LoggerConstants.SAFE_ROBORIO_LOG_PATH),
    COMPUTER(LoggerConstants.SIMULATION_LOG_PATH);

    public final String savePath;

    LogSaveSpot(String savePath) {
        this.savePath = savePath;
    }
}

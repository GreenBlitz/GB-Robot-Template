package frc.utils;

public abstract class MotorSubsystem extends GBSubsystem{
    public MotorSubsystem(String logPath) {
        super(logPath);
    }

    public void setBrake(boolean brake){}

}

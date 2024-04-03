package frc.robot.subsystems;

public interface IMotorSubsystem {

    boolean isAtPosition(double targetPositionUnits);

    double getVelocity();
    double getPosition();

    void setVelocityControl();
    void setPositionControl(double targetPositionUnits);

    double getPValue(int pidSlot);
    void setPValue(double value, int pidSlot);

    void stop();



}

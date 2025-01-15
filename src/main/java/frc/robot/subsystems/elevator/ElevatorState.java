package frc.robot.subsystems.elevator;

public enum ElevatorState {

    FEEDER(0.5),
    L1(0.1),
    L2(0.2),
    L3(0.3),
    L4(0.4),
    CLOSED(0.05),
    IDLE(0);

    private double meters;

    ElevatorState(double meters) {
        this.meters = meters;
    }

    public double getMeters() {
        return meters;
    }

}
package frc.robot.subsystems.elevator;

public enum ElevatorState {

    FEEDER(5),
    L1(1),
    L2(2),
    L3(3),
    L4(4),
    CLOSED(-1),
    IDLE(0);

    private double meters;

    ElevatorState(double meters) {
        this.meters = meters;
    }

    public double getMeters() {
        return meters;
    }

}
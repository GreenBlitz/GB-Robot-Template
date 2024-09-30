package frc.robot.subsystems.solenoid;

public enum SolenoidState {

    OFF(0),
    RETRACT(0.8),
    HOLD(0.2);


    private final double appliedPower;

    SolenoidState(double appliedPower) {
        this.appliedPower = appliedPower;
    }

    public double getAppliedPower() {
        return appliedPower;
    }
}

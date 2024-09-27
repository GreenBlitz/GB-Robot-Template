package frc.robot.subsystems.roller;

public enum RollerState {

    INTAKE(0.3),
    OUTTAKE(-0.3),
    STOP(0);

    private double power;

    RollerState(double power) {
        this.power = power;
    }

    public double getPower() {
        return power;
    }

}

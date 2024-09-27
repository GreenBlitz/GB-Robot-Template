package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public enum IntakeState {

    INTAKE(0.3),
    OUTTAKE(-0.3),
    DEFAULT(0);

    private final double power;

    IntakeState(double power) {
        this.power = power;
    }

    public double getPower() {
        return power;
    }

}

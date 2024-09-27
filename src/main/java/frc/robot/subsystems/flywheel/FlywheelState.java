package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;

public enum FlywheelState {

    DEFAULT(Rotation2d.fromRotations(30), 0.7),
    PRE_SPEAKER(Rotation2d.fromRotations(60), 0.7);

    private final Rotation2d shootingVelocity;
    private final double differentialRatio;

    FlywheelState(Rotation2d shootingVelocity, double differentialRatio) {
        this.shootingVelocity = shootingVelocity;
        this.differentialRatio = differentialRatio;
    }

    public Rotation2d getRightVelocity() {
        return shootingVelocity;
    }

    public Rotation2d getLeftVelocity() {
        return shootingVelocity.times(differentialRatio);
    }

}

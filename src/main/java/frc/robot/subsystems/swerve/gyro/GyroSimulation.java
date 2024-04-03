package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroSimulation {
    private double simulationRadians = 0;

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromRadians(simulationRadians);
    }

    public void update(double omegaRadiansPerSecond, double movementTimeSeconds) {
        simulationRadians += omegaRadiansPerSecond * movementTimeSeconds;
    }

    public void setHeading(Rotation2d heading) {
        simulationRadians = heading.getRadians();
    }
}

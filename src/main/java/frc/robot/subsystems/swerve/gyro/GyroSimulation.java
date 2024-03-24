package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroSimulation {
    private double simulationRadians = 0;

    public double getGyroYawDegrees() {
        return Math.toDegrees(simulationRadians);
    }

    public void update(double omegaRadiansPerSecond, double timeSeconds) {
        simulationRadians += omegaRadiansPerSecond * timeSeconds;
    }

    public void setHeading(Rotation2d heading) {
        simulationRadians = heading.getRadians();
    }
}

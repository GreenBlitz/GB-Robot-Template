package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroSimulation {

    private double simulationRadians = 0;

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromRadians(simulationRadians);
    }

    public void update(Rotation2d omegaPerSecond, double timeSeconds) {
        simulationRadians += omegaPerSecond.getRadians() * timeSeconds;
    }

    public void setHeading(Rotation2d heading) {
        simulationRadians = heading.getRadians();
    }

}
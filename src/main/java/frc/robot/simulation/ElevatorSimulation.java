package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.utils.Conversions;
import frc.utils.cycletimeutils.CycleTimeUtils;

public class ElevatorSimulation extends MotorSimulation {

    private final ElevatorSim elevatorSimulation;

    private final double diameterMeters;

    public ElevatorSimulation(ElevatorSim elevatorSimulation, double drumRadiusMeters) {
        this.elevatorSimulation = elevatorSimulation;
        this.diameterMeters = 2 * drumRadiusMeters;
    }

    public double getCurrent() {
        return elevatorSimulation.getCurrentDrawAmps();
    }

    /**
     * Returns in Rotation2D the position of the drum
     *
     * @return the position
     */
    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(Conversions.distanceToRevolutions(getPositionMeters(), diameterMeters));
    }

    public double getPositionMeters() {
        return elevatorSimulation.getPositionMeters();
    }

    /**
     * Returns the velocity in Rotation2D of the drum
     *
     * @return the velocity
     */
    @Override
    public Rotation2d getVelocity() {
        return Rotation2d.fromRotations(Conversions.distanceToRevolutions(getVelocityMetersPerSecond(), diameterMeters));
    }

    public double getVelocityMetersPerSecond() {
        return elevatorSimulation.getVelocityMetersPerSecond();
    }

    @Override
    protected void setInputVoltage(double voltage) {
        elevatorSimulation.setInputVoltage(voltage);
    }

    @Override
    protected void updateMotor() {
        elevatorSimulation.update(CycleTimeUtils.getCurrentCycleTime());
    }

}

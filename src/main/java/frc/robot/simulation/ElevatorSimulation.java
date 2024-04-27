package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.SimulationConstants;
import frc.utils.Conversions;

public class ElevatorSimulation extends MotorSimulation {

    private final ElevatorSim elevatorSimulation;

    private final double diameterMeters;

    public ElevatorSimulation(DCMotor gearbox, double gearRatio, double carriageMassKilograms, double drumRadiusMeters,
            double minimumHeightMeters, double maximumHeightMeters, double startingHeightMeters, boolean simulateGravity) {
        this.diameterMeters = 2 * drumRadiusMeters;
        this.elevatorSimulation = new ElevatorSim(
                gearbox,
                gearRatio,
                carriageMassKilograms,
                drumRadiusMeters,
                minimumHeightMeters,
                maximumHeightMeters,
                simulateGravity,
                startingHeightMeters
        );
    }

    @Override
    public double getCurrent() {
        return elevatorSimulation.getCurrentDrawAmps();
    }

    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(
                Conversions.distanceToRevolutions(
                        elevatorSimulation.getPositionMeters(),
                        diameterMeters
                )
        );
    }

    @Override
    public Rotation2d getVelocity() {
        return Rotation2d.fromRotations(
                Conversions.distanceToRevolutions(
                        elevatorSimulation.getVelocityMetersPerSecond(),
                        diameterMeters
                )
        );
    }

    @Override
    protected void setInputVoltage(double voltage) {
        elevatorSimulation.setInputVoltage(voltage);
    }

    @Override
    protected void updateMotor() {
        elevatorSimulation.update(SimulationConstants.TIME_STEP);
    }

}
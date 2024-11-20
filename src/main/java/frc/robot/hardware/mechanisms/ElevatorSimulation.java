package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.utils.Conversions;
import frc.utils.time.TimeUtils;

public class ElevatorSimulation extends MechanismSimulation{

    private final ElevatorSim elevatorSim;
    private final double elevatorMetersToRotationsConversionFactor;

    public ElevatorSimulation(ElevatorSim elevatorSim, double gearRatio, double elevatorMetersToRotationsConversionFactor) {
        super(gearRatio);
        this.elevatorSim = elevatorSim;
        this.elevatorMetersToRotationsConversionFactor = elevatorMetersToRotationsConversionFactor;
    }

    @Override
    public Rotation2d getSystemPosition() {
        return metersToElevatorRotations(elevatorSim.getPositionMeters());
    }

    @Override
    public Rotation2d getSystemVelocityAnglesPerSecond() {
        return metersToElevatorRotations(elevatorSim.getVelocityMetersPerSecond());
    }

    @Override
    public void setInputVoltage(double voltage) {
        elevatorSim.setInputVoltage(voltage);
    }

    public boolean hasHitLowerLimit(){
        return elevatorSim.hasHitLowerLimit();
    }

    public boolean hasHitUpperLimit(){
        return elevatorSim.hasHitUpperLimit();
    }

    public boolean wouldHitLowerLimit(Rotation2d position){
        return elevatorSim.wouldHitLowerLimit(elevatorRotationsToMeters(position));
    }

    public boolean wouldHitUpperLimit(Rotation2d position){
        return elevatorSim.wouldHitUpperLimit(elevatorRotationsToMeters(position));
    }

    @Override
    public void updateMotor() {
        elevatorSim.update(TimeUtils.getCurrentCycleTimeSeconds());
    }

    public Rotation2d metersToElevatorRotations(double meters) {
        return Rotation2d.fromRotations(meters * elevatorMetersToRotationsConversionFactor);
    }

    public double elevatorRotationsToMeters(Rotation2d rotations){
        return rotations.getRotations() / elevatorMetersToRotationsConversionFactor;
    }

}

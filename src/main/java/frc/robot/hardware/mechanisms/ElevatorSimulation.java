package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.utils.Conversions;
import frc.utils.time.TimeUtils;

public class ElevatorSimulation extends MechanismSimulation{

    private final ElevatorSim elevatorSim;
    private final double gearRatio;

    public ElevatorSimulation(ElevatorSim elevatorSim, double gearRatio) {
        super(gearRatio);
        this.elevatorSim = elevatorSim;
        this.gearRatio = gearRatio;
        elevatorSim.
    }

    @Override
    public Rotation2d getSystemPosition() {
        return Conversions.distanceToAngle(elevatorSim.getPositionMeters(), gearRatio);
    }

    @Override
    public Rotation2d getSystemVelocityAnglesPerSecond() {
        return Conversions.distanceToAngle(elevatorSim.getVelocityMetersPerSecond(), gearRatio);
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
        return elevatorSim.wouldHitLowerLimit(Conversions.angleToDistance(position, gearRatio));
    }

    @Override
    public void updateMotor() {
        elevatorSim.update(TimeUtils.getCurrentCycleTimeSeconds());
    }
}

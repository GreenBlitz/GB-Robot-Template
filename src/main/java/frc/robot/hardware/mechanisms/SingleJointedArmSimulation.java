package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.utils.time.TimeUtils;

public class SingleJointedArmSimulation extends MechanismSimulation{

    private final SingleJointedArmSim singleJointedArmSim;

    public SingleJointedArmSimulation(SingleJointedArmSim singleJointedArmSim, double gearRatio) {
        super(gearRatio);
        this.singleJointedArmSim = singleJointedArmSim;
    }

    @Override
    public Rotation2d getSystemPosition() {
        return Rotation2d.fromRadians(singleJointedArmSim.getAngleRads());
    }

    @Override
    public Rotation2d getSystemVelocityAnglesPerSecond() {
        return Rotation2d.fromRadians(singleJointedArmSim.getVelocityRadPerSec());
    }

    @Override
    public void setInputVoltage(double voltage) {
        singleJointedArmSim.setInputVoltage(voltage);
    }

    public boolean hasHitLowerLimit(){
        return singleJointedArmSim.hasHitLowerLimit();
    }

    public boolean hasHitUpperLimit(){
        return singleJointedArmSim.hasHitUpperLimit();
    }

    @Override
    public void updateMotor() {
        singleJointedArmSim.update(TimeUtils.getCurrentCycleTimeSeconds());
    }
}

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.utils.cycletimeutils.CycleTimeUtils;

public class SingleJointedArmSimulation extends MotorSimulation {

    private final SingleJointedArmSim armSimulation;

    public SingleJointedArmSimulation(DCMotor gearbox, double gearRatio, double armLengthMeters, double armMassKilograms,
            Rotation2d minimumAngle, Rotation2d maximumAngle, Rotation2d startingAngle, boolean simulateGravity) {
        this.armSimulation = new SingleJointedArmSim(
                gearbox,
                gearRatio,
                SingleJointedArmSim.estimateMOI(armLengthMeters, armMassKilograms),
                armLengthMeters,
                minimumAngle.getRadians(),
                maximumAngle.getRadians(),
                simulateGravity,
                startingAngle.getRadians()
        );
    }

    @Override
    public double getCurrent() {
        return armSimulation.getCurrentDrawAmps();
    }

    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromRadians(armSimulation.getAngleRads());
    }

    @Override
    public Rotation2d getVelocity() {
        return Rotation2d.fromRadians(armSimulation.getVelocityRadPerSec());
    }

    @Override
    protected void setInputVoltage(double voltage) {
        armSimulation.setInputVoltage(voltage);
    }

    @Override
    protected void updateMotor() {
        armSimulation.update(CycleTimeUtils.getDefaultCycleTime());
    }

}

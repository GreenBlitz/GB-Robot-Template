package frc.robot.hardware.simulations.wpilib;

import frc.utils.costomUnits.Angle2d;
import org.wpilib.simulation.SingleJointedArmSim;
import frc.utils.time.TimeUtil;

public class SingleJointedArmSimulation implements WPILibMechanismSimulation {

	private final SingleJointedArmSim singleJointedArmSimulation;
	private final double gearRatio;

	public SingleJointedArmSimulation(SingleJointedArmSim singleJointedArmSimulation, double gearRatio) {
		this.singleJointedArmSimulation = singleJointedArmSimulation;
		this.gearRatio = gearRatio;
	}

	@Override
	public Angle2d getMechanismPosition() {
		return Angle2d.fromRadians(singleJointedArmSimulation.getAngle());
	}

	@Override
	public Angle2d getMechanismVelocityRPS() {
		return Angle2d.fromRadians(singleJointedArmSimulation.getVelocity());
	}

	@Override
	public void setInputVoltage(double voltage) {
		singleJointedArmSimulation.setInputVoltage(voltage);
	}

	@Override
	public void updateMotor() {
		singleJointedArmSimulation.update(TimeUtil.getLatestCycleTimeSeconds());
	}

	@Override
	public double getGearRatio() {
		return gearRatio;
	}

}

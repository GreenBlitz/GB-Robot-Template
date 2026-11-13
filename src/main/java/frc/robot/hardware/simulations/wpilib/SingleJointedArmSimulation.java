package frc.robot.hardware.simulations.wpilib;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.utils.angle2d.Angle2d;
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
		return Angle2d.fromRadians(singleJointedArmSimulation.getAngleRads());
	}

	@Override
	public Angle2d getMechanismVelocityAPS() {
		return Angle2d.fromRadians(singleJointedArmSimulation.getVelocityRadPerSec());
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

package frc.robot.hardware.mechanisms.wpilib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.utils.time.TimeUtil;

public class SingleJointedArmSimulation implements WPILibMechanismSimulation {

	private final SingleJointedArmSim singleJointedArmSimulation;
	private final double gearRatio;

	public SingleJointedArmSimulation(SingleJointedArmSim singleJointedArmSimulation, double gearRatio) {
		this.singleJointedArmSimulation = singleJointedArmSimulation;
		this.gearRatio = gearRatio;
	}

	@Override
	public Rotation2d getMechanismPosition() {
		return Rotation2d.fromRadians(singleJointedArmSimulation.getAngleRads());
	}

	@Override
	public Rotation2d getMechanismVelocityAnglesPerSecond() {
		return Rotation2d.fromRadians(singleJointedArmSimulation.getVelocityRadPerSec());
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

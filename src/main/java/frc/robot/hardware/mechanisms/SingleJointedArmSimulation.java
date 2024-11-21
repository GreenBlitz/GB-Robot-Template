package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.utils.time.TimeUtils;

public class SingleJointedArmSimulation extends MechanismSimulation {

	private final SingleJointedArmSim singleJointedArmSimulation;

	public SingleJointedArmSimulation(SingleJointedArmSim singleJointedArmSimulation, double gearRatio) {
		super(gearRatio);
		this.singleJointedArmSimulation = singleJointedArmSimulation;
	}

	@Override
	public Rotation2d getSystemPosition() {
		return Rotation2d.fromRadians(singleJointedArmSimulation.getAngleRads());
	}

	@Override
	public Rotation2d getSystemVelocityAnglesPerSecond() {
		return Rotation2d.fromRadians(singleJointedArmSimulation.getVelocityRadPerSec());
	}

	@Override
	public void setInputVoltage(double voltage) {
		singleJointedArmSimulation.setInputVoltage(voltage);
	}

	@Override
	public void updateMotor() {
		singleJointedArmSimulation.update(TimeUtils.getCurrentCycleTimeSeconds());
	}

}

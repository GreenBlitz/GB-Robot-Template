package frc.robot.hardware.rev.motors.simulation;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.hardware.mechanisms.MechanismSimulation;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.utils.Conversions;
import frc.utils.battery.BatteryUtil;
import frc.utils.time.TimeUtil;

public class SparkMaxSimulation {

	private static final int DEFAULT_NUMBER_OF_MOTORS = 1;

	private final SparkMaxSim simulation;
	private final MechanismSimulation mechanismSimulation;

	public SparkMaxSimulation(SparkMaxWrapper sparkMaxWrapper, MechanismSimulation mechanismSimulation) {
		this(sparkMaxWrapper, mechanismSimulation, DCMotor.getNEO(DEFAULT_NUMBER_OF_MOTORS));
	}

	public SparkMaxSimulation(SparkMaxWrapper sparkMaxWrapper, MechanismSimulation mechanismSimulation, DCMotor simulationMotor) {
		this.simulation = new SparkMaxSim(sparkMaxWrapper, simulationMotor);
		this.mechanismSimulation = mechanismSimulation;
	}

	public double getVoltage() {
		return simulation.getAppliedOutput() * simulation.getBusVoltage();
	}

	public void updateMotor() {
		mechanismSimulation.setInputVoltage(getVoltage());
		mechanismSimulation.updateMotor();
		simulation.iterate(
			Conversions.perSecondToPerMinute(mechanismSimulation.getRotorVelocityAnglesPerSecond().getRotations()),
			BatteryUtil.getCurrentVoltage(),
			TimeUtil.getLatestCycleTimeSeconds()
		);
	}

}

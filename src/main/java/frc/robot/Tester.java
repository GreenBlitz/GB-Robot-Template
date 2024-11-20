package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.hardware.mechanisms.ElevatorSimulation;
import frc.robot.hardware.mechanisms.FlywheelSimulation;
import frc.robot.hardware.mechanisms.MechanismSimulation;
import frc.robot.hardware.mechanisms.SingleJointedArmSimulation;
import org.littletonrobotics.junction.Logger;

public class Tester {

	private final String logPath;
	private final MechanismSimulation simulation;

	public enum TESTER_TYPE {

		JOINT("Joint/"),
		FLYWHEEL("Flywheel/"),
		ELEVATOR("Elevator/");

		public final String logPathAddition;

		TESTER_TYPE(String logPathAddition) {
			this.logPathAddition = logPathAddition;
		}

	}

	public Tester(TESTER_TYPE type) {
		this.simulation = create(type);
		this.logPath = "Tester/" + type.logPathAddition;
	}

	private MechanismSimulation create(TESTER_TYPE type) {
		if (type == TESTER_TYPE.JOINT) {
			return createJoint();
		} else if (type == TESTER_TYPE.FLYWHEEL) {
			return createFlywheel();
		} else {
			return createElevator();
		}
	}

	private SingleJointedArmSimulation createJoint() {
		SingleJointedArmSim armSim = new SingleJointedArmSim(
			DCMotor.getFalcon500(1),
			(28.0 * (60.0 / 16.0)),
			SingleJointedArmSim.estimateMOI(0.44, 0.44),
			0.44,
			Rotation2d.fromDegrees(-81).getRadians(),
			Rotation2d.fromDegrees(90).getRadians(),
			false,
			Rotation2d.fromDegrees(0).getRadians()
		);
		return new SingleJointedArmSimulation(armSim, (28.0 * (60.0 / 16.0)));
	}

	private FlywheelSimulation createFlywheel() {
		FlywheelSim sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, 1 / 8.0), DCMotor.getNEO(1));
		return new FlywheelSimulation(sim, 1 / 8.0);
	}

	private ElevatorSimulation createElevator() {
		ElevatorSim sim = new ElevatorSim(DCMotor.getNEO(1), 1, 5, 0.05, 0.1, 1, false, 0.1);
		return new ElevatorSimulation(sim, 1, 1);
	}

	public void setVoltage(double voltage) {
		simulation.setInputVoltage(voltage);
	}

	public void update() {
		simulation.updateMotor();
		updateInputs();
	}

	private void updateInputs() {
		Logger.recordOutput(logPath + "RotorPositionDegrees", simulation.getRotorPosition().getDegrees());
		Logger.recordOutput(logPath + "RotorVelocityRPS", simulation.getRotorVelocityAnglesPerSecond().getRotations());
		Logger.recordOutput(logPath + "SystemPositionDegrees", simulation.getSystemPosition().getDegrees());
		Logger.recordOutput(logPath + "SystemVelocityRPS", simulation.getSystemVelocityAnglesPerSecond().getRotations());
	}

}

package frc.robot.subsystems.swerve.factories.modules.steer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.swerve.modules.steer.simulation.SimulationSteerConstants;

class SteerSimulationConstants {

	private static final double GEAR_RATIO = 150.0 / 7.0;

	private static final double MOMENT_OF_INERTIA = 0.00001;

	private static final boolean ENABLE_FOC = true;

	private static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration();
	static {
		MOTOR_CONFIG.Slot0.kP = 72;
		MOTOR_CONFIG.Slot0.kI = 0;
		MOTOR_CONFIG.Slot0.kD = 0;
		MOTOR_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;
	}

	protected static SimulationSteerConstants getConstants() {
		return new SimulationSteerConstants(new DCMotorSim(DCMotor.getFalcon500Foc(1), GEAR_RATIO, MOMENT_OF_INERTIA), MOTOR_CONFIG, ENABLE_FOC);
	}

}

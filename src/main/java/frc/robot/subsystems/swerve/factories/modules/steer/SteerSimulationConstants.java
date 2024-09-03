package frc.robot.subsystems.swerve.factories.modules.steer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.modules.steer.simulation.SimulationSteerConstants;

class SteerSimulationConstants {

	private static final double GEAR_RATIO = 150.0 / 7.0;

	private static final double MOMENT_OF_INERTIA = 0.00001;

	private static final boolean ENABLE_FOC = true;

	private static TalonFXConfiguration generateMotorConfig() {
		TalonFXConfiguration steerConfig = new TalonFXConfiguration();

		steerConfig.Slot0.kP = 72;
		steerConfig.Slot0.kI = 0;
		steerConfig.Slot0.kD = 0;
		steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

		return steerConfig;
	}

	protected static SimulationSteerConstants getConstants() {
		return new SimulationSteerConstants(
			new DCMotorSim(DCMotor.getFalcon500Foc(1), GEAR_RATIO, MOMENT_OF_INERTIA),
			generateMotorConfig(),
			ENABLE_FOC,
			new SysIdRoutine.Config()
		);
	}

}

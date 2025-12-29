package frc.robot.subsystems.swerve.factories.modules.steer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.swerve.maplewrappers.MapleControllableModuleMotor;
import frc.robot.subsystems.swerve.maplewrappers.requests.MaplePositionRequest;
import frc.robot.subsystems.swerve.maplewrappers.requests.MapleVoltageRequest;
import frc.robot.subsystems.swerve.module.records.SteerRequests;
import frc.robot.subsystems.swerve.module.records.SteerSignals;
import frc.utils.AngleUnit;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

public class MapleSteerBuilder {

	public static final double SLIP_CURRENT = 60;

	static ControllableMotor buildSteer(String logPath, SwerveModuleSimulation moduleSimulation) {
		return new MapleControllableModuleMotor(true, logPath, moduleSimulation, new PIDController(30, 0, 0));
	}

	static SteerRequests buildRequests() {
		return new SteerRequests(new MaplePositionRequest(), new MapleVoltageRequest());
	}

	static SteerSignals buildSignals(MapleControllableModuleMotor motor) {
		return new SteerSignals(
			new SuppliedAngleSignal("position", () -> motor.getModuleSimulation().getSteerAbsoluteFacing().getRadians(), AngleUnit.RADIANS),
			new SuppliedAngleSignal(
				"velocity",
				() -> motor.getModuleSimulation().getSteerAbsoluteEncoderSpeed().in(Units.RadiansPerSecond),
				AngleUnit.RADIANS
			),
			new SuppliedDoubleSignal("current", () -> motor.getModuleSimulation().getSteerMotorStatorCurrent().in(Units.Amp)),
			new SuppliedDoubleSignal("voltage", () -> motor.getModuleSimulation().getSteerMotorAppliedVoltage().in(Units.Volt))

		);
	}

}

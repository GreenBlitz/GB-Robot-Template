package frc.robot.subsystems.swerve.factories.modules.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.swerve.maplewrappers.MapleControllableModuleMotor;
import frc.robot.subsystems.swerve.maplewrappers.requests.MaplePositionRequest;
import frc.robot.subsystems.swerve.maplewrappers.requests.MapleVoltageRequest;
import frc.robot.subsystems.swerve.module.records.DriveRequests;
import frc.robot.subsystems.swerve.module.records.DriveSignals;
import frc.utils.AngleUnit;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

public class MapleDriveBuilder {

	public static final double SLIP_CURRENT = 60;

	static ControllableMotor buildDrive(String logPath, SwerveModuleSimulation moduleSimulation) {
		return new MapleControllableModuleMotor(logPath, moduleSimulation, new PIDController(1, 0, 0));
	}

	static DriveRequests buildRequests() {
		return new DriveRequests(new MaplePositionRequest(), new MapleVoltageRequest(), new MapleVoltageRequest());
	}

	static DriveSignals buildSignals(MapleControllableModuleMotor motor) {
		return new DriveSignals(
			new SuppliedAngleSignal(
				"position",
				() -> motor.getModuleSimulation().getDriveWheelFinalPosition().in(Units.Radians),
				AngleUnit.RADIANS
			),
			new SuppliedAngleSignal(
				"velocity",
				() -> motor.getModuleSimulation().getDriveWheelFinalSpeed().in(Units.RadiansPerSecond),
				AngleUnit.RADIANS
			),
			new SuppliedDoubleSignal("current", () -> motor.getModuleSimulation().getDriveMotorStatorCurrent().in(Units.Amp)),
			new SuppliedDoubleSignal("voltage", () -> motor.getModuleSimulation().getDriveMotorAppliedVoltage().in(Units.Volt))

		);
	}

}

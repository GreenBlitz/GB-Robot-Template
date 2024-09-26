package frc.robot.subsystems.pivot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;

public class PivotRealConstants {


	protected static final SysIdRoutine.Config SYS_ID_CONFIG = new SysIdRoutine.Config(
		Units.Volts.of(1.0).per(Units.Seconds.of(1.0)),
		Units.Volts.of(7.0),
		Units.Seconds.of(10.0)
	);

	protected static TalonFXMotor getTalonFXMotor(String logPath, TalonFXWrapper talonFXWrapper){
		return new TalonFXMotor(
				logPath,
				talonFXWrapper,
				SYS_ID_CONFIG
		);
	}
}

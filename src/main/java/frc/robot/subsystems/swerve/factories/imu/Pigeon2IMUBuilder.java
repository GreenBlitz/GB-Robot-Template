package frc.robot.subsystems.swerve.factories.imu;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.imu.Pigeon2IMU;
import frc.robot.hardware.phoenix6.imu.Pigeon2Wrapper;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.hardware.phoenix6.signal.SignalGetter;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.DoubleSignal;
import frc.robot.subsystems.swerve.IMUSignals;
import frc.utils.alerts.Alert;
import frc.utils.AngleUnit;

class Pigeon2IMUBuilder {

	private static final int APPLY_CONFIG_RETRIES = 5;

	private static Pigeon2Configuration buildIMUConfig() {
		Pigeon2Configuration imuConfig = new Pigeon2Configuration();
		imuConfig.MountPose.MountPoseYaw = 90.2035903930664;
		imuConfig.MountPose.MountPosePitch = 0.6566112637519836;
		imuConfig.MountPose.MountPoseRoll = -2.0430026054382324;
		return imuConfig;
	}

	static IIMU buildIMU(String logPath) {
		Pigeon2Wrapper pigeon2Wrapper = new Pigeon2Wrapper(IDs.Pigeon2IDs.SWERVE);
		Pigeon2Configuration pigeon2Configuration = buildIMUConfig();

		if (!pigeon2Wrapper.applyConfiguration(pigeon2Configuration, APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, logPath + "ConfigurationFailAt").report();
		}

		return new Pigeon2IMU(logPath, pigeon2Wrapper);
	}

	private static Phoenix6AngleSignal buildAnglePigeonSignal(StatusSignal<?> signal) {
		return Phoenix6SignalBuilder.build(signal, RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.DEGREES, BusChain.ROBORIO);
	}

	private static AngleSignal buildAnglePigeonSignal(StatusSignal<?> signal, SignalGetter signalSlope) {
		return Phoenix6SignalBuilder
			.build(signal, signalSlope, RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.DEGREES, BusChain.ROBORIO);
	}

	private static DoubleSignal buildDoublePigeonSignal(StatusSignal<?> signal) {
		return Phoenix6SignalBuilder.build(signal, RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);
	}

	static IMUSignals buildSignals(Pigeon2IMU pigeon2imu) {
		Phoenix6AngleSignal angularVelocityXWorld = buildAnglePigeonSignal(pigeon2imu.getDevice().getAngularVelocityXWorld());
		Phoenix6AngleSignal angularVelocityYWorld = buildAnglePigeonSignal(pigeon2imu.getDevice().getAngularVelocityYWorld());
		Phoenix6AngleSignal angularVelocityZWorld = buildAnglePigeonSignal(pigeon2imu.getDevice().getAngularVelocityZWorld());
		return new IMUSignals(
			buildAnglePigeonSignal(pigeon2imu.getDevice().getRoll(), angularVelocityXWorld),
			buildAnglePigeonSignal(pigeon2imu.getDevice().getPitch(), angularVelocityYWorld),
			buildAnglePigeonSignal(pigeon2imu.getDevice().getYaw(), angularVelocityZWorld),
			angularVelocityXWorld,
			angularVelocityYWorld,
			angularVelocityZWorld,
			buildDoublePigeonSignal(pigeon2imu.getDevice().getAccelerationX()),
			buildDoublePigeonSignal(pigeon2imu.getDevice().getAccelerationY()),
			buildDoublePigeonSignal(pigeon2imu.getDevice().getAccelerationZ())
		);
	}

}

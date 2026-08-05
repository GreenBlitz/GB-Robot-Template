package frc.robot.subsystems.swerve.factories.imu;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.imu.Pigeon2IMU;
import frc.robot.hardware.phoenix6.imu.Pigeon2Wrapper;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.hardware.phoenix6.signal.SignalGetter;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.DoubleSignal;
import frc.robot.subsystems.swerve.IMUSignals;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;

public class Pigeon2IMUBuilder {

	private static final int APPLY_CONFIG_RETRIES = 5;

	private static Pigeon2Configuration buildIMUConfig() {
		Pigeon2Configuration imuConfig = new Pigeon2Configuration();
		imuConfig.MountPose.MountPoseYaw = -91.42034149169922;
		imuConfig.MountPose.MountPosePitch = 0.3520643711090088;
		imuConfig.MountPose.MountPoseRoll = -1.9781156778335571;
		return imuConfig;
	}

	static IIMU buildIMU(String logPath, Phoenix6DeviceID id) {
		Pigeon2Wrapper pigeon2Wrapper = new Pigeon2Wrapper(id);
		Pigeon2Configuration pigeon2Configuration = buildIMUConfig();

		if (!pigeon2Wrapper.applyConfiguration(pigeon2Configuration, APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, logPath + "ConfigurationFailAt").report();
		}

		return new Pigeon2IMU(logPath, pigeon2Wrapper);
	}

	private static Phoenix6AngleSignal buildAnglePigeonSignal(StatusSignal<?> signal, BusChain busChain) {
		return Phoenix6SignalBuilder.build(signal, RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.DEGREES, busChain);
	}

	private static AngleSignal buildAnglePigeonSignal(StatusSignal<?> signal, SignalGetter signalSlope, BusChain busChain) {
		return Phoenix6SignalBuilder.build(signal, signalSlope, RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.DEGREES, busChain);
	}

	private static DoubleSignal buildDoublePigeonSignal(StatusSignal<?> signal, BusChain busChain) {
		return Phoenix6SignalBuilder.build(signal, RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, busChain);
	}

	static IMUSignals buildSignals(Pigeon2IMU pigeon2imu) {
		Phoenix6AngleSignal angularVelocityXWorld = buildAnglePigeonSignal(
			pigeon2imu.getDevice().getAngularVelocityXWorld(),
			pigeon2imu.getDevice().getBusChain()
		);
		Phoenix6AngleSignal angularVelocityYWorld = buildAnglePigeonSignal(
			pigeon2imu.getDevice().getAngularVelocityYWorld(),
			pigeon2imu.getDevice().getBusChain()
		);
		Phoenix6AngleSignal angularVelocityZWorld = buildAnglePigeonSignal(
			pigeon2imu.getDevice().getAngularVelocityZWorld(),
			pigeon2imu.getDevice().getBusChain()
		);
		return new IMUSignals(
			buildAnglePigeonSignal(pigeon2imu.getDevice().getRoll(), angularVelocityXWorld, pigeon2imu.getDevice().getBusChain()),
			buildAnglePigeonSignal(pigeon2imu.getDevice().getPitch(), angularVelocityYWorld, pigeon2imu.getDevice().getBusChain()),
			buildAnglePigeonSignal(pigeon2imu.getDevice().getYaw(), angularVelocityZWorld, pigeon2imu.getDevice().getBusChain()),
			angularVelocityXWorld,
			angularVelocityYWorld,
			angularVelocityZWorld,
			buildDoublePigeonSignal(pigeon2imu.getDevice().getAccelerationX(), pigeon2imu.getDevice().getBusChain()),
			buildDoublePigeonSignal(pigeon2imu.getDevice().getAccelerationY(), pigeon2imu.getDevice().getBusChain()),
			buildDoublePigeonSignal(pigeon2imu.getDevice().getAccelerationZ(), pigeon2imu.getDevice().getBusChain())
		);
	}

}

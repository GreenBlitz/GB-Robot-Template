// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.IDynamicMotionMagicRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6LatencySignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.alerts.AlertManager;
import frc.utils.DriverStationUtil;
import frc.utils.math.AngleUnit;
import frc.utils.time.TimeUtil;
import frc.utils.logger.LoggerFactory;
import org.littletonrobotics.junction.LoggedRobot;
import frc.utils.brakestate.BrakeStateManager;
import org.littletonrobotics.junction.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in
 * the project.
 */
public class RobotManager extends LoggedRobot {

	private final Robot robot;
	private Command auto;
	private int roborioCycles;

	private final TalonFXMotor motor = new TalonFXMotor(
		"Tester",
		new Phoenix6DeviceID(9),
		new SysIdRoutine.Config(),
		new SimpleMotorSimulation(
			new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, 12.8), DCMotor.getKrakenX60Foc(1))
		)
	);
	private final IDynamicMotionMagicRequest dynamicMotionMagicRequest = Phoenix6RequestBuilder.build(
		new DynamicMotionMagicVoltage(0, 0, 0, 0), 0, true
	);


	private final Phoenix6AngleSignal acc = Phoenix6SignalBuilder.build(motor.getDevice().getAcceleration(), 60,
		AngleUnit.ROTATIONS);

	private final Phoenix6AngleSignal vel = Phoenix6SignalBuilder.build(motor.getDevice().getVelocity(), 60, AngleUnit.ROTATIONS);
	private final Phoenix6AngleSignal pos = Phoenix6SignalBuilder.build(motor.getDevice().getPosition(), 60, AngleUnit.ROTATIONS);
	private final InputSignal<Double> volt = Phoenix6SignalBuilder.build(motor.getDevice().getMotorVoltage(), 60);

	public RobotManager() {
		LoggerFactory.initializeLogger();
		PathPlannerUtil.startPathfinder();

		this.roborioCycles = 0;
		this.robot = new Robot();

		TalonFXConfiguration configuration = new TalonFXConfiguration();
		configuration.Slot0.kV = 0.7;
		configuration.Slot0.kA = 0.4;
		configuration.Slot0.kS = 0.25;
		configuration.Feedback.SensorToMechanismRatio = 12.8;
		motor.applyConfiguration(configuration);

		JoysticksBindings.configureBindings(robot);
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.coast();
		}
	}

	@Override
	public void disabledExit() {
		BrakeStateManager.brake();
	}

	@Override
	public void autonomousInit() {
		robot.getRobotCommander().removeDefaultCommand();

//		this.auto = robot.getAuto();
//		auto.schedule();

		motor.applyRequest(dynamicMotionMagicRequest
			.withSetPoint(Rotation2d.fromRotations(100))
			.withMaxVelocityRotation2dPerSecond(Rotation2d.fromRotations(50))
			.withMaxAccelerationRotation2dPerSecondSquared(Rotation2d.fromRotations(50)));
	}

	@Override
	public void teleopInit() {
		if (auto != null) {
			auto.cancel();
		}
//		robot.getRobotCommander().initializeDefaultCommand();

		motor.applyRequest(dynamicMotionMagicRequest
			.withSetPoint(Rotation2d.fromRotations(-150))
			.withMaxVelocityRotation2dPerSecond(Rotation2d.fromRotations(2))
			.withMaxAccelerationRotation2dPerSecondSquared(Rotation2d.fromRotations(2)));
	}

	@Override
	public void testInit() {
		motor.applyRequest(dynamicMotionMagicRequest
			.withSetPoint(Rotation2d.fromRotations(0))
			.withMaxVelocityRotation2dPerSecond(Rotation2d.fromRotations(50))
			.withMaxAccelerationRotation2dPerSecondSquared(Rotation2d.fromRotations(2)));
	}

	@Override
	public void robotPeriodic() {
		updateTimeRelatedData(); // Better to be first
		JoysticksBindings.setDriversInputsToSwerve(robot.getSwerve());
		robot.periodic();
		motor.updateSimulation();
		motor.updateInputs(acc, pos, vel, volt);
		AlertManager.reportAlerts();
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtil.updateCycleTime(roborioCycles);
	}

}

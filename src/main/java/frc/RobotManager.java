// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6FeedForwardRequest;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.alerts.AlertManager;
import frc.utils.DriverStationUtil;
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
	private Command autonomousCommand;
	private int roborioCycles;

	private final TalonFXMotor motor = new TalonFXMotor("Tester", new Phoenix6DeviceID(0), new SysIdRoutine.Config(), new SimpleMotorSimulation(
		new DCMotorSim(
			LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1),
			DCMotor.getKrakenX60(1)
		)
	));


	private final Phoenix6FeedForwardRequest request = Phoenix6RequestBuilder.build(new PositionVoltage(0), 0, false);
	private final PositionVoltage positionVoltage = new PositionVoltage(0);
	private final Phoenix6AngleSignal pos = Phoenix6SignalBuilder.build(motor.getDevice().getPosition(), 60, AngleUnit.ROTATIONS);
	private final Phoenix6DoubleSignal volt = Phoenix6SignalBuilder.build(motor.getDevice().getMotorVoltage(), 60);
	public RobotManager() {
		LoggerFactory.initializeLogger();
		PathPlannerUtil.startPathfinder();

		this.roborioCycles = 0;
		this.robot = new Robot();

//		motor.applyConfiguration(new TalonFXConfiguration());

		JoysticksBindings.configureBindings(robot);
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.coast();
		}

		motor.getDevice().setControl(positionVoltage.withFeedForward(0).withPosition(0));
	}

	@Override
	public void disabledExit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.brake();
		}
	}

	@Override
	public void autonomousInit() {
		this.autonomousCommand = robot.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}

		motor.getDevice().setControl(positionVoltage.withFeedForward(8));

	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

		motor.getDevice().setControl(positionVoltage.withFeedForward(12));
	}

	@Override
	public void testInit() {
		motor.getDevice().setControl(positionVoltage.withFeedForward(-7));
	}

	@Override
	public void robotPeriodic() {
		updateTimeRelatedData(); // Better to be first
		robot.periodic();
		motor.updateSimulation();
		motor.updateInputs(pos, volt);
		Logger.recordOutput("FFFFF", positionVoltage.getFeedForwardMeasure().in(Units.Volts));
		AlertManager.reportAlerts();
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtil.updateCycleTime(roborioCycles);
	}

}

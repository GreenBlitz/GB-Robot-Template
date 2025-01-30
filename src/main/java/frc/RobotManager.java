// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.rev.motors.BrushlessSparkMAXMotor;
import frc.robot.hardware.rev.motors.SparkMaxConfiguration;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.robot.hardware.rev.request.SparkMaxRequest;
import frc.robot.hardware.rev.request.SparkMaxRequestBuilder;
import frc.robot.hardware.rev.request.SparkMaxVelocityRequest;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.utils.AngleUnit;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.alerts.AlertManager;
import frc.utils.DriverStationUtils;
import frc.utils.time.TimeUtils;
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

	BrushlessSparkMAXMotor motor;
	SuppliedDoubleSignal velocitySignal;
	SuppliedDoubleSignal voltageSignal;

	public RobotManager() {
		LoggerFactory.initializeLogger();
		PathPlannerUtils.startPathfinder();

		this.roborioCycles = 0;
		this.robot = new Robot();

		SimpleMotorSimulation simulation = new SimpleMotorSimulation(
			new DCMotorSim(LinearSystemId.createDCMotorSystem(
				DCMotor.getNEO(1), 0.001, 1), DCMotor.getNEO(1))
		);
		SparkMaxWrapper motorWrapper = new SparkMaxWrapper(new SparkMaxDeviceID(1));
		motor = new BrushlessSparkMAXMotor(
			"/motor",
			motorWrapper,
			simulation,
			new SysIdRoutine.Config()
		);
		SparkMaxConfig config = new SparkMaxConfig();
		 config.closedLoop.p(2);
		motor.applyConfiguration(new SparkMaxConfiguration().withSparkMaxConfig(config));
		velocitySignal = new SuppliedDoubleSignal("velocity", ()-> motorWrapper.getVelocityAnglePerSecond().getRotations());
		voltageSignal = new SuppliedDoubleSignal("voltage", motorWrapper::getVoltage);
		JoysticksBindings.configureBindings(robot);
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtils.isMatch()) {
			BrakeStateManager.coast();
		}
	}

	@Override
	public void disabledExit() {
		if (!DriverStationUtils.isMatch()) {
			BrakeStateManager.brake();
		}
	}

	@Override
	public void autonomousInit() {
		this.autonomousCommand = robot.getAutonomousCommand();
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		motor.applyRequest(SparkMaxRequestBuilder.build(Rotation2d.fromRotations(200), 0, Rotation2d::getRotations));
	}

	@Override
	public void robotPeriodic() {
		updateTimeRelatedData(); // Better to be first
		robot.periodic();
		AlertManager.reportAlerts();
		motor.updateSimulation();
		motor.updateInputs(velocitySignal);
		motor.updateInputs(voltageSignal);
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtils.updateCycleTime(roborioCycles);
	}

}

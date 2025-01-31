// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.RobotManager;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.chooser.ChooserDigitalInput;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6FeedForwardRequest;
import frc.robot.hardware.phoenix6.request.Phoenix6FeedForwardRequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.battery.BatteryUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();
	public final TalonFXMotor motor;
	public final Phoenix6FeedForwardRequest FFRequest;
	public final Phoenix6DoubleSignal signal;
	public final ChooserDigitalInput digitalInput;
	public final DigitalInputInputsAutoLogged digitalInputInputsInputs;

	public Robot() {
		BatteryUtil.scheduleLimiter();

		SimpleMotorSimulation simulation = new SimpleMotorSimulation(
			new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.00001, 1.0), DCMotor.getKrakenX60Foc(1))
		);

		this.motor = new TalonFXMotor("motor", new Phoenix6DeviceID(1), new SysIdRoutine.Config(), simulation);

		TalonFXConfiguration configuration = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(1));

		signal = Phoenix6SignalBuilder.build(motor.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		digitalInput = new ChooserDigitalInput("tester");
		digitalInputInputsInputs = new DigitalInputInputsAutoLogged();

		FFRequest = Phoenix6FeedForwardRequestBuilder.build(new PositionVoltage(0), 3, () -> digitalInputInputsInputs.nonDebouncedValue);
	}

	public void periodic() {
		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		digitalInput.updateInputs(digitalInputInputsInputs);
		motor.updateSimulation();
		motor.updateInputs(signal);
		CommandScheduler.getInstance().run(); // Should be last
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

}

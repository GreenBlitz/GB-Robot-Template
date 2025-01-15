// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.RobotManager;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.examplearm.ExampleArm;
import frc.robot.subsystems.examplearm.ExampleArmStateHandler;
import frc.robot.subsystems.factories.examplearm.ExampleArmFactory;
import frc.utils.AngleUnit;
import frc.utils.battery.BatteryUtils;

import static frc.robot.subsystems.factories.examplearm.ExampleTalonFXArmBuilder.kGEAR_RATIO;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.REAL; //RobotType.determineRobotType();
	public static final ExampleArm exampleArm = ExampleArmFactory.create("Subsystems/ExampleArm/");
	public static final ExampleArmStateHandler exampleArmStateHandler = new ExampleArmStateHandler(exampleArm);

	public Robot() {
		BatteryUtils.scheduleLimiter();
	}

	public void periodic() {
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

}

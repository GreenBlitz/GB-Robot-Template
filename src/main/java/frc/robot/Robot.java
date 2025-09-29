// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.rev.motors.SparkMaxConfiguration;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.battery.BatteryUtil;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();
	public static final int TEAM_NUMBER = 0;

	private final GBSubsystem subsystem;
	private final SparkMaxWrapper endEffector;
	private DifferentialDrive tankDrive;

	public Robot() {
		BatteryUtil.scheduleLimiter();
		subsystem = new GBSubsystem("EndEffector") {};
		endEffector = new SparkMaxWrapper(new SparkMaxDeviceID(4));
		SparkMaxConfig config = new SparkMaxConfig();
		config.closedLoop.p(1);
		endEffector.applyConfiguration(new SparkMaxConfiguration().withSparkMaxConfig(config));
		switch (TEAM_NUMBER) {
			case 0, 1 -> {
				TalonSRX frontLeft = new TalonSRX(3);
				TalonSRX rearLeft = new TalonSRX(4);
				TalonSRX frontRight = new TalonSRX(1);
				TalonSRX rearRight = new TalonSRX(2);
				tankDrive = new DifferentialDrive(power -> {
					frontLeft.set(ControlMode.PercentOutput, power);
					rearLeft.set(ControlMode.PercentOutput, power);
				}, power -> {
					frontRight.set(ControlMode.PercentOutput, -power);
					rearRight.set(ControlMode.PercentOutput, -power);
				});
			}
			case 2 -> {
				PWMSparkMax frontLeft = new PWMSparkMax(0);
				PWMSparkMax rearLeft = new PWMSparkMax(1);
				PWMSparkMax frontRight = new PWMSparkMax(2);
				PWMSparkMax rearRight = new PWMSparkMax(3);
				tankDrive = new DifferentialDrive(power -> {
					frontLeft.set(-power * 1.5);
					rearLeft.set(-power * 1.5);
				}, power -> {
					frontRight.set(power * 1.5);
					rearRight.set(power * 1.5);
				});
			}
			case 3 -> {
				VictorSP frontLeft = new VictorSP(0);
				VictorSP rearLeft = new VictorSP(2);
				VictorSP frontRight = new VictorSP(1);
				VictorSP rearRight = new VictorSP(3);
				tankDrive = new DifferentialDrive(power -> {
					frontLeft.set(power);
					rearLeft.set(power);
				}, power -> {
					frontRight.set(-power);
					rearRight.set(-power);
				});
			}
		}
	}

	public void periodic() {
		BusChain.refreshAll();
		Logger.recordOutput("EndEfector/position", endEffector.getEncoder().getPosition());

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

	public SparkMaxWrapper getEndEffector() {
		return endEffector;
	}
	
	public GBSubsystem getSubsystem() {
		return subsystem;
	}

	public DifferentialDrive getTankDrive() {
		return tankDrive;
	}

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.utils.battery.BatteryUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();
	public static final int TEAM_NUMBER = 1;

	private final SparkMaxWrapper intake;
	private MecanumDrive mecanumDrive;
	private DifferentialDrive tankDrive;

	public Robot() {
		BatteryUtil.scheduleLimiter();
		intake = new SparkMaxWrapper(new SparkMaxDeviceID(0));
		switch (TEAM_NUMBER) {
			case 0 -> {
				TalonSRX frontLeft = new TalonSRX(0);
				TalonSRX rearLeft = new TalonSRX(0);
				TalonSRX frontRight = new TalonSRX(0);
				TalonSRX rearRight = new TalonSRX(0);
				mecanumDrive = new MecanumDrive(
					power -> frontLeft.set(ControlMode.PercentOutput, power),
					power -> rearLeft.set(ControlMode.PercentOutput, power),
					power -> frontRight.set(ControlMode.PercentOutput, power),
					power -> rearRight.set(ControlMode.PercentOutput, power)
				);
			}
			case 1 -> {
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
				PWMSparkMax rearLeft = new PWMSparkMax(0);
				PWMSparkMax frontRight = new PWMSparkMax(0);
				PWMSparkMax rearRight = new PWMSparkMax(0);
				tankDrive = new DifferentialDrive(power -> {
					frontLeft.set(power);
					rearLeft.set(power);
				}, power -> {
					frontRight.set(power);
					rearRight.set(power);
				});
			}
		}
	}

	public void periodic() {
		BusChain.refreshAll();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

	public SparkMaxWrapper getIntake() {
		return intake;
	}

	public MecanumDrive getMecanumDrive() {
		return mecanumDrive;
	}

	public DifferentialDrive getTankDrive() {
		return tankDrive;
	}

}

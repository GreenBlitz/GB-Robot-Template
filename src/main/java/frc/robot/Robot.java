// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);
	public final DifferentialDrive tank;

	public Robot() {
		BatteryUtil.scheduleLimiter();
		TalonSRX left1 = new TalonSRX(3);
		TalonSRX left2 = new TalonSRX(4);
		TalonSRX right1 = new TalonSRX(1);
		TalonSRX right2 = new TalonSRX(2);

		right1.setInverted(true);
		right2.setInverted(true);

		this.tank = new DifferentialDrive(
				(double power) -> {
					left1.set(TalonSRXControlMode.PercentOutput, power);
					left2.set(TalonSRXControlMode.PercentOutput, power);
				},
				(double power) -> {
					right1.set(TalonSRXControlMode.PercentOutput, power);
					right2.set(TalonSRXControlMode.PercentOutput, power);
				}
		);
	}

	public void periodic() {
		BusChain.refreshAll();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public DifferentialDrive getTank() {
		return tank;
	}

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

}

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.rev.motors.*;
import frc.utils.battery.BatteryUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);
	public static final int TANK_TYPE = 2;

	private DifferentialDrive tankDrive;
	private double MAX_POWER_LIMIT = 1;

	public BrushlessSparkMAXMotor endEffector;

	public Robot() {
		// hardware presentation
		// motors and controllers presentation
		// elec with them
		// black box codeing

		BatteryUtil.scheduleLimiter();
		switch (TANK_TYPE) {
			case 0 -> {
				TalonSRX frontLeft = new TalonSRX(3);
				TalonSRX rearLeft = new TalonSRX(4);
				TalonSRX frontRight = new TalonSRX(1);
				TalonSRX rearRight = new TalonSRX(2);
				tankDrive = new DifferentialDrive(power -> {
					frontLeft.set(ControlMode.PercentOutput, power * MAX_POWER_LIMIT);
					rearLeft.set(ControlMode.PercentOutput, power * MAX_POWER_LIMIT);
				}, power -> {
					frontRight.set(ControlMode.PercentOutput, -power * MAX_POWER_LIMIT);
					rearRight.set(ControlMode.PercentOutput, -power * MAX_POWER_LIMIT);
				});
			}
			case 1 -> {
				PWMSparkMax frontLeft = new PWMSparkMax(0);
				PWMSparkMax rearLeft = new PWMSparkMax(1);
				PWMSparkMax frontRight = new PWMSparkMax(2);
				PWMSparkMax rearRight = new PWMSparkMax(3);
				tankDrive = new DifferentialDrive(power -> {
					frontLeft.set(-power * MAX_POWER_LIMIT);
					rearLeft.set(-power * MAX_POWER_LIMIT);
				}, power -> {
					frontRight.set(power * MAX_POWER_LIMIT);
					rearRight.set(power * MAX_POWER_LIMIT);
				});
			}
			case 2 -> {
				VictorSP left = new VictorSP(4);
				VictorSP right = new VictorSP(0);
				tankDrive = new DifferentialDrive(power -> {
					left.set(power * MAX_POWER_LIMIT);
				}, power -> {
					right.set(-power * MAX_POWER_LIMIT);
				});
			}
		}

		endEffector = buildEndEffector();
	}

	private BrushlessSparkMAXMotor buildEndEffector() {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(new SparkMaxDeviceID(5));

		SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

		sparkMaxConfig.softLimit.forwardSoftLimit(1);
		sparkMaxConfig.softLimit.reverseSoftLimit(-1);
		sparkMaxConfig.softLimit.forwardSoftLimitEnabled(true);
		sparkMaxConfig.softLimit.reverseSoftLimitEnabled(true);

		SparkMaxConfiguration configuration = new SparkMaxConfiguration().withSparkMaxConfig(sparkMaxConfig);

		sparkMaxWrapper.applyConfiguration(configuration);

		return new BrushlessSparkMAXMotor(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/endEffector", sparkMaxWrapper, new SysIdRoutine.Config());
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

	public DifferentialDrive getTankDrive() {
		return tankDrive;
	}

}

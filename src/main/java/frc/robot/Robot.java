// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.subsystems.arm.ArmBuilder;
import frc.robot.subsystems.arm.DynamicMotionMagicArm;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;
import frc.utils.calibration.sysid.SysIdCalibrator;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();
	private static final Slot0Configs config = new Slot0Configs();
	private final DynamicMotionMagicArm arm;

	public Robot() {
		BatteryUtil.scheduleLimiter();
		config.kP = 70;
		config.kI = 0;
		config.kD = 0;
		config.kS = 0;
		config.kG = 0;

//        config.MotionMagic.withMotionMagicCruiseVelocity(3);
//        config.MotionMagic.withMotionMagicAcceleration(3);
		Phoenix6DeviceID id = new Phoenix6DeviceID(20);
		FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
		feedbackConfigs.RotorToSensorRatio = 450.0 / 7.0;
		feedbackConfigs.SensorToMechanismRatio = 1;

		arm = ArmBuilder.createDynamicMotionMagic(
                "Arm/",
                new TalonFXFollowerConfig(),
                id,
                new SysIdCalibrator.SysIdConfigInfo(new SysIdRoutine.Config(), true),feedbackConfigs,config,config,40,50,0.001,0.3,Rotation2d.fromDegrees(-24 + -(16)).getRadians(),Rotation2d.fromDegrees(246 +  Rotation2d.fromDegrees(-16).getDegrees()).getRadians(),InvertedValue.Clockwise_Positive,0,Rotation2d.fromRotations(3), Rotation2d.fromRotations(3));

	}


	public void periodic() {
		BusChain.refreshAll();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public DynamicMotionMagicArm getArm() {
		return arm;
	}

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

}

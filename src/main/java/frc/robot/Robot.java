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
import frc.robot.subsystems.arm.Arm;
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
	private static final Slot0Configs configPivot = new Slot0Configs();
	private static final Slot0Configs configArm = new Slot0Configs();
	private final DynamicMotionMagicArm arm;
	private final Arm pivot;

	public Robot() {
		BatteryUtil.scheduleLimiter();
		configArm.kP = 70;
		configArm.kI = 0;
		configArm.kD = 0;
		configArm.kS = 0;
		configArm.kG = 0;

        configPivot.kP = 5;
        configPivot.kI = 0;
        configPivot.kD = 0;
        configPivot.kG = 0;
        configPivot.kS = 0;
        configPivot.kV = 0;
        configPivot.kA = 0;
//        config.MotionMagic.withMotionMagicCruiseVelocity(3);
//        config.MotionMagic.withMotionMagicAcceleration(3);
		Phoenix6DeviceID id = new Phoenix6DeviceID(20);
		FeedbackConfigs feedbackConfigsArm = new FeedbackConfigs();
		feedbackConfigsArm.RotorToSensorRatio = 450.0 / 7.0;
		feedbackConfigsArm.SensorToMechanismRatio = 1;
        FeedbackConfigs feedbackConfigsPivot = new FeedbackConfigs();
		feedbackConfigsPivot.RotorToSensorRatio = 21.43;
		feedbackConfigsPivot.SensorToMechanismRatio = 1;



		arm = ArmBuilder.createDynamicMotionMagic(
			"Arm/",
			new TalonFXFollowerConfig(),
			id,
			new SysIdCalibrator.SysIdConfigInfo(new SysIdRoutine.Config(), true),
			feedbackConfigsArm,
                configArm,
                configArm,
			40,
			50,
			0.001,
			0.3,
			Rotation2d.fromDegrees(-24 + -(16)).getRadians(),
			Rotation2d.fromDegrees(246 + Rotation2d.fromDegrees(-16).getDegrees()).getRadians(),
			InvertedValue.Clockwise_Positive,
			0,
			Rotation2d.fromRotations(3),
			Rotation2d.fromRotations(3)
		);
        pivot = ArmBuilder.create(
			"Pivot/",
                new TalonFXFollowerConfig(),
                new Phoenix6DeviceID(5),
                new SysIdCalibrator.SysIdConfigInfo(new SysIdRoutine.Config(), true),
                feedbackConfigsPivot,
                configPivot,
                configPivot,
                40,
                50,
                0.001,
                0.359,
                Rotation2d.fromDegrees(-33).getRadians(),
                Rotation2d.fromDegrees(130).getRadians(),
                InvertedValue.Clockwise_Positive,
                0);

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

    public Arm getPivot(){
        return pivot;
    }
	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

}

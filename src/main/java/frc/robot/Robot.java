// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
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

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false   );
	private static final Slot0Configs configPivot = new Slot0Configs();
	private static final Slot0Configs configRealPivot = new Slot0Configs();
	private static final Slot0Configs configArm = new Slot0Configs();
	private static final Slot0Configs configRealArm = new Slot0Configs();
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

        configRealArm.kP = 28;
        configRealArm.kI = 0;
        configRealArm.kD = 0;
        configRealArm.kS = 0.065;
        configRealArm.kG = 0.37;
        configRealArm.kV = 9.0000095367432;
        configRealArm.kA = 0.5209;
		configRealArm.GravityType = GravityTypeValue.Arm_Cosine;

        configRealPivot.kP = 20;
        configRealPivot.kI = 0;
        configRealPivot.kD = 0;
        configRealPivot.kG = 0.4;
        configRealPivot.kS = 0.05;
        configRealPivot.kV = 0;
        configRealPivot.kA = 0;
		configRealPivot.GravityType = GravityTypeValue.Arm_Cosine;

//        config.MotionMagic.withMotionMagicCruiseVelocity(3);
//        config.MotionMagic.withMotionMagicAcceleration(3);
		Phoenix6DeviceID armId = new Phoenix6DeviceID(20, BusChain.Super);
		FeedbackConfigs feedbackConfigsArm = new FeedbackConfigs();
		feedbackConfigsArm.RotorToSensorRatio = 450.0 / 7.0;
		feedbackConfigsArm.SensorToMechanismRatio = 1;
		feedbackConfigsArm.FeedbackRemoteSensorID = 20;
		feedbackConfigsArm.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        FeedbackConfigs feedbackConfigsPivot = new FeedbackConfigs();
		feedbackConfigsPivot.RotorToSensorRatio = 1;
		feedbackConfigsPivot.SensorToMechanismRatio = 21.43;

		arm = ArmBuilder.buildDynamicMotionMagicArm(
			"Subsystem/Arm",
			armId,
            true,
            new TalonFXFollowerConfig(),
            new SysIdRoutine.Config(),
			feedbackConfigsArm,
            configRealArm,
            configArm,
            40,
            (int)RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			0.001,
			0.3,
			0,
            Rotation2d.fromDegrees(231 -16),
            Rotation2d.fromDegrees(-24 -16),
			Rotation2d.fromRotations(3),
			Rotation2d.fromRotations(3)
		);

        pivot = ArmBuilder.buildArm(
			"Subsystem/Pivot",
                new Phoenix6DeviceID(15,BusChain.ROBORIO),
                true,
                new TalonFXFollowerConfig(),
                new SysIdRoutine.Config(),
                feedbackConfigsPivot,
                configRealPivot,
                configPivot,
                40,
                50,
                0.001,
                0.359,
                0,
                Rotation2d.fromDegrees(120),
                Rotation2d.fromDegrees(-10)
                );
		pivot.setPosition(Rotation2d.fromDegrees(130));
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

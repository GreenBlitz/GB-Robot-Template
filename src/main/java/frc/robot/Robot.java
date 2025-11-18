// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.TalonFXArmBuilder;
import frc.robot.subsystems.constants.hood.HoodConstants;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);
	private final Arm hood;

	public Robot() {
		BatteryUtil.scheduleLimiter();
		hood = createHood();
	}

	public void periodic() {
		BusChain.refreshAll();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

	public Arm getHood() {
		return hood;
	}

	public Arm createHood() {
		FeedbackConfigs hoodFeedbackConfig = new FeedbackConfigs();
		Slot0Configs realSlotConfig = new Slot0Configs();
		Slot0Configs simulationSlotConfig = new Slot0Configs();

		realSlotConfig.kP = HoodConstants.kP;
		realSlotConfig.kI = HoodConstants.kI;
		realSlotConfig.kD = HoodConstants.kD;
		realSlotConfig.kV = HoodConstants.kV;
		realSlotConfig.kG = HoodConstants.kG;
		realSlotConfig.kA = HoodConstants.kA;
		realSlotConfig.kS = HoodConstants.kS;

		simulationSlotConfig.kP = HoodConstants.SIM_kP;
		simulationSlotConfig.kI = HoodConstants.SIM_kI;
		simulationSlotConfig.kD = HoodConstants.SIM_kD;
		simulationSlotConfig.kG = HoodConstants.SIM_kG;
		simulationSlotConfig.kS = HoodConstants.SIM_kS;

		hoodFeedbackConfig.SensorToMechanismRatio = HoodConstants.GEAR_RATIO;
		hoodFeedbackConfig.RotorToSensorRatio = 1;

		return TalonFXArmBuilder.buildMotionMagicArm(
			RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Hood",
			IDs.TalonFXIDs.hoodId,
			HoodConstants.IS_INVERTED,
			new TalonFXFollowerConfig(),
			new SysIdRoutine.Config(), // q
			hoodFeedbackConfig,
			realSlotConfig,
			simulationSlotConfig,
			HoodConstants.CURRENT_LIMIT,
			(int) RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			HoodConstants.MOMENT_OF_INERTIA,
			HoodConstants.ARM_LENGTH_METERS,
			HoodConstants.ARBITRARY_FEEDFORWARD,
			HoodConstants.FORWARD_SOFTWARE_LIMIT,
			HoodConstants.BACKWARD_SOFTWARE_LIMIT,
			HoodConstants.DEFAULT_MAX_ACCELERATION_PER_SECOND_SQUARE,
			HoodConstants.DEFAULT_MAX_VELOCITY_PER_SECOND
		);
	}

}

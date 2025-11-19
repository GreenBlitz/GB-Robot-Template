// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.TalonFXArmBuilder;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	private final Arm turret;
	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);

	public Robot() {
		BatteryUtil.scheduleLimiter();
		this.turret = createTurret();
	}

	public void periodic() {
		BusChain.refreshAll();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	private Arm createTurret() {
		return TalonFXArmBuilder.buildMotionMagicArm(
				TurretConstants.LOG_PATH,
				TurretConstants.DEVICE_ID,
				TurretConstants.IS_INVERTED,
				TurretConstants.TALON_FX_FOLLOWER_CONFIG,
				TurretConstants.SYS_ID_ROUTINE_CONFIG,
				TurretConstants.FEEDBACK_CONFIGS,
				TurretConstants.REAL_SLOTS_CONFIG(),
				TurretConstants.SIMULATION_SLOTS_CONFIG(),
				TurretConstants.CURRENT_LIMIT,
				TurretConstants.SIGNALS_FREQUENCY,
				TurretConstants.MOMENT_OF_INERTIA,
				TurretConstants.ARM_LENGTH,
				TurretConstants.ARBITRARY_FEED_FORWARD,
				TurretConstants.FORWARD_SOFTWARE_LIMIT,
				TurretConstants.BACKWARDS_SOFTWARE_LIMIT,
				TurretConstants.DEFAULT_MAX_ACCELERATION_PER_SECOND_SQUARE,
				TurretConstants.DEFAULT_MAX_VELOCITY_PER_SECOND
		);
	}

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.subsystems.constants.intakeRollers.IntakeRollerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.TalonFXArmBuilder;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.TalonFXArmBuilder;
import frc.robot.subsystems.constants.hood.HoodConstants;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.flywheel.KrakenX60FlyWheelBuilder;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.SparkMaxRollerBuilder;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;
import frc.utils.brakestate.BrakeStateManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);
	private final Arm turret;
	private final FlyWheel flyWheel;
	private final Roller intakeRoller;
	private final Arm hood;

	public Robot() {
		BatteryUtil.scheduleLimiter();

		this.turret = createTurret();
		turret.setPosition(TurretConstants.MIN_POSITION);
		BrakeStateManager.add(() -> turret.setBrake(true), () -> turret.setBrake(false));

		this.flyWheel = KrakenX60FlyWheelBuilder.build("Subsystems/FlyWheel", IDs.TalonFXIDs.FLYWHEEL);

		this.intakeRoller = createIntakeRollers();

		this.hood = createHood();
		hood.setPosition(HoodConstants.MINIMUM_POSITION);
		BrakeStateManager.add(() -> hood.setBrake(true), () -> hood.setBrake(false));
	}

	public void resetSubsystems() {
		if (HoodConstants.MINIMUM_POSITION.getRadians() > hood.getPosition().getRadians()) {
			hood.setPosition(HoodConstants.MINIMUM_POSITION);
		}
		if (TurretConstants.MIN_POSITION.getRadians() > turret.getPosition().getRadians()) {
			turret.setPosition(TurretConstants.MIN_POSITION);
		}
	}

	public void periodic() {
		BusChain.refreshAll();
		resetSubsystems();
		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public Roller createIntakeRollers() {
		return SparkMaxRollerBuilder.build(
			RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/IntakeRollers",
			IDs.SparkMAXIDs.INTAKE_ROLLERS,
			IntakeRollerConstants.IS_INVERTED,
			IntakeRollerConstants.GEAR_RATIO,
			IntakeRollerConstants.CURRENT_LIMIT,
			IntakeRollerConstants.MOMENT_OF_INERTIA
		);
	}

	private Arm createTurret() {
		return TalonFXArmBuilder.buildMotionMagicArm(
			TurretConstants.LOG_PATH,
			IDs.TalonFXIDs.TURRET,
			TurretConstants.IS_INVERTED,
			TurretConstants.TALON_FX_FOLLOWER_CONFIG,
			TurretConstants.SYS_ID_ROUTINE_CONFIG,
			TurretConstants.FEEDBACK_CONFIGS,
			TurretConstants.REAL_SLOTS_CONFIG,
			TurretConstants.SIMULATION_SLOTS_CONFIG,
			TurretConstants.CURRENT_LIMIT,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			TurretConstants.MOMENT_OF_INERTIA,
			TurretConstants.ARM_LENGTH,
			TurretConstants.ARBITRARY_FEED_FORWARD,
			TurretConstants.FORWARD_SOFTWARE_LIMIT,
			TurretConstants.BACKWARDS_SOFTWARE_LIMIT,
			TurretConstants.DEFAULT_MAX_ACCELERATION_PER_SECOND_SQUARE,
			TurretConstants.DEFAULT_MAX_VELOCITY_PER_SECOND
		);
	}

	public Arm getTurret() {
		return turret;
	}

	public FlyWheel getFlyWheel() {
		return flyWheel;
	}

	public Roller getIntakeRoller() {
		return intakeRoller;
	}

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

	public Arm getHood() {
		return hood;
	}

	private Arm createHood() {
		return TalonFXArmBuilder.buildMotionMagicArm(
			RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Hood",
			IDs.TalonFXIDs.HOOD,
			HoodConstants.IS_INVERTED,
			new TalonFXFollowerConfig(),
			HoodConstants.SYSIDROUTINE_CONFIG,
			HoodConstants.FEEDBACK_CONFIGS,
			HoodConstants.REAL_SLOT,
			HoodConstants.SIMULATION_SLOT,
			HoodConstants.CURRENT_LIMIT,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			HoodConstants.MOMENT_OF_INERTIA,
			HoodConstants.HOOD_LENGTH_METERS,
			HoodConstants.ARBITRARY_FEEDFORWARD,
			HoodConstants.FORWARD_SOFTWARE_LIMIT,
			HoodConstants.BACKWARD_SOFTWARE_LIMIT,
			HoodConstants.DEFAULT_MAX_ACCELERATION_PER_SECOND_SQUARE,
			HoodConstants.DEFAULT_MAX_VELOCITY_PER_SECOND
		);
	}

}

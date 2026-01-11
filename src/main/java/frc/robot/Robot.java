// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.statemachine.RobotCommander;
import frc.robot.statemachine.ScoringHelpers;
import frc.robot.statemachine.shooterstatehandler.ShooterStateHandler;
import frc.robot.subsystems.arm.ArmSimulationConstants;
import frc.robot.subsystems.constants.intakeRollers.IntakeRollerConstants;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.TalonFXArmBuilder;
import frc.robot.subsystems.constants.belly.BellyConstants;
import frc.robot.subsystems.constants.fourBar.FourBarConstants;
import frc.robot.subsystems.constants.hood.HoodConstants;
import frc.robot.subsystems.constants.omni.OmniConstant;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.flywheel.KrakenX60FlyWheelBuilder;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.SparkMaxRollerBuilder;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.imu.IMUFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
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
	private final Arm fourBar;
	private final Arm hood;
	private final IDigitalInput intakeRollerSensor;
	private final Roller belly;
	private final Roller omni;
	private final IDigitalInput funnelDigitalInput;
	private final SimulationManager simulationManager;

	private final RobotCommander robotCommander;

	private final Swerve swerve;
	private final IPoseEstimator poseEstimator;

	public Robot() {
		BatteryUtil.scheduleLimiter();

		this.turret = createTurret();
		turret.setPosition(TurretConstants.MIN_POSITION);
		BrakeStateManager.add(() -> turret.setBrake(true), () -> turret.setBrake(false));

		this.flyWheel = KrakenX60FlyWheelBuilder.build("Subsystems/FlyWheel", IDs.TalonFXIDs.FLYWHEEL);

		this.fourBar = createFourBar();
		fourBar.setPosition(FourBarConstants.MAXIMUM_POSITION);
		BrakeStateManager.add(() -> fourBar.setBrake(true), () -> fourBar.setBrake(false));

		this.hood = createHood();
		hood.setPosition(HoodConstants.MINIMUM_POSITION);
		BrakeStateManager.add(() -> hood.setBrake(true), () -> hood.setBrake(false));

		Pair<Roller, IDigitalInput> intakeRollerAndDigitalInput = createIntakeRollers();
		this.intakeRoller = intakeRollerAndDigitalInput.getFirst();
		this.intakeRollerSensor = intakeRollerAndDigitalInput.getSecond();
		BrakeStateManager.add(() -> intakeRoller.setBrake(true), () -> intakeRoller.setBrake(false));

		this.belly = createBelly();
		BrakeStateManager.add(() -> belly.setBrake(true), () -> belly.setBrake(false));

		Pair<Roller, IDigitalInput> omniAndDigitalInput = createOmniAndSignal();
		this.omni = omniAndDigitalInput.getFirst();
		this.funnelDigitalInput = omniAndDigitalInput.getSecond();
		BrakeStateManager.add(() -> omni.setBrake(true), () -> omni.setBrake(false));

		IIMU imu = IMUFactory.createIMU(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			imu,
			IMUFactory.createSignals(imu)
		);

		this.poseEstimator = new WPILibPoseEstimatorWrapper(
			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH,
			swerve.getKinematics(),
			swerve.getModules().getWheelPositions(0),
			swerve.getGyroAbsoluteYaw().getValue(),
			swerve.getGyroAbsoluteYaw().getTimestamp(),
			swerve.getIMUAcceleration()
		);

		robotCommander = new RobotCommander("/RobotCommander", this);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.getStateHandler().setIsTurretMoveLegalSupplier(() -> isTurretMoveLegal());
		swerve.getStateHandler().setRobotPoseSupplier(() -> poseEstimator.getEstimatedPose());
		swerve.getStateHandler().setTurretAngleSupplier(() -> turret.getPosition());

		simulationManager = new SimulationManager("SimulationManager", this);
	}

	public void resetSubsystems() {
		if (HoodConstants.MINIMUM_POSITION.getRadians() > hood.getPosition().getRadians()) {
			hood.setPosition(HoodConstants.MINIMUM_POSITION);
		}
		if (TurretConstants.MIN_POSITION.getRadians() > turret.getPosition().getRadians()) {
			turret.setPosition(TurretConstants.MIN_POSITION);
		}
		if (FourBarConstants.MAXIMUM_POSITION.getRadians() < fourBar.getPosition().getRadians()) {
			fourBar.setPosition(FourBarConstants.MAXIMUM_POSITION);
		}
	}

	public boolean isTurretMoveLegal() {
		return ShooterStateHandler.isTurretMoveLegal(
			ShooterStateHandler.getRobotRelativeLookAtTowerAngleForTurret(
				ScoringHelpers.getClosestTower(poseEstimator.getEstimatedPose()).getPose().getTranslation(),
				poseEstimator.getEstimatedPose()
			),
			turret
		);
	}

	public void periodic() {
		BusChain.refreshAll();
		resetSubsystems();
		simulationManager.logPoses();

		swerve.update();
		poseEstimator.updateOdometry(swerve.getAllOdometryData());
		poseEstimator.log();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	private Pair<Roller, IDigitalInput> createIntakeRollers() {
		return SparkMaxRollerBuilder.buildWithDigitalInput(
			RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/IntakeRollers",
			IDs.SparkMAXIDs.INTAKE_ROLLERS,
			IntakeRollerConstants.IS_INVERTED,
			IntakeRollerConstants.GEAR_RATIO,
			IntakeRollerConstants.CURRENT_LIMIT,
			IntakeRollerConstants.MOMENT_OF_INERTIA,
			IntakeRollerConstants.DIGITAL_INPUT_NAME,
			IntakeRollerConstants.DEBOUNCE_TIME,
			IntakeRollerConstants.IS_FORWARD_LIMIT_SWITCH,
			IntakeRollerConstants.IS_SENSOR_INVERTED
		);
	}

	private Arm createTurret() {
		ArmSimulationConstants turretSimulationConstants = new ArmSimulationConstants(
			TurretConstants.MAX_POSITION,
			TurretConstants.MIN_POSITION,
			TurretConstants.MIN_POSITION,
			TurretConstants.MOMENT_OF_INERTIA,
			TurretConstants.TURRET_RADIUS
		);
		return TalonFXArmBuilder.buildMotionMagicArm(
			TurretConstants.LOG_PATH,
			IDs.TalonFXIDs.TURRET,
			TurretConstants.IS_INVERTED,
			TurretConstants.IS_CONTINUOUS_WRAP,
			TurretConstants.TALON_FX_FOLLOWER_CONFIG,
			TurretConstants.SYS_ID_ROUTINE_CONFIG,
			TurretConstants.FEEDBACK_CONFIGS,
			TurretConstants.REAL_SLOTS_CONFIG,
			TurretConstants.SIMULATION_SLOTS_CONFIG,
			TurretConstants.CURRENT_LIMIT,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			TurretConstants.ARBITRARY_FEED_FORWARD,
			TurretConstants.FORWARD_SOFTWARE_LIMIT,
			TurretConstants.BACKWARDS_SOFTWARE_LIMIT,
			turretSimulationConstants,
			TurretConstants.DEFAULT_MAX_ACCELERATION_PER_SECOND_SQUARE,
			TurretConstants.DEFAULT_MAX_VELOCITY_PER_SECOND
		);
	}

	private Arm createFourBar() {
		ArmSimulationConstants fourBarSimConstant = new ArmSimulationConstants(
			FourBarConstants.MAXIMUM_POSITION,
			FourBarConstants.MINIMUM_POSITION,
			FourBarConstants.MAXIMUM_POSITION,
			FourBarConstants.MOMENT_OF_INERTIA,
			FourBarConstants.FOUR_BAR_LENGTH
		);
		return TalonFXArmBuilder.buildDynamicMotionMagicArm(
			FourBarConstants.LOG_PATH,
			IDs.TalonFXIDs.FOUR_BAR,
			FourBarConstants.IS_INVERTED,
			FourBarConstants.IS_CONTINUOUS_WRAP,
			FourBarConstants.TALON_FX_FOLLOWER_CONFIG,
			FourBarConstants.SYS_ID_ROUTINE,
			FourBarConstants.FEEDBACK_CONFIGS,
			FourBarConstants.REAL_SLOT,
			FourBarConstants.SIMULATION_SLOT,
			FourBarConstants.CURRENT_LIMIT,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			FourBarConstants.ARBITRARY_FEED_FORWARD,
			FourBarConstants.FORWARD_SOFTWARE_LIMITS,
			FourBarConstants.BACKWARD_SOFTWARE_LIMITS,
			fourBarSimConstant,
			FourBarConstants.MAX_ACCELERATION_ROTATION2D_PER_SECONDS_SQUARE,
			FourBarConstants.MAX_VELOCITY_ROTATION2D_PER_SECONDS
		);
	}

	private Roller createBelly() {
		return SparkMaxRollerBuilder.build(
			BellyConstants.LOG_PATH,
			IDs.SparkMAXIDs.BELLY,
			BellyConstants.INVERTED,
			BellyConstants.GEAR_RATIO,
			BellyConstants.CURRENT_LIMIT,
			BellyConstants.MOMENT_OF_INERTIA
		);
	}

	private Arm createHood() {
		ArmSimulationConstants hoodSimulationConstants = new ArmSimulationConstants(
			HoodConstants.MAXIMUM_POSITION,
			HoodConstants.MINIMUM_POSITION,
			HoodConstants.MINIMUM_POSITION,
			HoodConstants.MOMENT_OF_INERTIA,
			HoodConstants.HOOD_LENGTH_METERS
		);
		return TalonFXArmBuilder.buildDynamicMotionMagicArm(
			RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Hood",
			IDs.TalonFXIDs.HOOD,
			HoodConstants.IS_INVERTED,
			HoodConstants.IS_CONTINUOUS_WRAP,
			new TalonFXFollowerConfig(),
			HoodConstants.SYSIDROUTINE_CONFIG,
			HoodConstants.FEEDBACK_CONFIGS,
			HoodConstants.REAL_SLOT,
			HoodConstants.SIMULATION_SLOT,
			HoodConstants.CURRENT_LIMIT,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			HoodConstants.ARBITRARY_FEEDFORWARD,
			HoodConstants.FORWARD_SOFTWARE_LIMIT,
			HoodConstants.BACKWARD_SOFTWARE_LIMIT,
			hoodSimulationConstants,
			HoodConstants.DEFAULT_MAX_ACCELERATION_PER_SECOND_SQUARE,
			HoodConstants.DEFAULT_MAX_VELOCITY_PER_SECOND
		);
	}

	private Pair<Roller, IDigitalInput> createOmniAndSignal() {
		return SparkMaxRollerBuilder.buildWithDigitalInput(
			OmniConstant.LOG_PATH,
			IDs.SparkMAXIDs.OMNI,
			OmniConstant.IS_INVERTED,
			OmniConstant.GEAR_RATIO,
			OmniConstant.CURRENT_LIMIT,
			OmniConstant.MOMENT_OF_INERTIA,
			OmniConstant.FUNNEL_INPUT_NAME,
			OmniConstant.DEBOUNCE_TIME,
			OmniConstant.IS_FORWARD_LIMIT_SWITCH,
			OmniConstant.IS_FORWARD_LIMIT_SWITCH_INVERTED
		);
	}

	public IDigitalInput getIntakeRollerSensor() {
		return intakeRollerSensor;
	}

	public Roller getIntakeRoller() {
		return intakeRoller;
	}

	public Roller getBelly() {
		return belly;
	}

	public Arm getTurret() {
		return turret;
	}

	public FlyWheel getFlyWheel() {
		return flyWheel;
	}

	public Arm getFourBar() {
		return fourBar;
	}

	public Roller getOmni() {
		return omni;
	}

	public IDigitalInput getFunnelDigitalInput() {
		return funnelDigitalInput;
	}

	public Arm getHood() {
		return hood;
	}

	public IPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public RobotCommander getRobotCommander() {
		return robotCommander;
	}

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

}

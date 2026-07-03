// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.RobotManager;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.autonomous.AutosBuilder;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.statemachine.RobotCommander;
import frc.robot.statemachine.RobotState;
import frc.robot.statemachine.ShootingCalculations;
import frc.robot.statemachine.intakestatehandler.IntakeState;
import frc.robot.subsystems.arm.CurrentControlArm;
import frc.robot.subsystems.arm.VelocityPositionArm;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.constants.conveyor.ConveyorConstants;
import frc.robot.subsystems.constants.flywheel.FlywheelConstants;
import frc.robot.subsystems.constants.pivot.PivotConstants;
import frc.robot.subsystems.constants.hood.HoodConstants;
import frc.robot.subsystems.constants.intakeRollers.IntakeRollerConstants;
import frc.robot.subsystems.constants.magazine.MagazineConstant;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.robot.subsystems.constants.upperRoller.UpperRollerConstants;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.VelocityRoller;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.imu.IMUFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.statemachine.shooterstatehandler.TurretCalculations;
import frc.utils.GamePeriodUtils;
import frc.utils.auto.AutonomousChooser;
import frc.robot.subsystems.swerve.factories.modules.drive.KrakenX60DriveBuilder;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.vision.cameras.limelight.Limelight;
import frc.robot.vision.cameras.limelight.LimelightFilters;
import frc.robot.vision.cameras.limelight.LimelightPipeline;
import frc.robot.vision.cameras.limelight.LimelightStdDevCalculations;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;
import frc.utils.brakestate.BrakeMode;
import frc.utils.brakestate.BrakeStateManager;
import frc.utils.math.StandardDeviations2D;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);

	private final FlyWheel flyWheel;
	private final VelocityPositionArm turret;
	private final Arm hood;

	private final CurrentControlArm pivot;
	private final Roller intakeRoller;

	private final VelocityRoller magazine;
	private final Roller conveyor;
	private final Roller upperRoller;

	private final IDigitalInput magazineBallSensor;

	private final Swerve swerve;

	private final SimulationManager simulationManager;

	private final RobotCommander robotCommander;

	private AutonomousChooser autonomousChooser;
	private SendableChooser<Boolean> returnToMiddle;

	private final IPoseEstimator poseEstimator;

	private final Limelight limelightFront;
	private final Limelight limelightRight;
	private final Limelight limelightLeft;
	private final List<Limelight> limelights;

	private static double ballCounterIncludingPassing;
	private static double ballCounterWithoutPassing;

	private final TimeInterpolatableBuffer<Double> ballsBufferIncludingPassing;
	private final TimeInterpolatableBuffer<Double> ballsBufferWithoutPassing;
	private final Supplier<Double> lastBallThrownTimestamp;

	public Robot() {
		BatteryUtil.scheduleLimiter();

		this.flyWheel = FlywheelConstants.createFlyWheel();

		ballsBufferIncludingPassing = TimeInterpolatableBuffer
			.createBuffer(Interpolator.forDouble(), RobotConstants.MAX_TIME_FOR_BPS_INTERPOLATOR);
		ballsBufferWithoutPassing = TimeInterpolatableBuffer
			.createBuffer(Interpolator.forDouble(), RobotConstants.MAX_TIME_FOR_BPS_INTERPOLATOR);
		this.lastBallThrownTimestamp = () -> ballsBufferIncludingPassing.getInternalBuffer().floorKey(TimeUtil.getCurrentTimeSeconds()) == null
			? TimeUtil.getCurrentTimeSeconds()
			: ballsBufferIncludingPassing.getInternalBuffer().floorKey(TimeUtil.getCurrentTimeSeconds());

		this.turret = TurretConstants.createTurret();
		turret.setPosition(TurretConstants.MAX_POSITION);
		BrakeStateManager.add(() -> turret.setBrake(true), () -> turret.setBrake(false));

		this.hood = HoodConstants.createHood();
		hood.setPosition(HoodConstants.MINIMUM_POSITION);
		BrakeStateManager.add(() -> hood.setBrake(true), () -> hood.setBrake(false));

		this.pivot = PivotConstants.createPivot();
		pivot.setPosition(PivotConstants.MAXIMUM_POSITION);
		BrakeStateManager.add(() -> pivot.setBrake(true), () -> pivot.setBrake(false));

		this.intakeRoller = IntakeRollerConstants.createIntakeRollers();
		BrakeStateManager.add(() -> intakeRoller.setBrake(true), () -> intakeRoller.setBrake(false));

		this.magazine = MagazineConstant.createMagazine();
		BrakeStateManager.add(() -> magazine.setBrake(true), () -> magazine.setBrake(false));

		this.conveyor = ConveyorConstants.createConveyor();
		BrakeStateManager.add(() -> conveyor.setBrake(true), () -> conveyor.setBrake(false));

		this.upperRoller = UpperRollerConstants.createUpperRoller();
		BrakeStateManager.add(() -> upperRoller.setBrake(true), () -> upperRoller.setBrake(false));

		this.magazineBallSensor = MagazineConstant.createMagazineBallSensor();

		IIMU imu = IMUFactory.createIMU(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			imu,
			IMUFactory.createSignals(imu)
		);
		BrakeStateManager.add(() -> swerve.getModules().setBrake(true), () -> swerve.getModules().setBrake(false));

		this.poseEstimator = new WPILibPoseEstimatorWrapper(
			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH,
			swerve.getKinematics(),
			swerve.getModules().getWheelPositions(0),
			swerve.getModules().getCurrentStates(),
			swerve.getIMUOrientation(),
			swerve.getIMUAccelerationG().toTranslation2d(),
			swerve.getIMUAbsoluteYaw().getTimestamp()
		);

		this.limelightFront = new Limelight(
			"limelight-front",
			"Vision",
			new Pose3d(
				new Translation3d(0.297, -0.143, 0.361),
				new Rotation3d(Math.toRadians(-0.18), Math.toRadians(27.38), Math.toRadians(-0.35))
			),
			LimelightPipeline.APRIL_TAG
		);
		this.limelightRight = new Limelight(
			"limelight-right",
			"Vision",
			new Pose3d(
				new Translation3d(-0.06, 0.367, 0.469),
				new Rotation3d(Math.toRadians(-177.78), Math.toRadians(20.64), Math.toRadians(-90.7))
			),
			LimelightPipeline.APRIL_TAG
		);
		this.limelightLeft = new Limelight(
			"limelight-left",
			"Vision",
			new Pose3d(
				new Translation3d(-0.125, -0.37, 0.481),
				new Rotation3d(Math.toRadians(-179.25), Math.toRadians(20.05), Math.toRadians(90.35))
			),
			LimelightPipeline.APRIL_TAG
		);

		this.limelights = List.of(limelightFront, limelightRight, limelightLeft);
		limelights.forEach(
			limelight -> limelight.setMT1StdDevsCalculation(
				LimelightStdDevCalculations.getMT1StdDevsCalculation(
					limelight,
					new StandardDeviations2D(0.5),
					new StandardDeviations2D(0.15),
					new StandardDeviations2D(0.4),
					new StandardDeviations2D(0.011)
				)
			)
		);
		limelights.forEach(
			limelight -> limelight.setMT1PoseFilter(
				LimelightFilters.megaTag1Filter(
					limelight,
					timestamp -> poseEstimator.getEstimatedPoseAtTimestamp(timestamp).map(Pose2d::getRotation),
					poseEstimator::isIMUOffsetCalibrated,
					new Translation2d(0.1, 0.1),
					Rotation2d.fromDegrees(10)
				)
			)
		);

		robotCommander = new RobotCommander("StateMachine", this);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.getStateHandler().setIsTurretMoveLegalSupplier(() -> isTurretMoveLegal());
		swerve.getStateHandler().setRobotPoseSupplier(() -> poseEstimator.getEstimatedPose());
		swerve.getStateHandler().setTurretAngleSupplier(() -> turret.getPosition());

		simulationManager = new SimulationManager("SimulationManager", this);

		new Trigger(DriverStation::isTeleopEnabled)
			.onTrue(robotCommander.setState(RobotState.RESET_SUBSYSTEMS).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));

		ballCounterIncludingPassing = 0;
		ballCounterWithoutPassing = 0;

		new Trigger(() -> robotCommander.getShooterStateHandler().hasABallBeenShot()).onTrue(new InstantCommand(() -> {
			ballCounterIncludingPassing++;
			ballsBufferIncludingPassing.addSample(TimeUtil.getCurrentTimeSeconds(), ballCounterIncludingPassing);
		}));

		new Trigger(
			() -> robotCommander.getShooterStateHandler().hasABallBeenShot()
				&& (robotCommander.getCurrentState() == RobotState.SCORE || robotCommander.getCurrentState() == RobotState.CALIBRATION_SCORE)
		).onTrue(new InstantCommand(() -> {
			ballCounterWithoutPassing++;
			ballsBufferIncludingPassing.addSample(TimeUtil.getCurrentTimeSeconds(), ballCounterWithoutPassing);
		}));

		new Trigger(GamePeriodUtils::isTransitionShift).onFalse(
			new InstantCommand(
				() -> Logger.recordOutput(
					"averagePeriodPBS/TransitionShift",
					getAverageBPSForLastXSeconds(GamePeriodUtils.TRANSITION_SHIFT_DURATION_SECONDS)
				)
			)
		);
		new Trigger(GamePeriodUtils::isInEndgame).onFalse(
			new InstantCommand(
				() -> Logger.recordOutput("averagePeriodPBS/Endgame", getAverageBPSForLastXSeconds(GamePeriodUtils.ENDGAME_DURATION_SECONDS))
			)
		);
		new Trigger(GamePeriodUtils::isInActive1).onFalse(
			new InstantCommand(
				() -> Logger
					.recordOutput("averagePeriodPBS/Active1", getAverageBPSForLastXSeconds(GamePeriodUtils.ALLIANCE_SHIFT_DURATION_SECONDS))
			)
		);
		new Trigger(GamePeriodUtils::isInActive2).onFalse(
			new InstantCommand(
				() -> Logger
					.recordOutput("averagePeriodPBS/Active2", getAverageBPSForLastXSeconds(GamePeriodUtils.ALLIANCE_SHIFT_DURATION_SECONDS))
			)
		);

		configureBrakeStateChooser();
		configureAuto();
	}

	public RobotConfig getRobotConfig() {
		return new RobotConfig(
			RobotConstants.ROBOT_MASS_KG,
			RobotConstants.ROBOT_MOI,
			new ModuleConfig(
				swerve.getModules().getModule(ModuleUtil.ModulePosition.FRONT_LEFT).getConstants().wheelDiameterMeters() / 2,
				swerve.getConstants().velocityAt12VoltsMetersPerSecond(),
				RobotConstants.WHEEL_COF,
				DCMotor.getKrakenX60Foc(1),
				KrakenX60DriveBuilder.GEAR_RATIO,
				KrakenX60DriveBuilder.SLIP_CURRENT,
				1
			),
			swerve.getModules().getModulePositionsFromCenterMeters()
		);
	}

	private void updateAllSubsystems() {
		flyWheel.update();
		turret.update();
		hood.update();
		pivot.update();
		intakeRoller.update();
		magazine.update();
		conveyor.update();
		upperRoller.update();
		swerve.update();
	}

	public boolean isTurretMoveLegal() {
		return TurretCalculations.isTurretMoveLegal(ShootingCalculations.getShootingParams().targetTurretPosition(), turret.getPosition());
	}

	public void periodic() {
		BusChain.refreshAll();
		updateAllSubsystems();
		robotCommander.update();

		poseEstimator.updateOdometry(swerve.getAllOdometryData());

		getLimelights().forEach(Limelight::updateHardwareInputs);
		getLimelights().forEach(Limelight::updateMT1);
		getLimelights().forEach(limelight -> limelight.getIndependentRobotPose().ifPresent(poseEstimator::updateVision));

		poseEstimator.log();
		ShootingCalculations
			.updateShootingParams(poseEstimator.getEstimatedPose(), swerve.getFieldRelativeVelocity(), swerve.getIMUAngularVelocityRPS()[2]);

		Logger.recordOutput("lastBallThrownTimestamp", lastBallThrownTimestamp.get());
		Logger.recordOutput(
			"TimeSinceLastBall",
			TimeUtil.getCurrentTimeSeconds()
				- (ballsBufferIncludingPassing.getInternalBuffer().floorKey(TimeUtil.getCurrentTimeSeconds()) == null
					? TimeUtil.getCurrentTimeSeconds()
					: ballsBufferIncludingPassing.getInternalBuffer().floorKey(TimeUtil.getCurrentTimeSeconds()))
		);
		Logger.recordOutput("BallCounterIncludingPassing", ballCounterIncludingPassing);
		Logger.recordOutput("BallCounterWithoutPassing", ballCounterWithoutPassing);
		Logger.recordOutput("CurrentBPS", getAverageBPSForLastXSeconds(RobotConstants.TIME_FOR_AVERAGE_BPS_CALCULATION_SECONDS));

		GamePeriodUtils.log();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public double getAverageBPSForLastXSeconds(double seconds) {
		if (ballsBufferIncludingPassing.getSample(TimeUtil.getCurrentTimeSeconds() - seconds).isPresent()) {
			return (ballCounterIncludingPassing - ballsBufferIncludingPassing.getSample(TimeUtil.getCurrentTimeSeconds() - seconds).get())
				/ seconds;
		}
		return 0;
	}

	public FlyWheel getFlyWheel() {
		return flyWheel;
	}

	public VelocityPositionArm getTurret() {
		return turret;
	}

	public Arm getHood() {
		return hood;
	}

	public CurrentControlArm getPivot() {
		return pivot;
	}

	public Roller getIntakeRoller() {
		return intakeRoller;
	}

	public VelocityRoller getMagazine() {
		return magazine;
	}

	public Roller getConveyor() {
		return conveyor;
	}

	public Roller getUpperRoller() {
		return upperRoller;
	}

	public IDigitalInput getMagazineBallSensor() {
		return magazineBallSensor;
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public IPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

	public Limelight getLimelightFront() {
		return limelightFront;
	}

	public Limelight getLimelightRight() {
		return limelightRight;
	}

	public Limelight getLimelightLeft() {
		return limelightLeft;
	}

	public List<Limelight> getLimelights() {
		return limelights;
	}

	public RobotCommander getRobotCommander() {
		return robotCommander;
	}

	public SimulationManager getSimulationManager() {
		return simulationManager;
	}

	public AutonomousChooser getAutonomousChooser() {
		return autonomousChooser;
	}

	public SendableChooser<Boolean> getReturnToMiddleChooser() {
		return returnToMiddle;
	}

	public Supplier<Double> getLastBallThrownTimestamp() {
		return lastBallThrownTimestamp;
	}

	public TimeInterpolatableBuffer<Double> getBallsBufferIncludingPassing() {
		return ballsBufferIncludingPassing;
	}

	public TimeInterpolatableBuffer<Double> getBallsBufferWithoutPassing() {
		return ballsBufferWithoutPassing;
	}

	private void configureBrakeStateChooser() {
		SendableChooser<BrakeMode> brakeStateChooser = new SendableChooser<>();
		brakeStateChooser.setDefaultOption("Coast", BrakeMode.COAST);
		brakeStateChooser.addOption("Brake", BrakeMode.BRAKE);
		SmartDashboard.putData("BrakeState", brakeStateChooser);
		brakeStateChooser.onChange(BrakeStateManager::setBrakeMode);
	}

	private void configureAuto() {
		Supplier<Command> autonomousOpenIntakeCommand = () -> getRobotCommander().getIntakeStateHandler().setState(IntakeState.INTAKE);
		Supplier<Command> autonomousCloseIntakeCommand = () -> getRobotCommander().getIntakeStateHandler().setState(IntakeState.CLOSED);

		Supplier<Command> autonomousScoringSequenceCommand = () -> getRobotCommander().scoreSequence();
		Supplier<Command> autonomousPassSequenceCommand = () -> getRobotCommander().passSequence();

		Supplier<Command> autonomousResetSubsystemsCommand = () -> getRobotCommander().setState(RobotState.RESET_SUBSYSTEMS);

		Supplier<Command> autonomousOuttakeCommand = () -> getRobotCommander().getIntakeStateHandler().setState(IntakeState.OUTTAKE);

		getSwerve().configPathPlanner(() -> getPoseEstimator().getEstimatedPose(), (pose) -> {}, getRobotConfig());

		this.returnToMiddle = new SendableChooser<>();
		returnToMiddle.setDefaultOption("Don't Return To Middle", false);
		returnToMiddle.addOption("Return To Middle", true);
		SmartDashboard.putData("Return To Middle", returnToMiddle);

		List<Supplier<PathPlannerAutoWrapper>> autos = AutosBuilder.getAutoList(
			this,
			autonomousResetSubsystemsCommand,
			autonomousOpenIntakeCommand,
			autonomousCloseIntakeCommand,
			autonomousScoringSequenceCommand,
			autonomousPassSequenceCommand,
			autonomousOuttakeCommand,
			AutonomousConstants.DEFAULT_PATHFINDING_CONSTRAINTS,
			AutonomousConstants.DEFAULT_IS_NEAR_END_OF_PATH_TOLERANCE,
			AutonomousConstants.DEFAULT_STUCK_IS_NEAR_END_OF_PATH_TOLERANCE,
			AutonomousConstants.DEFAULT_STUCK_DEBOUNCE_SECONDS,
			() -> returnToMiddle.getSelected() != null && returnToMiddle.getSelected()
		);

		this.autonomousChooser = new AutonomousChooser("Autonomous Chooser", autos);
	}

}

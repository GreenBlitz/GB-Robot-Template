// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.constants.ConstantsFactory;
import frc.robot.subsystems.swerve.factories.GyroFactory;
import frc.robot.subsystems.swerve.factories.ModulesFactory;
import frc.robot.superstructers.poseestimator.PoseEstimatorSuperstructure;
import frc.utils.RobotTypeUtils;


public class Robot {

    public static final RobotTypeUtils.RobotType ROBOT_TYPE = RobotTypeUtils.determineRobotType(RobotTypeUtils.RobotType.REAL);


    private static final SwerveConstants swerveConstants = ConstantsFactory.create();
    public static final Swerve swerve = new Swerve(
            swerveConstants,
            ModulesFactory.create(swerveConstants.getModuleConstants()),
            GyroFactory.create(swerveConstants.getGyroConstants())
    );
    public static final PoseEstimatorSuperstructure poseEstimator = new PoseEstimatorSuperstructure(swerve);

    public Robot() {
        buildPathPlannerForAuto();
        configureBindings();
    }

    private void initializeSubsystems() {

    }

    private void configureCommands() {

    }

    private void buildPathPlannerForAuto() {
        // Register commands...
        swerve.buildPathPlannerForAuto(poseEstimator::getCurrentPose, poseEstimator::resetPose);
    }

    private void configureBindings() {
        JoysticksBindings.configureBindings();
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }

}

package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.MathConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.modules.ModuleUtils;

import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.subsystems.swerve.swervestatehelpers.AimAssistUtils.getRotationAssistedSpeeds;
import static frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis.BACK_LEFT_MODULE;
import static frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis.BACK_RIGHT_MODULE;
import static frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis.FRONT_LEFT_MODULE;
import static frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis.FRONT_RIGHT_MODULE;

public class SwerveStateHelper {

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Swerve swerve;
    private final SwerveConstants swerveConstants;
    private final Supplier<Optional<Translation2d>> noteTranslationSupplier;

    public SwerveStateHelper(Supplier<Pose2d> robotPoseSupplier, Supplier<Optional<Translation2d>> noteTranslationSupplier,
            Swerve swerve) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.swerve = swerve;
        this.swerveConstants = swerve.getConstants();
        this.noteTranslationSupplier = noteTranslationSupplier;
    }

    public ChassisSpeeds applyAimAssistOnInputsSpeeds(AimAssist aimAssistState, ChassisSpeeds inputSpeeds) {
        return switch (aimAssistState) {
            case SPEAKER -> getRotationAssistedSpeeds(
                    inputSpeeds,
                    swerve.getRobotRelativeVelocity(),
                    () -> robotPoseSupplier.get().getRotation(),
                    () -> SwerveMath.getRelativeTranslation(robotPoseSupplier.get().getTranslation(),new Translation2d(0,0)).getAngle(),
                    swerveConstants
            );
            case AMP -> getRotationAssistedSpeeds(
                    inputSpeeds,
                    swerve.getRobotRelativeVelocity(),
                    () -> robotPoseSupplier.get().getRotation(),
                    () -> Rotation2d.fromDegrees(90),
                    swerveConstants
            );
            case NOTE -> handleNoteAssistState(
                    inputSpeeds,
                    robotPoseSupplier,
                    noteTranslationSupplier
            );
            case NONE -> inputSpeeds;
        };
    }


    private ChassisSpeeds handleNoteAssistState(ChassisSpeeds inputSpeeds,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<Optional<Translation2d>> noteTranslationSupplier) {
        if (noteTranslationSupplier.get().isEmpty()) {
            return inputSpeeds;
        }
        Translation2d noteRelativeToRobot = SwerveMath.getPoseRelativeTranslation(robotPoseSupplier.get(),
                noteTranslationSupplier.get().get());


        double wantedHorizontalVelocity = swerveConstants.yMetersPIDController().calculate(0, noteRelativeToRobot.getY())
                + inputSpeeds.vyMetersPerSecond;

        return ChassisSpeeds.fromRobotRelativeSpeeds(
                inputSpeeds.vxMetersPerSecond,
                wantedHorizontalVelocity,
                inputSpeeds.omegaRadiansPerSecond,
                robotPoseSupplier.get().getRotation()
        );
    }


    public Translation2d getRotationAxis(RotateAxis rotationAxisState) {
        return switch (rotationAxisState) {
            case MIDDLE_OF_ROBOT -> new Translation2d();
            case FRONT_LEFT_MODULE -> swerveConstants.LOCATIONS[ModuleUtils.ModuleName.FRONT_LEFT.getIndex()];
            case FRONT_RIGHT_MODULE -> swerveConstants.LOCATIONS[ModuleUtils.ModuleName.FRONT_RIGHT.getIndex()];
            case BACK_LEFT_MODULE -> swerveConstants.LOCATIONS[ModuleUtils.ModuleName.BACK_LEFT.getIndex()];
            case BACK_RIGHT_MODULE -> swerveConstants.LOCATIONS[ModuleUtils.ModuleName.BACK_RIGHT.getIndex()];
        };
    }
    public RotateAxis getFarRotateAxis(boolean isLeft) {
        Rotation2d currentAllianceAngle = swerve.getAllianceRelativeHeading();
        if (Math.abs(currentAllianceAngle.getDegrees()) <= MathConstants.EIGHTH_CIRCLE.getDegrees()) { // -45 <= x <= 45
            return isLeft ? FRONT_LEFT_MODULE : FRONT_RIGHT_MODULE;
        }
        if (Math.abs(currentAllianceAngle.getDegrees()) >= MathConstants.EIGHTH_CIRCLE.getDegrees() * 3) { // -135 - x - 135
            return isLeft ? BACK_RIGHT_MODULE : BACK_LEFT_MODULE;
        }
        if (currentAllianceAngle.getDegrees() > 0) { // 45 <= x <= 135
            return isLeft ? FRONT_RIGHT_MODULE : BACK_RIGHT_MODULE;
        }
        return isLeft ? BACK_LEFT_MODULE : FRONT_LEFT_MODULE; // -45 >= x >= -135
    }

    public RotateAxis getFarRightRotateAxis() {
        return getFarRotateAxis(false);
    }

    public RotateAxis getFarLeftRotateAxis() {
        return getFarRotateAxis(true);
    }
}

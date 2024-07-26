package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;

import java.util.function.Function;
import java.util.function.Supplier;

import static frc.robot.subsystems.swerve.swervestatehelpers.AimAssistUtils.getRotationAssistedSpeeds;

public class SwerveStateHandler {

    private final Supplier<Pose2d> robotPoseSupplier;
    private Swerve swerve;
    private SwerveConstants swerveConstants;

    public SwerveStateHandler(Supplier<Pose2d> robotPoseSupplier, Swerve swerve) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.swerve = swerve;
        this.swerveConstants = swerve.getConstants();
    }

    public ChassisSpeeds applyStateOnInputsSpeeds (AimAssist aimAssistState, ChassisSpeeds inputSpeeds, ChassisSpeeds currentSpeeds){
        return switch(aimAssistState){
            case SPEAKER -> handleSpeakerState(
                    inputSpeeds,
                    swerve.getFieldRelativeVelocity(),
                    robotPoseSupplier,
                    AimAssist.SPEAKER.targetRotationSupplier,
                    swerveConstants
            );
            case AMP ->handleAmpState(
                    inputSpeeds,
                    currentSpeeds,
                    robotPoseSupplier,
                    AimAssist.AMP.targetRotationSupplier,
                    swerveConstants
            );
            case NONE -> inputSpeeds;
        };
    }


    private static ChassisSpeeds handleAmpState(ChassisSpeeds inputSpeeds, ChassisSpeeds currentSpeeds,
            Supplier<Pose2d> robotPoseSupplier,
            Function<Pose2d, Rotation2d> targetRotationSupplier, SwerveConstants swerveConstants) {
        return getRotationAssistedSpeeds(
                inputSpeeds,
                currentSpeeds,
                robotPoseSupplier,
                targetRotationSupplier,
                swerveConstants
        );
    }

    private static ChassisSpeeds handleSpeakerState(ChassisSpeeds inputSpeeds, ChassisSpeeds currentSpeeds,
            Supplier<Pose2d> robotPoseSupplier,
            Function<Pose2d ,Rotation2d> targetRotationSupplier, SwerveConstants swerveConstants) {
        return getRotationAssistedSpeeds(
                inputSpeeds,
                currentSpeeds,
                robotPoseSupplier,
                targetRotationSupplier,
                swerveConstants
        );
    }

}

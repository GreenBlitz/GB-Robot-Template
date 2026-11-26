package frc.robot.vision.interfaces;

import frc.robot.vision.cameras.limelight.Limelight;
import frc.robot.vision.cameras.limelight.LimelightPipeline;
import frc.utils.Filter;
import frc.utils.limelight.LimelightHelpers;
import frc.utils.math.StandardDeviations2D;

import java.util.function.Function;
import java.util.function.Supplier;

public interface IndependentCamera extends IndependentRobotPoseSupplier, ObjectDetector, OrientationRequiringRobotPoseSupplier {

        void update(LimelightPipeline pipeline);

        String getName();

        void updateHardwareInputs();

        void updateMT1();

        void updateMT2();

        Filter getMt1PoseFilter();

        Filter getMt2PoseFilter();

        void setNeuralDetectionFilter(Function<LimelightHelpers.RawDetection, Boolean> neuralDetectionFilter);

        void setColorDetectionFilter(Function<LimelightHelpers.RawTarget, Boolean> colorDetectionFilter);

        void setMT1PoseFilter(Filter mt1RobotPoseFilter);

        void setMT2PoseFilter(Filter mt2RobotPoseFilter);

        void setMT1StdDevsCalculation(
                Supplier<StandardDeviations2D> calculateMT1StdDevs
        );

        void setMT2StdDevsCalculation(
                Supplier<StandardDeviations2D> calculateMT2StdDevs
        );

        void setColorDetectionFilter();

        void setNeuralDetectionFilter();

        void updateColorDetection();

        void updateNeuralDetection();

}

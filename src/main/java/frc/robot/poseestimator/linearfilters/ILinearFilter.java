package frc.robot.poseestimator.linearfilters;

public interface ILinearFilter<dataType> {

	dataType calculateNewData(dataType input);

}

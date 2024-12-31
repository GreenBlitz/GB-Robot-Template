package frc.robot.poseestimator.helpers.dataswitcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.helpers.ProcessedVisionData;
import frc.robot.poseestimator.helpers.StandardDeviations2D;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class VisionObservationSwitcher implements IDataSwitcher<ProcessedVisionData> {

	private final Function<Double, Double> timeToWeights;
	private final double timeToSwitchSeconds;

	private final DoubleSwitcher XSwitcher;
	private final DoubleSwitcher YSwitcher;
	private final DoubleSwitcher AngleSwitcherRadians;

	private final DoubleSwitcher XStaDevsSwitcher;
	private final DoubleSwitcher YStaDevsSwitcher;
	private final DoubleSwitcher AngleStaDevsSwitcher;

	private final DoubleSwitcher timestampSwitcher;

	public VisionObservationSwitcher(
		Supplier<Optional<ProcessedVisionData>> firstSource,
		Supplier<Optional<ProcessedVisionData>> secondSource,
		Function<Double, Double> timeToWeights,
		double timeToSwitchSeconds
	) {
		this.timeToWeights = timeToWeights;
		this.timeToSwitchSeconds = timeToSwitchSeconds;

		this.XSwitcher = createDataSwitcher(
			() -> (firstSource.get().isPresent() ? Optional.of(firstSource.get().get().getEstimatedPose().getX()) : Optional.empty()),
			() -> (secondSource.get().isPresent() ? Optional.of(secondSource.get().get().getEstimatedPose().getX()) : Optional.empty())
		);
		this.YSwitcher = createDataSwitcher(
			() -> (firstSource.get().isPresent() ? Optional.of(firstSource.get().get().getEstimatedPose().getY()) : Optional.empty()),
			() -> (secondSource.get().isPresent() ? Optional.of(secondSource.get().get().getEstimatedPose().getY()) : Optional.empty())
		);
		this.AngleSwitcherRadians = createDataSwitcher(
			() -> (firstSource.get().isPresent()
				? Optional.of(firstSource.get().get().getEstimatedPose().getRotation().getRadians())
				: Optional.empty()),
			() -> (secondSource.get().isPresent()
				? Optional.of(secondSource.get().get().getEstimatedPose().getRotation().getRadians())
				: Optional.empty())
		);
		this.XStaDevsSwitcher = createDataSwitcher(
			extractStdDevFromVisionObservation(firstSource, 0),
			extractStdDevFromVisionObservation(secondSource, 0)
		);
		this.YStaDevsSwitcher = createDataSwitcher(
			extractStdDevFromVisionObservation(firstSource, 1),
			extractStdDevFromVisionObservation(secondSource, 1)
		);
		this.AngleStaDevsSwitcher = createDataSwitcher(
			extractStdDevFromVisionObservation(firstSource, 2),
			extractStdDevFromVisionObservation(secondSource, 2)
		);

		this.timestampSwitcher = new DoubleSwitcher(
			() -> (firstSource.get().isPresent() ? Optional.of(firstSource.get().get().getTimestamp()) : Optional.empty()),
			() -> (firstSource.get().isPresent() ? Optional.of(firstSource.get().get().getTimestamp()) : Optional.empty()),
			timeToWeights,
			timeToSwitchSeconds
		);
	}

	private DoubleSwitcher createDataSwitcher(Supplier<Optional<Double>> firstSource, Supplier<Optional<Double>> secondSource) {
		return new DoubleSwitcher(firstSource, secondSource, timeToWeights, timeToSwitchSeconds);
	}

	private static Supplier<Optional<Double>> extractStdDevFromVisionObservation(Supplier<Optional<ProcessedVisionData>> source, int index) {
		return () -> source.get().isPresent() ? Optional.of(source.get().get().getStdDev().yStandardDeviations()) : Optional.empty();
	}

	@Override
	public Optional<ProcessedVisionData> getValue(double time) {
		var x = XSwitcher.getValue(time);
		var y = YSwitcher.getValue(time);
		var angle = AngleSwitcherRadians.getValue(time);
		var xStdDev = XStaDevsSwitcher.getValue(time);
		var yStdDev = YStaDevsSwitcher.getValue(time);
		var angleStdDev = AngleStaDevsSwitcher.getValue(time);
		var timestamp = timestampSwitcher.getValue(time);

		if (
			x.isEmpty()
				|| y.isEmpty()
				|| angle.isEmpty()
				|| xStdDev.isEmpty()
				|| yStdDev.isEmpty()
				|| angleStdDev.isEmpty()
				|| timestamp.isEmpty()
		) {
			return Optional.empty();
		}

		return Optional.of(
			new ProcessedVisionData(
				new Pose2d(x.get(), y.get(), Rotation2d.fromRadians(angle.get())),
				timestamp.get(),
				new StandardDeviations2D(xStdDev.get(), yStdDev.get(), angleStdDev.get())
			)
		);
	}

	@Override
	public void switchSources() {
		XSwitcher.switchSources();
		YSwitcher.switchSources();
		AngleSwitcherRadians.switchSources();
		XStaDevsSwitcher.switchSources();
		YStaDevsSwitcher.switchSources();
		AngleStaDevsSwitcher.switchSources();
		timestampSwitcher.switchSources();
	}

	@Override
	public void switchToFirstSource() {
		XSwitcher.switchToFirstSource();
		YSwitcher.switchToFirstSource();
		AngleSwitcherRadians.switchToFirstSource();
		XStaDevsSwitcher.switchToFirstSource();
		YStaDevsSwitcher.switchToFirstSource();
		AngleStaDevsSwitcher.switchToFirstSource();
		timestampSwitcher.switchToFirstSource();
	}

	@Override
	public void switchToSecondsSource() {
		XSwitcher.switchToSecondsSource();
		YSwitcher.switchToSecondsSource();
		AngleSwitcherRadians.switchToSecondsSource();
		XStaDevsSwitcher.switchToSecondsSource();
		YStaDevsSwitcher.switchToSecondsSource();
		AngleStaDevsSwitcher.switchToSecondsSource();
		timestampSwitcher.switchToSecondsSource();
	}

	@Override
	public boolean isFirstSourceBeingUsed() {
		return XSwitcher.isFirstSourceBeingUsed();
	}

}

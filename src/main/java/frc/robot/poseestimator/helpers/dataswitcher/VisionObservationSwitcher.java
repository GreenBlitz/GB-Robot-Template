package frc.robot.poseestimator.helpers.dataswitcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.observations.VisionObservation;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class VisionObservationSwitcher implements IDataSwitcher<VisionObservation> {

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
		Supplier<Optional<VisionObservation>> firstSource,
		Supplier<Optional<VisionObservation>> secondSource,
		Function<Double, Double> timeToWeights,
		double timeToSwitchSeconds
	) {
		this.timeToWeights = timeToWeights;
		this.timeToSwitchSeconds = timeToSwitchSeconds;

		this.XSwitcher = createDataSwitcher(
			() -> (firstSource.get().isPresent() ? Optional.of(firstSource.get().get().robotPose().getX()) : Optional.empty()),
			() -> (secondSource.get().isPresent() ? Optional.of(secondSource.get().get().robotPose().getX()) : Optional.empty())
		);
		this.YSwitcher = createDataSwitcher(
			() -> (firstSource.get().isPresent() ? Optional.of(firstSource.get().get().robotPose().getY()) : Optional.empty()),
			() -> (secondSource.get().isPresent() ? Optional.of(secondSource.get().get().robotPose().getY()) : Optional.empty())
		);
		this.AngleSwitcherRadians = createDataSwitcher(
			() -> (firstSource.get().isPresent()
				? Optional.of(firstSource.get().get().robotPose().getRotation().getRadians())
				: Optional.empty()),
			() -> (secondSource.get().isPresent()
				? Optional.of(secondSource.get().get().robotPose().getRotation().getRadians())
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
			() -> (firstSource.get().isPresent() ? Optional.of(firstSource.get().get().timestamp()) : Optional.empty()),
			() -> (firstSource.get().isPresent() ? Optional.of(firstSource.get().get().timestamp()) : Optional.empty()),
			timeToWeights,
			timeToSwitchSeconds
		);
	}

	private DoubleSwitcher createDataSwitcher(Supplier<Optional<Double>> firstSource, Supplier<Optional<Double>> secondSource) {
		return new DoubleSwitcher(firstSource, secondSource, timeToWeights, timeToSwitchSeconds);
	}

	private static Supplier<Optional<Double>> extractStdDevFromVisionObservation(Supplier<Optional<VisionObservation>> source, int index) {
		return () -> source.get().isPresent() ? Optional.of(source.get().get().standardDeviations()[index]) : Optional.empty();
	}

	@Override
	public Optional<VisionObservation> getValue(double time) {
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
			new VisionObservation(
				new Pose2d(x.get(), y.get(), Rotation2d.fromRadians(angle.get())),
				new double[] {xStdDev.get(), yStdDev.get(), angleStdDev.get()},
				timestamp.get()
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

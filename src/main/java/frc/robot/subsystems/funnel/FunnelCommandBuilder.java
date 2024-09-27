package frc.robot.subsystems.funnel;

public class FunnelCommandBuilder {

    private final Pivot pivot;

    public PivotCommandsBuilder(Pivot pivot) {
        this.pivot = pivot;
    }

    //@formatter:off
    public Command setPower(double power) {
        return new FunctionalCommand(
                () -> {},
                () -> pivot.setPower(power),
                (interrupted) -> pivot.stop(),
                () -> false, pivot
        ).withName("Set power to: " + power);
    }

    public Command setPower(DoubleSupplier power) {
        return new FunctionalCommand(
                () -> {},
                () -> pivot.setPower(power.getAsDouble()),
                (interrupted) -> pivot.stop(),
                () -> false, pivot
        ).withName("Set power by supplier");
    }
    //@formatter:on

    public Command moveToPosition(Rotation2d position, Rotation2d tolerance) {
        return new FunctionalCommand(
                () -> pivot.setTargetPosition(position),
                () -> {},
                (interrupted) -> pivot.stayInPlace(),
                () -> pivot.isAtPosition(position, tolerance),
                pivot
        ).withName("Move to position: " + position);
    }

    public Command moveToPosition(Supplier<Rotation2d> positionSupplier) {
        return new FunctionalCommand(
                () -> {},
                () -> pivot.setTargetPosition(positionSupplier.get()),
                (interrupted) -> pivot.stayInPlace(),
                () -> false,
                pivot
        ).withName("Move to supplier position");
    }

    public Command stop() {
        return new InstantCommand(pivot::stop, pivot).withName("Stop");
    }

}

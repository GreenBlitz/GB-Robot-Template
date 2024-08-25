package frc.utils.beamBreaker.consumerBeamBreaker;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.utils.beamBreaker.BeamBreakerInputsAutoLogged;
import frc.utils.beamBreaker.IBeamBreaker;

import java.util.function.Supplier;

public class SuppliedBeamBreaker implements IBeamBreaker {

    private static DigitalInput input;

    private Debouncer debouncer;

    private final Supplier<Boolean> isObstructedConsumer;

    public SuppliedBeamBreaker(Supplier<Boolean> isObstructedConsumer){
        this.isObstructedConsumer=isObstructedConsumer;
        input = new DigitalInput(0);
        debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    }

    @Override
    public void updateInputs(BeamBreakerInputsAutoLogged inputs) {
        inputs.isObstructed = debouncer.calculate(input.get());
    }




}

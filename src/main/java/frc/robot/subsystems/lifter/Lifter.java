package frc.robot.subsystems.lifter;

import edu.wpi.first.math.MathUtil;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.utils.GBSubsystem;

public class Lifter extends GBSubsystem {

    private final ControllableMotor motor;
    private final LifterStuff lifterStuff;
    private final LifterCommandsBuilder lifterCommandsBuilder;

    public Lifter(LifterStuff lifterStuff){
        super(lifterStuff.logPath());
        this.motor = lifterStuff.motor();
        this.lifterStuff = lifterStuff;
        this.lifterCommandsBuilder = new LifterCommandsBuilder(this);
    }

    public void setPower(double power){
        motor.setPower(power);
    }
    public void stop(){
        motor.stop();
    }

    public void setBrake(boolean brake){
        motor.setBrake(brake);
    }

    public void setTargetPosition(double position){
        motor.applyDoubleRequest(lifterStuff.positionRequest().withSetPoint(position));
    }

    public boolean isAtTargetPosition(double expectedPosition, double positionTolerance){
        return MathUtil.isNear(
                expectedPosition,
                lifterStuff.positionSignal().getLatestValue(),
                positionTolerance
        );
    }

    public LifterStuff getLifterStuff() {
        return lifterStuff;
    }

    public LifterCommandsBuilder getLifterCommandsBuilder() {
        return lifterCommandsBuilder;
    }

    @Override
    protected void subsystemPeriodic() {
        motor.updateSignals(lifterStuff.positionSignal());
        motor.updateSignals(lifterStuff.otherSignals());
    }
}

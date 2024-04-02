package frc.robot.commands.calibration;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.commands.GBCommand;
import frc.utils.devicewrappers.GBTalonFXPro;

public class FindP extends GBCommand {

    private final GBTalonFXPro motor;
    private final PositionVoltage controlRequest;
    private final double targetValue1;
    private final double targetValue2;
    private double usedTargetValue;
    private final Slot0Configs slot0Configs;

    private final double tolerance;

    private final Timer timer;
    private final double timeout;

    private final double changePFactor;

    private final double minErrorRange;
    private final double maxErrorRange;

    private boolean isCheckingMin;
    private double edgeValue;


    private double accuracy;

    private boolean isInit;
    private boolean isExe;
    private boolean isEnd;

    //Todo - add motor sub sys with SYSID and the functions: getVelocity, setVelocity, getPosition, setPosition, getPValue, setPValue
    //Todo - instead of getting motor and control get motorSubSys

    public FindP(GBTalonFXPro motor, PositionVoltage controlRequest, double targetValue1, double targetValue2, double timeout, double tolerance, double factor, double startRange, double endRange) {
        this.motor = motor;
        this.controlRequest = controlRequest;
        this.targetValue1 = targetValue1;
        this.targetValue2 = targetValue2;
        this.slot0Configs = new Slot0Configs();

        this.timer = new Timer();
        this.timeout = timeout;

        this.minErrorRange = startRange;
        this.maxErrorRange = endRange;

        this.changePFactor = factor;
        this.tolerance = tolerance;

        this.usedTargetValue = targetValue2;

        this.accuracy = 0;
    }

    public double getTargetValue(double lastUsedTargetValue) {
        return lastUsedTargetValue == targetValue2 ? targetValue1 : targetValue2;
    }

    private void setIsInit(boolean isInit){
        this.isInit = isInit;
        if (isInit){
            isEnd = false;
            isExe = false;
        }
    }
    private void setIsExe(boolean isExe){
        this.isExe = isExe;
        if (isExe){
            isInit = false;
            isEnd = false;
        }
    }
    private void setIsEnd(boolean isEnd){
        this.isEnd = isEnd;
        if (isEnd){
            isInit = false;
            isExe = false;
        }
    }

    @Override
    public void execute() {
        if (isInit) {
            accuracy = 0;
            timer.restart();

            usedTargetValue = getTargetValue(usedTargetValue);
            isCheckingMin = motor.getPosition().refresh().getValue() > usedTargetValue;
            edgeValue = motor.getPosition().refresh().getValue();
            motor.setControl(controlRequest.withPosition(usedTargetValue));

            setIsExe(true);
        }

        else if (isExe) {
            if (isCheckingMin) {
                if (edgeValue > motor.getPosition().refresh().getValue()) {
                    edgeValue = motor.getPosition().refresh().getValue();
                }
            } else {
                if (edgeValue < motor.getPosition().refresh().getValue()) {
                    edgeValue = motor.getPosition().refresh().getValue();
                }
            }
            setIsEnd(Math.abs(usedTargetValue - motor.getPosition().refresh().getValue()) <= tolerance || timer.hasElapsed(timeout));
        }

        else if (isEnd) {
            timer.stop();

            double sign = isCheckingMin ? Math.signum(edgeValue - usedTargetValue) : Math.signum(usedTargetValue - edgeValue);
            double error = Math.abs(edgeValue - usedTargetValue);

            motor.getConfigurator().refresh(slot0Configs);
            slot0Configs.kP += sign * error * changePFactor;
            motor.getConfigurator().apply(slot0Configs);

            accuracy = 100 - (100 / (maxErrorRange - minErrorRange + 1)) * error;

            if (accuracy < 90){
                setIsInit(true);
            }
        }
    }
    
    @Override
    public boolean isFinished() {
        return accuracy > 90;
    }

}

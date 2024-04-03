package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.IMotorSubsystem;
import frc.utils.commands.GBCommand;

public class FindP extends GBCommand {

    private final IMotorSubsystem motorSubsystem;
    private final double targetValue1;
    private final double targetValue2;
    private double usedTargetValue;
    private final int pidSlot;

    private final Timer timer;
    private final double timeoutForAction;

    private final double changePFactor;

    private final double minErrorRange;
    private final double maxErrorRange;

    private boolean isCheckingMin;
    private double edgeValue;

    private final double wantedAccuracy;
    private double accuracy;

    private boolean isInit;
    private boolean isExe;
    private boolean isEnd;

    //Todo - add motor sub sys with SYSID

    public FindP(IMotorSubsystem motorSubsystem, int pidSlot, double wantedAccuracy, double targetValue1, double targetValue2, double timeoutForAction, double factor, double startRange, double endRange) {
        this.motorSubsystem = motorSubsystem;
        this.pidSlot = pidSlot;

        this.targetValue1 = targetValue1;
        this.targetValue2 = targetValue2;
        this.usedTargetValue = targetValue2;

        this.timer = new Timer();
        this.timeoutForAction = timeoutForAction;

        this.minErrorRange = startRange;
        this.maxErrorRange = endRange;

        this.changePFactor = factor;

        this.wantedAccuracy = wantedAccuracy;
        this.accuracy = 0;
        
        this.isInit = true;
        this.isExe = false;
        this.isEnd = false;
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
            double currentPosition = motorSubsystem.getPosition();
            accuracy = 0;
            timer.restart();

            usedTargetValue = getTargetValue(usedTargetValue);
            isCheckingMin = currentPosition > usedTargetValue;
            edgeValue = currentPosition;
            motorSubsystem.setPositionControl(usedTargetValue);

            setIsExe(true);
        }

        else if (isExe) {
            double currentPosition = motorSubsystem.getPosition();

            if (isCheckingMin) {
                if (edgeValue > currentPosition) {
                    edgeValue = currentPosition;
                }
            } else {
                if (edgeValue < currentPosition) {
                    edgeValue = currentPosition;
                }
            }
            if (motorSubsystem.isAtPosition(usedTargetValue) || timer.hasElapsed(timeoutForAction)){
                setIsEnd(true);
            }
        }

        else if (isEnd) {
            timer.stop();

            double sign = isCheckingMin ? Math.signum(edgeValue - usedTargetValue) : Math.signum(usedTargetValue - edgeValue);
            double error = Math.abs(edgeValue - usedTargetValue);

            accuracy = 100 - (100 / (maxErrorRange - minErrorRange + 1)) * error;

            if (accuracy < wantedAccuracy){
                motorSubsystem.setPValue(motorSubsystem.getPValue(pidSlot) + sign * error / changePFactor, pidSlot);
                setIsInit(true);
            }
        }
    }
    
    @Override
    public boolean isFinished() {
        return accuracy > wantedAccuracy;
    }
    
    @Override
    public void end(boolean interrupted) {
        motorSubsystem.stop();
    }
}

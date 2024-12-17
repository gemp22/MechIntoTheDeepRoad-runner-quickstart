package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    public double setPoint = 0, minInput = 0, maxInput = 0, minOutput = 0;
    public double maxOutput = 0, thresholdPercent = 0;
    private double currentError = 0;

    private double derivative = 0;
    private double Kp;
    private double Ki;
    private double Kd;

    private double integralSum = 0;
    private double intergralLimit = .25;
    private double lastError = 0;

    ElapsedTime PIDtimer = new ElapsedTime();



    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void setThresholdValuePID (double thresholdPercent) {
        this.thresholdPercent = thresholdPercent; }
    public void setSetPointPID(double setPoint) {
        this.setPoint = setPoint;
    }
    public void setInputRangePID(double minInput, double maxInput) {
        this.minInput = Math.abs(minInput);
        this.maxInput = Math.abs(maxInput);
    }
    public void setOutputRangePID(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }
    public double getComputedOutputPID(double input) {
        // get the error as absolute value - the sign of error is immaterial since
        // that gets taken care of where the PID Controller value is used
        currentError = Math.abs(Math.abs(setPoint) - Math.abs(input));

        derivative = (currentError - lastError) / PIDtimer.seconds();

        integralSum = integralSum + (currentError * PIDtimer.seconds());

        if(integralSum>intergralLimit) {
            integralSum=intergralLimit;
        }
        if(integralSum<-intergralLimit) { // this if probably isn't need since taken care of where the PID Controller value is used
            integralSum=-intergralLimit;
        }

        double computedOutput =  ((currentError * Kp) + (Ki * integralSum) + (Kd * derivative))* (maxOutput - minOutput);

        if (computedOutput > (maxOutput-minOutput)) {
            computedOutput = (maxOutput-minOutput);
        }

        lastError = currentError;

        PIDtimer.reset();

        return computedOutput;

    }
    public boolean hasPIDControllerReachedTarget() {
        double percentDifferenceFromTarget =
                (currentError /(maxInput - minInput))*100;
        if(percentDifferenceFromTarget < thresholdPercent) {
            return true;
        }
        return false;
    }
}

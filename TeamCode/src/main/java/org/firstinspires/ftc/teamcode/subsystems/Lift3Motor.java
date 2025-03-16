package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants3Motor;
import org.firstinspires.ftc.teamcode.PController;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.Robot3Motor;

public class Lift3Motor {  //this is a subsystem Class used in Auto. its based on example for RR actions.
    public DcMotorEx liftLeft;
    public DcMotorEx liftRight;
    public DcMotorEx upperLift;
    public int liftLeftPosition = 0;
    public int liftRightPosition = 0;
    public int upperLiftPosition = 0;
    private final double liftLeftMaxTicks = 2370;
    private final double liftRightMaxTicks = 2370;
    private final double upperLiftMaxTicks = 2370;
    // for lift pController input
    public final double minPowerLiftLeft = .001;
    public final double minPowerLiftRight = .001;
    public final double minPowerUpperLift = .001;
    public final double maxPowerUpperLift = 1;
    public final double maxPowerLiftLeft = 1;
    public final double maxPowerLiftRight = 1;
    //private double liftLeftActionPosition;
    //private double liftRightActionPosition;

    // variable for RR1.0 actions
    //private double pixelLiftPosition; // variable for regular use like on init.. humm may not need since its defined in the method?
    //private boolean exitLiftLeftPControllerLoop = false;
    //private boolean exitLiftRightPControllerLoop = false;

    public PController pControllerLiftLeft = new PController(0.005);
    public PController pControllerLiftRight = new PController(0.005);
    public PController pControllerUpperLift = new PController(0.005);
    public PIDController pidControllerLiftLeft = new PIDController(0.005,.00007,0);
    public PIDController pidControllerLiftRight = new PIDController(0.005,0.00007,0);
    public PIDController pidControllerUpperLift = new PIDController(0.005,0.00007,0);

    public Lift3Motor(HardwareMap hardwareMap) {
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftLeft.setDirection(DcMotor.Direction.FORWARD);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setPower(0);
        //pixelLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // to reset at initiation
        //pixelLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(Robot3Motor.resetEncoders) {
            liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftRight.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setPower(0);

        if(Robot3Motor.resetEncoders) {
            liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        upperLift = hardwareMap.get(DcMotorEx.class, "upperLift");
        upperLift.setDirection(DcMotor.Direction.FORWARD);
        upperLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperLift.setPower(0);

        if(Robot3Motor.resetEncoders) {
            upperLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            upperLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            upperLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void initLiftPController(){

        pControllerLiftLeft.setInputRange(0, liftLeftMaxTicks);
        pControllerLiftLeft.setSetPoint(0);
        pControllerLiftLeft.setOutputRange(minPowerLiftLeft, maxPowerLiftLeft);
        pControllerLiftLeft.setThresholdValue(5);

        pControllerLiftRight.setInputRange(0, liftRightMaxTicks);
        pControllerLiftRight.setSetPoint(0);
        pControllerLiftRight.setOutputRange(minPowerLiftRight, maxPowerLiftRight);
        pControllerLiftRight.setThresholdValue(5);

        pControllerUpperLift.setInputRange(0, upperLiftMaxTicks);
        pControllerUpperLift.setSetPoint(0);
        pControllerUpperLift.setOutputRange(minPowerUpperLift, maxPowerUpperLift);
        pControllerUpperLift.setThresholdValue(5);
        //armPivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // this is for lift left, change Kp to calibrate
    }
    public void initLiftPIDController(){

        pidControllerLiftLeft.setInputRangePID(0, liftLeftMaxTicks);
        pidControllerLiftLeft.setSetPointPID(0);
        pidControllerLiftLeft.setOutputRangePID(minPowerLiftLeft, maxPowerLiftLeft);
        pidControllerLiftLeft.setThresholdValuePID(5);

        pidControllerLiftRight.setInputRangePID(0, liftRightMaxTicks);
        pidControllerLiftRight.setSetPointPID(0);
        pidControllerLiftRight.setOutputRangePID(minPowerLiftRight, maxPowerLiftRight);
        pidControllerLiftRight.setThresholdValuePID(5);

        pidControllerUpperLift.setInputRangePID(0, upperLiftMaxTicks);
        pidControllerUpperLift.setSetPointPID(0);
        pidControllerUpperLift.setOutputRangePID(minPowerUpperLift, maxPowerUpperLift);
        pidControllerUpperLift.setThresholdValuePID(5);
        //armPivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // this is for lift left, change Kp to calibrate
    }

    public void setLiftPosition() {
        liftLeftPosition=liftLeft.getCurrentPosition();
        liftRightPosition= liftRight.getCurrentPosition();
        upperLiftPosition = upperLift.getCurrentPosition();

    }

    public void setLiftPower(double liftPower) {
        liftLeft.setPower(liftPower);
        liftRight.setPower(liftPower);
        upperLift.setPower(liftPower);

    }

    public double getLiftExtension() {
        liftLeftPosition = liftLeft.getCurrentPosition();
        liftRightPosition= liftRight.getCurrentPosition();
        upperLiftPosition = liftRight.getCurrentPosition();

        double averageLiftPosition = (liftLeftPosition + liftRightPosition + upperLiftPosition)/3.0;

       return (averageLiftPosition / (127)); //in inches
    }

    public void setSetPoint(double in) {
        pControllerLiftLeft.setSetPoint(in * (127));
        pControllerLiftRight.setSetPoint(in * (127));
        pControllerUpperLift.setSetPoint(in * (127));
    }

    public void updateLiftPosition() {
        liftLeftPosition = liftLeft.getCurrentPosition();
        liftRightPosition = liftRight.getCurrentPosition();
        upperLiftPosition = upperLift.getCurrentPosition();

        if (liftLeftPosition < pControllerLiftLeft.setPoint) {

            liftLeft.setPower(minPowerLiftLeft +
                    pControllerLiftLeft.getComputedOutput(liftLeftPosition));
        } else {
            liftLeft.setPower(minPowerLiftLeft -
                    pControllerLiftLeft.getComputedOutput(liftLeftPosition));
        }

        if (liftRightPosition < pControllerLiftRight.setPoint) {

            liftRight.setPower(minPowerLiftRight +
                    pControllerLiftRight.getComputedOutput(liftRightPosition));
        } else {
            liftRight.setPower(minPowerLiftRight -
                    pControllerLiftRight.getComputedOutput(liftRightPosition));
        }

        if (upperLiftPosition < pControllerUpperLift.setPoint) {

            upperLift.setPower(minPowerUpperLift +
                    pControllerUpperLift.getComputedOutput(upperLiftPosition));
        } else {
            upperLift.setPower(minPowerUpperLift -
                    pControllerUpperLift.getComputedOutput(upperLiftPosition));
        }
    }
    public void updateLiftPositionPID() {
        liftLeftPosition=liftLeft.getCurrentPosition();
        liftRightPosition= liftRight.getCurrentPosition();
        upperLiftPosition = upperLift.getCurrentPosition();

        if (liftLeftPosition < pidControllerLiftLeft.setPoint) {

            liftLeft.setPower(minPowerLiftLeft +
                    pidControllerLiftLeft.getComputedOutputPID(liftLeftPosition));
        } else {
            liftLeft.setPower(minPowerLiftLeft -
                    pidControllerLiftLeft.getComputedOutputPID(liftLeftPosition));
        }


        if (liftRightPosition < pidControllerLiftRight.setPoint) {

            liftRight.setPower(minPowerLiftRight +
                    pidControllerLiftRight.getComputedOutputPID(liftRightPosition));
        } else {
            liftRight.setPower(minPowerLiftRight -
                    pidControllerLiftRight.getComputedOutputPID(liftRightPosition));
        }

        if (upperLiftPosition < pControllerUpperLift.setPoint) {

            upperLift.setPower(minPowerUpperLift +
                    pidControllerUpperLift.getComputedOutputPID(upperLiftPosition));
        } else {
            upperLift.setPower(minPowerUpperLift -
                    pidControllerUpperLift.getComputedOutputPID(upperLiftPosition));
        }
    }

    public void setLiftVelocityFromPivotVelocity(double pivotVelocity)
    {
        double velocity = pivotVelocity / 5.0611;

        liftRight.setVelocity(velocity);
        liftLeft.setVelocity(velocity);
        upperLift.setVelocity(velocity);
    }

    public void resetLiftEncoders(){
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /////////////////////////   Lift   /////////////////////////////////////






}

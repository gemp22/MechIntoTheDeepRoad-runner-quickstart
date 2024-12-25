package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {  //this is a subsystem Class used in Auto. its based on example for RR actions.
    public DcMotorEx liftLeft;
    public DcMotorEx liftRight;
    public int liftLeftPosition = 0;
    public int liftRightPosition = 0;
    private double liftLeftMaxTicks = 2370;
    private double liftRightMaxTicks = 2370;
    // for lift pController input
    public final double minPowerLiftLeft = .001;
    public final double minPowerLiftRight = .001;
    public final double maxPowerLiftLeft = 0.99;
    public final double maxPowerLiftRight = 0.99;
    private double liftLeftActionPosition;
    private double liftRightActionPosition;

    // variable for RR1.0 actions
    //private double pixelLiftPosition; // variable for regular use like on init.. humm may not need since its defined in the method?
    private boolean exitLiftLeftPControllerLoop = false;
    private boolean exitLiftRightPControllerLoop = false;
    public PController pControllerLiftLeft = new PController(0.005);
    public PController pControllerLiftRight = new PController(0.005);

    public PIDController pidControllerLiftLeft = new PIDController(0.005,.00007,0);
    public PIDController pidControllerLiftRight = new PIDController(0.005,0.00007,0);

    public Lift(HardwareMap hardwareMap) {
        liftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setPower(0);
        //pixelLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // to reset at initiation
        //pixelLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        liftRight = hardwareMap.get(DcMotorEx.class, "rightLift");
        liftRight.setDirection(DcMotor.Direction.FORWARD);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setPower(0);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
        //armPivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // this is for lift left, change Kp to calibrate
    }


    public void setLiftPosition() {
        liftLeftPosition=liftLeft.getCurrentPosition();
        liftRightPosition=liftRight.getCurrentPosition();

    }
    public void setLiftPower(double liftPower) {
        liftLeft.setPower(liftPower);
        liftRight.setPower(liftPower);

    }

    public void setSetPoint(double in) {
        pControllerLiftLeft.setSetPoint(in * 177.333);
        pControllerLiftRight.setSetPoint(in * 177.333);
    }

    public void updateLiftPosition() {
        liftLeftPosition=liftLeft.getCurrentPosition();
        liftRightPosition=liftRight.getCurrentPosition();

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
    }
    public void updateLiftPositionPID() {
        liftLeftPosition=liftLeft.getCurrentPosition();
        liftRightPosition=liftRight.getCurrentPosition();

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
    }

    public void setLiftVelocityFromPivotVelocity(double pivotVelocity)
    {
        double velocity = pivotVelocity / 5.0611;

        liftRight.setVelocity(velocity);
        liftLeft.setVelocity(velocity);
    }



    /////////////////////////   Lift   /////////////////////////////////////
    public class LiftElevationMaintainer implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {


                liftLeftPosition=liftLeft.getCurrentPosition();

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
            return true;

        }
    }
    public Action LiftElevationMaintainer() {  // this method is for use in RR trajectories
        return new LiftElevationMaintainer();
    }

    public Action SetLiftPosition(int liftSetPosition) {  // this method is for use in RR trajectories
        liftLeftActionPosition = liftSetPosition;
        liftRightPosition = liftSetPosition;
        return new LiftActionSet();
    }

    public class LiftActionSet implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            pControllerLiftLeft.setSetPoint(liftLeftActionPosition);
            pControllerLiftRight.setSetPoint(liftRightActionPosition);
            return false;

        }
    }
    public class LiftActionHome implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            pControllerLiftLeft.setSetPoint(0);
            pControllerLiftRight.setSetPoint(0);
            return false;

        }
    }
    public Action setLiftActionHome() {  // this method is for use in RR trajectories

        return new LiftActionHome();
    }
        public class LiftActionLow implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                pControllerLiftLeft.setSetPoint(600);
                pControllerLiftRight.setSetPoint(600);
                return false;

            }
    }
    public Action setLiftActionLow() {  // this method is for use in RR trajectories

        return new LiftActionLow();
    }
    public class LiftActionHigh implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            pControllerLiftLeft.setSetPoint(700);
            pControllerLiftRight.setSetPoint(700);
            return false;

        }
    }
    public Action setLiftActionHigh() {  // this method is for use in RR trajectories

        return new LiftActionHigh();
    }





}

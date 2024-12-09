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
    public void InitArmPivotPIDController(){

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

    public void setArmPivotPosition() {
        liftLeftPosition=liftLeft.getCurrentPosition();
        liftRightPosition=liftRight.getCurrentPosition();

    }
    public void setArmPivotPower(double armPivotPower) {
        liftLeft.setPower(armPivotPower);
        liftRight.setPower(armPivotPower);

    }

    public void updateLiftPosition() {
        liftLeftPosition=liftLeft.getCurrentPosition();
        liftRightPosition=liftRight.getCurrentPosition();

        if (liftLeftPosition < pControllerLiftLeft.setPoint) {

            armPivotLeft.setPower(minPowerArmPivotLeft +
                    pControllerArmPivotLeft.getComputedOutput(armPivotLeftPosition));
        } else {
            armPivotLeft.setPower(minPowerArmPivotLeft -
                    pControllerArmPivotLeft.getComputedOutput(armPivotLeftPosition));
        }


        if (armPivotRightPosition < pControllerArmPivotRight.setPoint) {

            armPivotRight.setPower(minPowerArmPivotRight +
                    pControllerArmPivotRight.getComputedOutput(armPivotRightPosition));
        } else {
            armPivotRight.setPower(minPowerArmPivotRight -
                    pControllerArmPivotRight.getComputedOutput(armPivotRightPosition));
        }
    }


    /////////////////////////   PixelLift   /////////////////////////////////////
    public class ArmPivotElevationMaintainer implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {


                armPivotLeftPosition=armPivotLeft.getCurrentPosition();

                if (armPivotLeftPosition < pControllerArmPivotLeft.setPoint) {

                    armPivotLeft.setPower(minPowerArmPivotLeft +
                            pControllerArmPivotLeft.getComputedOutput(armPivotLeftPosition));
                } else {
                    armPivotLeft.setPower(minPowerArmPivotLeft -
                            pControllerArmPivotLeft.getComputedOutput(armPivotLeftPosition));
                }

            if (armPivotRightPosition < pControllerArmPivotRight.setPoint) {

                armPivotRight.setPower(minPowerArmPivotRight +
                        pControllerArmPivotRight.getComputedOutput(armPivotRightPosition));
            } else {
                armPivotRight.setPower(minPowerArmPivotRight -
                        pControllerArmPivotRight.getComputedOutput(armPivotRightPosition));
            }
            return true;

        }
    }
    public Action armPivotElevationMaintainer() {  // this method is for use in RR trajectories
        return new ArmPivotElevationMaintainer();
    }

    public Action setArmPivotPosition(int pixelLiftActionSetPosition) {  // this method is for use in RR trajectories
        armPivotLeftActionPosition = pixelLiftActionSetPosition;
        armPivotRightActionPosition = pixelLiftActionSetPosition;
        return new ArmPivotActionSet();
    }

    public class ArmPivotActionSet implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            pControllerArmPivotLeft.setSetPoint(armPivotLeftActionPosition);
            pControllerArmPivotRight.setSetPoint(armPivotRightActionPosition);
            return false;

        }
    }
    public class ArmPivotActionHome implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            pControllerArmPivotLeft.setSetPoint(0);
            pControllerArmPivotRight.setSetPoint(0);
            return false;

        }
    }
    public Action setArmPivotActionHome() {  // this method is for use in RR trajectories

        return new ArmPivotActionHome();
    }
        public class ArmPivotActionLow implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                pControllerArmPivotLeft.setSetPoint(600);
                pControllerArmPivotRight.setSetPoint(600);
                return false;

            }
    }
    public Action setArmPivotActionLow() {  // this method is for use in RR trajectories

        return new ArmPivotActionLow();
    }
    public class ArmPivotActionHigh implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            pControllerArmPivotLeft.setSetPoint(700);
            pControllerArmPivotRight.setSetPoint(700);
            return false;

        }
    }
    public Action setArmPivotActionHigh() {  // this method is for use in RR trajectories

        return new ArmPivotActionHigh();
    }





}

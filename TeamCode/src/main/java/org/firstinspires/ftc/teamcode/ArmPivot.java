package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmPivot {  //this is a subsystem Class used in Auto. its based on example for RR actions.
    public DcMotorEx armPivotLeft;
    public DcMotorEx armPivotRight;
    public int armPivotLeftPosition = 0;
    public int armPivotRightPosition = 0;
    private double armPivotLeftMaxTicks = 2370;
    private double armPivotRightMaxTicks = 2370;
    // for lift pController input
    private double minPowerArmPivotLeft = .1;
    private double minPowerArmPivotRight = .1;
    private double maxPowerArmPivotLeft = 0.98;
    private double maxPowerArmPivotRight = 0.98;
    private double armPivotLeftActionPosition;
    private double armPivotRightActionPosition;

    // variable for RR1.0 actions
    //private double pixelLiftPosition; // variable for regular use like on init.. humm may not need since its defined in the method?
    private boolean exitArmPivotLeftPControllerLoop = false;
    private boolean exitArmPivotRightPControllerLoop = false;
    public PController pControllerArmPivotLeft = new PController(0.002);
    public PController pControllerArmPivotRight = new PController(0.002);

    public ArmPivot(HardwareMap hardwareMap) {
        armPivotLeft = hardwareMap.get(DcMotorEx.class, "pivotLeft");
        armPivotLeft.setDirection(DcMotor.Direction.REVERSE);
        armPivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armPivotLeft.setPower(0);
        //pixelLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // to reset at initiation
        //pixelLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPivotLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armPivotRight = hardwareMap.get(DcMotorEx.class, "pivotRight");
        armPivotRight.setDirection(DcMotor.Direction.FORWARD);
        armPivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armPivotRight.setPower(0);
        armPivotRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void InitArmPivotPIDController(){

        pControllerArmPivotLeft.setInputRange(0, armPivotLeftMaxTicks);
        pControllerArmPivotLeft.setSetPoint(0);
        pControllerArmPivotLeft.setOutputRange(minPowerArmPivotLeft, maxPowerArmPivotLeft);
        pControllerArmPivotLeft.setThresholdValue(5);

        pControllerArmPivotRight.setInputRange(0, armPivotRightMaxTicks);
        pControllerArmPivotRight.setSetPoint(0);
        pControllerArmPivotRight.setOutputRange(minPowerArmPivotRight, maxPowerArmPivotRight);
        pControllerArmPivotRight.setThresholdValue(5);
        // this is for lift left, change Kp to calibrate
    }

    public void updateLiftPosition() {
        armPivotLeftPosition=armPivotLeft.getCurrentPosition();

        if (armPivotLeftPosition < pControllerArmPivotLeft.setPoint) {

            armPivotLeft.setPower(minPowerArmPivotLeft +
                    pControllerArmPivotLeft.getComputedOutput(armPivotLeftPosition));
        } else {
            armPivotLeft.setPower(minPowerArmPivotLeft -
                    pControllerArmPivotLeft.getComputedOutput(armPivotLeftPosition));
        }
        armPivotRightPosition=armPivotRight.getCurrentPosition();

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

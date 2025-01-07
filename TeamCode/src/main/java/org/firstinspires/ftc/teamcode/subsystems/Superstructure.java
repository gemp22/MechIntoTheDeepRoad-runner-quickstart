package org.firstinspires.ftc.teamcode.subsystems;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

public class Superstructure {
    private ArmPivot armPivot;
    private Lift lift;
    private Robot robot;
    public double liftWantedHeight = 0; // inches

    public enum SuperstructureStates {
        RESTING,
        PICKUP,
        DELIVERY_LEVEL_1,
        DELIVERY_LEVEL_2,

        HANG_BAR_1_PREP,
        HANG_BAR_1,
        HANG_BAR_2_PREP,
        HANG_BAR_2
    }
    private int currentState = SuperstructureStates.RESTING.ordinal();
    private long stateStartTime = 0;
    private boolean stateFinished = true;

    private void initializeStateVariables() {
        stateStartTime = SystemClock.uptimeMillis();
        stateFinished = false;
    }

    public void nextState() {
        stateFinished = true;
        currentState++;
    }

    public void nextState(int state) {
        stateFinished = true;
        currentState = state;
    }

    public Superstructure(ArmPivot armPivot, Lift lift, Robot robot) {
        this.armPivot = armPivot;
        this.lift = lift;
        this.robot = robot;
    }

    public void update(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        long currentTimeMillis = SystemClock.uptimeMillis();

        if (currentState == SuperstructureStates.RESTING.ordinal()) {
            if (stateFinished) {
                // init vars as needed
                initializeStateVariables();
            }

            if (lift.getLiftExtension() < 8 && currentTimeMillis-stateStartTime > 1000) {
                armPivot.twist.setPosition(0.005);
                armPivot.update(0, 0.15, 45, 0.05, telemetry);
            }
            if (currentTimeMillis-stateStartTime > 1000) {
                lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();
            }

            if (armPivot.getArmAngle() < 45) {
                armPivot.setIntakeTiltAngle(60);
            } else {
                armPivot.setIntakeTiltAngle(0);
            }
        }

        if(currentState == SuperstructureStates.PICKUP.ordinal()){
            if (stateFinished) {
                // init vars as needed
                initializeStateVariables();
            }

            if(!robot.isAuto) {
                lift.setLiftPower(-gamepad2.right_stick_y);
            } else {
                lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();
            }

            if (lift.getLiftExtension()>6){
                armPivot.setIntakeTiltAngle(armPivot.intakeTiltNoArmPower(lift.getLiftExtension()));
            }
        }

        if (currentState == SuperstructureStates.DELIVERY_LEVEL_1.ordinal()){
            if (stateFinished){
                initializeStateVariables();
            }

            if (armPivot.getArmAngle() > 80){
                armPivot.twist.setPosition(0.772);
                armPivot.intakeTilt.setPosition(.25);
                lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();

                if (!armPivot.getPivotLimitState()) {
                    armPivot.setArmPivotPower(0.25);
                } else {
                    armPivot.setArmPivotPower(0);
                }
            } else {
                armPivot.setIntakeTiltAngle(0);
                armPivot.update(90, 0.75, 30,0.2, telemetry);
            }
        }

        if (currentState == SuperstructureStates.DELIVERY_LEVEL_2.ordinal()){
            if (stateFinished){
                initializeStateVariables();
            }

            if (armPivot.getArmAngle() > 80){
                armPivot.twist.setPosition(0.772);
                armPivot.intakeTilt.setPosition(.25);
                lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();

                if (!armPivot.getPivotLimitState()) {
                    armPivot.setArmPivotPower(0.25);
                } else {
                    armPivot.setArmPivotPower(0);
                }
            } else {
                armPivot.setIntakeTiltAngle(0);
                armPivot.update(90, 0.75, 30,0.2, telemetry);
            }
        }
    }
}

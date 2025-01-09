package org.firstinspires.ftc.teamcode.subsystems;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
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
        System.out.println("Changing State: " + state);
        System.out.println("Delivery Level Ordinal: " + Superstructure.SuperstructureStates.DELIVERY_LEVEL_1.ordinal());
    }

    public Superstructure(ArmPivot armPivot, Lift lift, Robot robot) {
        this.armPivot = armPivot;
        this.lift = lift;
        this.robot = robot;
    }

    public void update(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        long currentTimeMillis = SystemClock.uptimeMillis();

        telemetry.addData("Superstructure State", currentState);
        System.out.println("Superstructure State: " + currentState);
        System.out.println("Tilt Pos: " + armPivot.intakeTilt.getPosition());

        if (currentState == SuperstructureStates.RESTING.ordinal()) {
            if (stateFinished) {
                // init vars as needed
                initializeStateVariables();
            }

            if (lift.getLiftExtension() < 8) {
                armPivot.twist.setPosition(Constants.TWIST_SERVO_HORIZONTAL_POSITION);
                armPivot.update(0, 0.5, 45, 0.05, telemetry);
            }
            lift.setSetPoint(liftWantedHeight);
            lift.updateLiftPosition();

            if (armPivot.getArmAngle() < 45) {
                System.out.println("Setting servo 60 degrees");
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
                armPivot.twist.setPosition(Constants.TWIST_SERVO_BASKET_DEPOSIT_POSITION);
                armPivot.intakeTilt.setPosition(Constants.TILT_SERVO_PARALLEL_WITH_FLOOR);
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
                armPivot.twist.setPosition(Constants.TWIST_SERVO_BASKET_DEPOSIT_POSITION);
                armPivot.intakeTilt.setPosition(Constants.TILT_SERVO_PARALLEL_WITH_FLOOR);
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

        if (currentState == SuperstructureStates.HANG_BAR_1_PREP.ordinal()){
            if (stateFinished){
                initializeStateVariables();
            }

            lift.setSetPoint(liftWantedHeight);
            lift.updateLiftPosition();
            armPivot.twist.setPosition(Constants.TWIST_SERVO_HORIZONTAL_POSITION);
            armPivot.setIntakeTiltAngle(Constants.TILT_SERVO_90_DEGREES_UP);
            armPivot.update(55, 0.75, 30,0.2, telemetry);
        }

        if (currentState == SuperstructureStates.HANG_BAR_1.ordinal()){
            if (stateFinished){
                initializeStateVariables();
            }

            if (armPivot.getArmAngle() < 45) {
                lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();
            }
            armPivot.twist.setPosition(Constants.TWIST_SERVO_HORIZONTAL_POSITION);
            armPivot.setIntakeTiltAngle(Constants.TILT_SERVO_90_DEGREES_UP);
            armPivot.update(-5, 1, 5,0.25, telemetry);
        }

        if (currentState == SuperstructureStates.HANG_BAR_2_PREP.ordinal()) {
            if (stateFinished) {
                initializeStateVariables();
            }
        }
    }
}

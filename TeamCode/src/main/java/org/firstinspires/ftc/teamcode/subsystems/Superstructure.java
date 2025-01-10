package org.firstinspires.ftc.teamcode.subsystems;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;

public class Superstructure {
    private ArmPivot armPivot;
    private Lift lift;
    private Robot robot;
    public double liftWantedHeight = 0;// inches
    public double armWantedAngle = 0; // inches
    boolean liftIsReadyForBar2Pivot = false;
    boolean liftSwitchPressedOnce = false;
    double holdEveryThingLiftPose = 0;
    double holdEveryThingLiftAngle=0;

    private long lastHangRestTime = 0;

    public enum SuperstructureStates {
        //basket delivery////
        RESTING,
        PICKUP,
        DELIVERY_LEVEL_1,
        DELIVERY_LEVEL_2,
      // hang automation
        HANG_BAR_1_PREP,
        HANG_BAR_1,
        HANG_BAR_2_PREP,
        HANG_BAR_2,
        HANG_BAR_2_FINISH,
        ROBOT_RESTING_ON_BAR_2,
        HOLD_EVERYTHING,

        //specimen State

        COLLECT_SPECIMEN_PREP,
        COLLECT_SPECIMEN_WALL,
        DELIVER_SPECIMEN_PREP,
        DELIVER_SPECIMEN


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

                    if(SystemClock.uptimeMillis()-stateStartTime>550){
                    armPivot.update(0, 0.5, 45, 0.05, telemetry);
                    }
            }
            lift.setSetPoint(liftWantedHeight);
            lift.updateLiftPosition();

            if (armPivot.getArmAngle() < 0) {
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

        ////// Hang States /////////

        if (currentState == SuperstructureStates.HANG_BAR_1_PREP.ordinal()){
            if (stateFinished){
                initializeStateVariables();
            }
            System.out.println("HANG_BAR_1PREP DEBUG Arm Angle: " + armPivot.getArmAngle());

            lift.setSetPoint(liftWantedHeight);
            lift.updateLiftPosition();
            armPivot.twist.setPosition(Constants.TWIST_SERVO_HORIZONTAL_POSITION);
            armPivot.setIntakeTiltAngle(90);
            armPivot.update(55, 0.75, 30,0.2, telemetry);
        }

        if (currentState == SuperstructureStates.HANG_BAR_1.ordinal()){
            if (stateFinished){
                initializeStateVariables();
            }

            System.out.println("HANG DEBUG BAR 1 Limit Switch State: " + armPivot.getLiftLimitState());
            System.out.println("HANG DEBUG BAR 1 Lift In State: " + (lift.getLiftExtension() < 5));
            System.out.println("HANG DEBUG BAR 1 Lift Height: " + lift.getLiftExtension());
            System.out.println("HANG DEBUG BAR 1 Arm Angle: " + armPivot.getArmAngle());

            if (lift.getLiftExtension() < 2.25 && armPivot.getArmAngle() < 5) {
                lift.setLiftPower(0);
                armPivot.setArmPivotPower(0);
                nextState(SuperstructureStates.HANG_BAR_2_PREP.ordinal());
            } else {
                if (lift.getLiftExtension() > 2) {
                    lift.setLiftPower(-1);
                } else {
                    lift.setLiftPower(-0.5);
                }
//                lift.setSetPoint(liftWantedHeight);
//                lift.updateLiftPosition();
                armPivot.twist.setPosition(Constants.TWIST_SERVO_HORIZONTAL_POSITION);
                armPivot.setIntakeTiltAngle(90);

                if (armPivot.getArmAngle() > 0) {
                    armPivot.setArmPivotPower(-1);
                } else {
                    armPivot.setArmPivotPower(-0.5);
                }

                //armPivot.update(-5, 1, 5,1, telemetry);
            }
        }

        if (currentState == SuperstructureStates.HANG_BAR_2_PREP.ordinal()) {
            if (stateFinished) {
                initializeStateVariables();
            }


            armPivot.twist.setPosition(Constants.TWIST_SERVO_HORIZONTAL_POSITION);
            armPivot.setIntakeTiltAngle(90);

            System.out.println("HANG DEBUG BAR 2 PREP Limit Switch State: " + armPivot.getLiftLimitState());
            System.out.println("HANG DEBUG BAR 2 PREP Lift In State: " + (lift.getLiftExtension() < 5));
            System.out.println("HANG DEBUG BAR 2 PREP Lift Height: " + lift.getLiftExtension());
            System.out.println("HANG DEBUG BAR 2 PREP Arm Angle: " + armPivot.getArmAngle());

            if(SystemClock.uptimeMillis()-stateStartTime>750 && lift.getLiftExtension() > 7){
                liftIsReadyForBar2Pivot = true;
            }
            if(liftIsReadyForBar2Pivot ){
                if(armPivot.getArmAngle()>18){
                    lift.setSetPoint(21.5);
                    lift.updateLiftPosition();
                    if (lift.getLiftExtension() > 18) {
                        armPivot.update(17, 0.65, 10, 0.3, telemetry);
                    } else {
                        armPivot.update(23, 0.650, 10, 0.3, telemetry);
                    }
                    //nextState(SuperstructureStates.HANG_BAR_2.ordinal());
                } else {
                    if (SystemClock.uptimeMillis()-stateStartTime>1000) {
                        lift.setSetPoint(8);
                        lift.updateLiftPosition();
                        armPivot.update(23, 0.650, 10, 0.3, telemetry);
                    }
                }
            } else {
                if (SystemClock.uptimeMillis()-stateStartTime>100) {
                    lift.setSetPoint(8);
                    lift.updateLiftPosition();
                }
            }
        }
        if (currentState == SuperstructureStates.HANG_BAR_2.ordinal()) {
            if (stateFinished) {
                liftSwitchPressedOnce = false;
                initializeStateVariables();
            }

            if(!armPivot.getLiftLimitState() && !liftSwitchPressedOnce){
                lift.setLiftPower(-1);
                armPivot.setArmPivotPower(0);
            }

            if(armPivot.getLiftLimitState()){
                liftSwitchPressedOnce = true;
            }

            if (liftSwitchPressedOnce) {
                lift.setSetPoint(0);
                lift.updateLiftPosition();
                armPivot.update(20, 1, 10, 0.3, telemetry);
                if(armPivot.getArmAngle() < 40){
                    nextState(SuperstructureStates.HANG_BAR_2_FINISH.ordinal());
                }
            }
        }

        if (currentState == SuperstructureStates.HANG_BAR_2_FINISH.ordinal()) {
            if (stateFinished) {
                lastHangRestTime = SystemClock.uptimeMillis();
                initializeStateVariables();
            }

            System.out.println("HANG DEBUG BAR 2 FINISH Limit Switch State: " + armPivot.getLiftLimitState());
            System.out.println("HANG DEBUG BAR 2 FINISH Lift In State: " + (lift.getLiftExtension() < 5));
            System.out.println("HANG DEBUG BAR 2 FINISH Lift Height: " + lift.getLiftExtension());
            System.out.println("HANG DEBUG BAR 2 FINISH Arm Angle: " + armPivot.getArmAngle());
            System.out.println("HANG DEBUG BAR 2 FINISH Lift Power: " + lift.liftLeft.getPower());

            if (SystemClock.uptimeMillis()-stateStartTime > 3500) {
                lift.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                lift.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armPivot.update(-5, 1, 10, 0.5, telemetry);
                if (armPivot.getArmAngle() < 1) {
                    if (lift.getLiftExtension() < 3 && armPivot.getArmAngle() < 1) {
                        nextState(SuperstructureStates.ROBOT_RESTING_ON_BAR_2.ordinal());
                    } else {
                        if (lift.getLiftExtension() > 2) {
                            lift.setLiftPower(-1);
                        } else {
                            lift.setLiftPower(-0.5);
                        }
                    }
                }
            } else {
                if (SystemClock.uptimeMillis()-lastHangRestTime >= 10) {
                    if (((SystemClock.uptimeMillis() / 10) % 2) == 0) {
                        lift.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        lift.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    } else {
                        lift.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        lift.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    }

                    lastHangRestTime = SystemClock.uptimeMillis();
                }
                armPivot.update(45, 1, 10, 0.3, telemetry);
                lift.setLiftPower(0);
            }
        }

        if (currentState == SuperstructureStates.ROBOT_RESTING_ON_BAR_2.ordinal()) {
            if (stateFinished) {
                initializeStateVariables();
            }
            armPivot.setArmPivotPower(0);
            lift.setLiftPower(0);
        }

        if (currentState == SuperstructureStates.HOLD_EVERYTHING.ordinal()) {
            if (stateFinished) {
                holdEveryThingLiftAngle = armPivot.getArmAngle();
                holdEveryThingLiftPose = lift.getLiftExtension();
                initializeStateVariables();
            }
            lift.setSetPoint(holdEveryThingLiftPose);
            lift.updateLiftPosition();
            armPivot.update(holdEveryThingLiftAngle, 1, 10, 0.3, telemetry);

        }
        ///// specimen states/////

        if (currentState == SuperstructureStates.COLLECT_SPECIMEN_PREP.ordinal()){
            if (stateFinished){
                initializeStateVariables();
            }

            System.out.println("SPECIMEN DEBUG Lift In State: " + (lift.getLiftExtension() < 5));
            System.out.println("SPECIMEN DEBUG Lift Height: " + lift.getLiftExtension());
            System.out.println("SPECIMEN DEBUG Lift Arm Angle: " + armPivot.getArmAngle());
            System.out.println("SPECIMEN DEBUG Jaw pos: " + armPivot.intakeJawServo.getPosition());


            if (armPivot.getArmAngle() > 3){
                System.out.println("SPECIMEN Entered If Statement in State 11");
                armPivot.twist.setPosition(Constants.TWIST_SERVO_WALL_COLLECTION_POSITION);
                armPivot.intakeTilt.setPosition(Constants.TILT_SERVO_PARALLEL_WITH_FLOOR);
                armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_WALL_COLLECTION);
                armPivot.vexIntake.setPower(-.5);
                lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();

                if (!armPivot.getPivotLimitState()) {
                    armPivot.setArmPivotPower(0.25);
                } else {
                    armPivot.setArmPivotPower(0);
                }
            } else {
                armPivot.setIntakeTiltAngle(0);
                armPivot.update(9, 0.8, 2,0.6, telemetry);
            }
        }

        if (currentState == SuperstructureStates.COLLECT_SPECIMEN_WALL.ordinal()){
            if (stateFinished){
                initializeStateVariables();
            }

            System.out.println("SPECIMEN WALL GRAB DEBUG Lift In State: " + (lift.getLiftExtension() < 5));
            System.out.println("SPECIMEN WALL GRAB DEBUG Lift Height: " + lift.getLiftExtension());
            System.out.println("SPECIMEN WALL GRAB DEBUG Lift Arm Angle: " + armPivot.getArmAngle());

            armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_GRAB_POSITION);
            armPivot.vexIntake.setPower(0);

            if (SystemClock.uptimeMillis()-stateStartTime>550){
                System.out.println("SPECIMEN Grab Entered If Statement in State 12");
                lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();

            }
        }




    }
}

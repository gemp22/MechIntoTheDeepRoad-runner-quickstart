package org.firstinspires.ftc.teamcode.subsystems;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoSpecimens;
import org.firstinspires.ftc.teamcode.ButtonPress;
import org.firstinspires.ftc.teamcode.Constants3Motor;
import org.firstinspires.ftc.teamcode.Robot3Motor;

public class Superstructure3Motor {
    private ArmPivot3Motor armPivot;
    private Lift3Motor lift;
    private Robot3Motor robot;
    public double liftWantedHeight = 0;// inches
    public double tiltWantedAngle = 0;// inches
    public double armWantedAngle = 0; // inches
    boolean liftIsReadyForBar2Pivot = false;
    boolean liftIsReadyToAttachToBar2 =false;
    boolean liftSwitchPressedOnce = false;
    boolean armPivotReadytoGrabOffTheWall = false;
    boolean servosGrabOffTheWall = false;
    boolean setToHomeResting = false;
    boolean collectionMode = false;
    boolean taskReStartTime = false;
    boolean taskReStartTime2 = false;
    boolean outTakeMode = false;
    boolean outTakePreValue = false;
    boolean deliveryTwist = false;
    boolean deliveryTIlt = false;
    boolean isLiftOrPivotSmall = false;
    public boolean sampleCollected = false;
    public boolean intakeUnJam = false;
    boolean intakeUnJamTwist = false;
    boolean intakeUnJamLimitSwitchPressed=false;
    double holdEveryThingLiftPose = 0;
    double holdEveryThingLiftAngle = 0;
    double targetPivotAngle;
    double liftStateStartExtension=0;

    double pivotStateStartAngle = 0;
    int armLeftPivotTicks;
    int armRightPivotTicks;
    private long lastHangRestTime = 0;
    boolean isManualControlActive = false;
    public static boolean disableTelopGamepad1A = false;
    private boolean okToExtendLiftToBar2 = false;
    private double hangPivotAngleToBarTwoPrep = 0.0;

    private double hangPivotAngleBarTwoAttachAdjustment = 0;

    double pivotAngleWhenAtHardstop = 0;
    boolean pivotAtHardstop = false;
    boolean liftExtenedToTipBotForward = false;

    private int hangCounter = 0;
    public enum SuperstructureStates {
        //basket delivery////
        RESTING,
        PICKUP,
        DELIVERY_LEVEL_1,
        DELIVERY_LEVEL_2,
        DELIVERY_SAMPLE_DROP,

        // hang automation
        HANG_BAR_1_PREP,
        HANG_BAR_1,
        HANG_BAR_1_RESTING,
        HANG_BAR_2_PREP,
        HANG_BAR_TWO_PREP_RESTING,
        HANG_BAR_2,
        HANG_BAR_2_DONE_DONE_DONE,
        HANG_BAR_2_FINISH,
        ROBOT_RESTING_ON_BAR_2,
        HOLD_EVERYTHING,

        //specimen States
        COLLECT_SPECIMEN_PREP,
        COLLECT_SPECIMEN_WALL,
        SPECIMEN_TRANSPORT,
        SPECIMEN_HANG_PREP,
        SPECIMEN_HANG_CHAMBER,
        SPECIMEN_HANG_CHAMBER_TELE,
        SPECIMEN_HANG_FRONT_PREP,
        SPECIMEN_HANG_FRONT_CHAMBER,

        //SAMPLE COLLECTION //////
        SAMPLE_COLLECTION_EXTENSTION,
        SAMPLE_COLLECTION_INTAKE,
        UNJAM_INTAKE,

        //manual control

        //auto front hang

        SPECIMEN_HANG_FRONT_PREP_AUTO,
        SPECIMEN_HANG_FRONT_CHAMBER_AUTO,
        AUTO_PARK,


        TELEOP_START,
        RESET_PIVOT_ANGLE

    }

    private boolean hangPastValidation = false;
    private int currentState = SuperstructureStates.RESTING.ordinal();
    private long stateStartTime = 0;
    private long taskStartTime = 0;
    private long taskStartTime2 = 0;
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
        System.out.println("Delivery Level Ordinal: " + Superstructure3Motor.SuperstructureStates.DELIVERY_LEVEL_1.ordinal());
    }

    public Superstructure3Motor(ArmPivot3Motor armPivot, Lift3Motor lift, Robot3Motor robot) {
        this.armPivot = armPivot;
        this.lift = lift;
        this.robot = robot;
    }
    private double restingStateStartingAngle = 0;
    private double lastPitchAngle = 0;
    private long lastUpdateTime = 0;

    public void update(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        long currentTimeMillis = SystemClock.uptimeMillis();

        telemetry.addData("Superstructure State", currentState);
        telemetry.addData("is state finished? ", stateFinished);
        System.out.println("Superstructure State: " + currentState);
        System.out.println("Tilt Pos: " + armPivot.intakeTilt.getPosition());
        System.out.println("lift extension" + lift.getLiftExtension());
        System.out.println("pivot angle" + armPivot.getArmAngle());


        if (currentState == SuperstructureStates.RESTING.ordinal()) {
            if (stateFinished) {
                restingStateStartingAngle = armPivot.getArmAngle();
                // init vars as needed
                setToHomeResting = false;
                collectionMode = false;
                outTakeMode = false;
                taskReStartTime = false;
                taskReStartTime2 = false;
                isLiftOrPivotSmall = false;
//                liftStateStartExtension = lift.getLiftExtension();
//                pivotStateStartAngle = armPivot.getArmAngle();
                armPivot.vexIntake.setPower(0);
                lift.setSetPoint(0);
                liftWantedHeight = 0;
                armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_GRAB_POSITION);
                armPivot.intakeTilt.setPosition(Constants3Motor.TILT_SERVO_PARALLEL_WITH_PIVOT);
                if(lift.getLiftExtension()<6 || armPivot.getArmAngle()<60){
                    isLiftOrPivotSmall = true;
                }
                initializeStateVariables();
            }

            if(isLiftOrPivotSmall){
                System.out.println("lift/pivot is small");

                if(!taskReStartTime){
                taskStartTime = SystemClock.uptimeMillis();
                taskReStartTime = true;
                }
                System.out.println("arm safe timer" + (SystemClock.uptimeMillis() - taskStartTime));
                if(SystemClock.uptimeMillis() - taskStartTime > 750){ //wait for jaw and tilt
                    armPivot.twist.setPosition(Constants3Motor.TWIST_SERVO_HORIZONTAL_POSITION);
                    if (lift.getLiftExtension()<3 && armPivot.getArmAngle() > -3 && (SystemClock.uptimeMillis() - taskStartTime) > 1500) { //gives time for twist before Pivot goes down
                        armPivot.setIntakeTiltAngle(90);
                        System.out.println("in bad if statement at " + (SystemClock.uptimeMillis() - taskStartTime));
                        armPivot.update(-3, 0.5, 20, 0.05, telemetry);
                    }else if(armPivot.getArmAngle() < -3) {
                        //armPivot.setIntakeTiltAngle(90);
                        armPivot.armPivot.setPower(0);
                    }
                }
            }else if (lift.getLiftExtension() < 12 && !isLiftOrPivotSmall) { //this gives time for jaw and tilt to get right before twist and pivot for long lift extensions
                armPivot.twist.setPosition(Constants3Motor.TWIST_SERVO_HORIZONTAL_POSITION);
                System.out.println("lift extension is less than 12 ");
                if (lift.getLiftExtension() < 3 && armPivot.getArmAngle() > -5) {
                    armPivot.setIntakeTiltAngle(90);
                    armPivot.update(-3, 0.5, 20, 0.05, telemetry);
                }else {
                    armPivot.armPivot.setPower(0);

                }
            }else{
                System.out.println("Angle we should be setting to: " + restingStateStartingAngle);
                armPivot.update(restingStateStartingAngle,.75,20,0.10, telemetry);
            }

            if ((gamepad2.right_stick_y) < -0.01  // this is out
                    && lift.getLiftExtension() < Constants3Motor.LIFT_MAX_HORIZONTAL_POSITION_IN) {
                lift.setLiftPower(-gamepad2.right_stick_y);
                liftWantedHeight = lift.getLiftExtension();
                //liftWantedHeight = lift.getLiftExtension();

            } else if (gamepad2.right_stick_y > 0.01 && lift.getLiftExtension() > 0) { //this is in
                lift.setLiftPower(-gamepad2.right_stick_y);
                liftWantedHeight = lift.getLiftExtension();
                //liftWantedHeight = lift.getLiftExtension();

            } else {
                lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();
            }

            if (ButtonPress.isGamepad2_left_stick_button_pressed() && lift.getLiftExtension() > 3.1 ) {
                if (!collectionMode) {
                    armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_INTAKE_POSITION);
                    armPivot.vexIntake.setPower(-.91);
                    collectionMode = true;
                } else {
                    armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_GRAB_POSITION);
                    armPivot.vexIntake.setPower(0);
                    armPivot.setIntakeTiltAngle(45);
                    collectionMode = false;
                }
            }

            if (collectionMode) {
                armPivot.setIntakeTiltAngle(armPivot.intakeTiltNoArmPower(lift.getLiftExtension()));
            }

            if (lift.getLiftExtension() < 3.1 && !outTakeMode && armPivot.getArmAngle()<10) {
                collectionMode = false;
                armPivot.setIntakeTiltAngle(90);
            }

            if (gamepad2.guide && !outTakePreValue) {
                if (!outTakeMode) {
                    armPivot.setIntakeTiltAngle(Constants3Motor.TILT_INTAKE_ANGLE_OUTTAKE);
                    armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_WALL_COLLECTION);
                    armPivot.vexIntake.setPower(.5);
                    outTakeMode = true;
                    collectionMode=false;
                } else {
                    armPivot.vexIntake.setPower(0);
                    outTakeMode = false;
                    collectionMode=false;
                }
            }

            outTakePreValue = gamepad2.guide;
        }

        if (currentState == SuperstructureStates.PICKUP.ordinal()) {
            if (stateFinished) {
                // init vars as needed
                initializeStateVariables();
            }
            if (!robot.isAuto) {
                lift.setLiftPower(-gamepad2.right_stick_y);
            } else {
                lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();
            }
            if (lift.getLiftExtension() > 6) {
                armPivot.setIntakeTiltAngle(armPivot.intakeTiltNoArmPower(lift.getLiftExtension()));
            }
        }

        if (currentState == SuperstructureStates.DELIVERY_LEVEL_1.ordinal()) {
            if (stateFinished) {
                deliveryTIlt = false;
                deliveryTwist = false;
                liftWantedHeight = 10;
                initializeStateVariables();
            }


            if (armPivot.getArmAngle() > 73) {
                armPivot.twist.setPosition(Constants3Motor.TWIST_SERVO_BASKET_DEPOSIT_POSITION);
                armPivot.intakeTilt.setPosition(Constants3Motor.TILT_SERVO_PARALLEL_WITH_FLOOR);
                lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();

                if (!armPivot.getPivotLimitState()) {
                    armPivot.setArmPivotPower(0.25);
                } else {
                    armPivot.setArmPivotPower(0);
                }
            }

            armPivot.setIntakeTiltAngle(0);
            armPivot.update2(90, 0.99, 15, 0.6, telemetry);
            armPivot.setArmPivotSetPointTicks(armPivot.armPivot.getCurrentPosition());


//            if (armPivot.getArmAngle() > 80) {
//                armPivot.twist.setPosition(Constants.TWIST_SERVO_BASKET_DEPOSIT_POSITION);
//                armPivot.intakeTilt.setPosition(Constants.TILT_SERVO_BASKET_DELIVERY);
//                lift.setSetPoint(liftWantedHeight);
//                lift.updateLiftPosition();
//
//                if (!armPivot.getPivotLimitState()) {
//                    armPivot.setArmPivotPower(0.25);
//                } else {
//                    armPivot.setArmPivotPower(0);
//                }
//            }
//
//            armPivot.setIntakeTiltAngle(0);
//            armPivot.update2(94, 0.75, 30, 0.25, telemetry);
//
//            if (armPivot.getArmAngle() > 5 && !deliveryTIlt) {
//                armPivot.setIntakeTiltAngle(0);
//                deliveryTIlt = true;
//            }
//            if (armPivot.getArmAngle() > 80 && !deliveryTwist) {
//                armPivot.twist.setPosition(Constants.TWIST_SERVO_BASKET_DEPOSIT_POSITION);
//                deliveryTwist = true;
//            }
        }

        if (currentState == SuperstructureStates.DELIVERY_LEVEL_2.ordinal()) {
            if (stateFinished) {
                liftWantedHeight = 25;
                initializeStateVariables();
            }

            if (armPivot.getArmAngle() > 73) {
                armPivot.twist.setPosition(Constants3Motor.TWIST_SERVO_BASKET_DEPOSIT_POSITION);
                armPivot.intakeTilt.setPosition(Constants3Motor.TILT_SERVO_PARALLEL_WITH_FLOOR);
                lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();

                if (!armPivot.getPivotLimitState()) {
                    armPivot.setArmPivotPower(0.25);
                } else {
                    armPivot.setArmPivotPower(0);
                }
            }

                armPivot.setIntakeTiltAngle(0);
                armPivot.update2(90, 0.99, 15, 0.6, telemetry);
                armPivot.setArmPivotSetPointTicks(armPivot.armPivot.getCurrentPosition());


        }

        if (currentState == SuperstructureStates.DELIVERY_SAMPLE_DROP.ordinal()) {
            if (stateFinished) {
                armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_DROP_POSITION-0.15);
                armPivot.vexIntake.setPower(.5);

                armPivot.setIntakeTiltAngle(-64);
                restingStateStartingAngle = armPivot.getArmAngle();
                initializeStateVariables();
            }

            if (SystemClock.uptimeMillis()-stateStartTime > 100) {
            }

            armPivot.update(restingStateStartingAngle,.75,15,0.15, telemetry);

            if (gamepad1.right_bumper) {
                armPivot.vexIntake.setPower(0);
                armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_GRAB_POSITION);
                armPivot.setIntakeTiltAngle(Constants3Motor.TILT_SERVO_PARALLEL_WITH_PIVOT);
            }


        }


        ////// Hang States /////////

        if (currentState == SuperstructureStates.HANG_BAR_1_PREP.ordinal()) {
            if (stateFinished) {
                liftWantedHeight = 16;
                initializeStateVariables();
            }
            System.out.println("HANG_BAR_1PREP DEBUG Arm Angle: " + armPivot.getArmAngle());
            System.out.println("HANG_BAR_1PREP DEBUG lift height: " + lift.getLiftExtension());

            lift.setSetPoint(liftWantedHeight);
            lift.updateLiftPosition();
            armPivot.twist.setPosition(Constants3Motor.TWIST_SERVO_HORIZONTAL_POSITION);
            armPivot.setIntakeTiltAngle(90);
            armPivot.update(53, 0.8, 10, 0.45, telemetry);
        }

        if (currentState == SuperstructureStates.HANG_BAR_1.ordinal()) {
            if (stateFinished) {
                initializeStateVariables();
            }

            System.out.println("HANG DEBUG BAR 1 Limit Switch State: " + armPivot.getLiftLimitState());
            System.out.println("HANG DEBUG BAR 1 Lift In State: " + (lift.getLiftExtension() < 5));
            System.out.println("HANG DEBUG BAR 1 Lift Height: " + lift.getLiftExtension());
            System.out.println("HANG DEBUG BAR 1 Arm Angle: " + armPivot.getArmAngle());
            System.out.println("HANG DEBUG BAR 1 Arm power: " + armPivot.armPivot.getPower());
            System.out.println("HANG DEBUG BAR 1 Lift Power: " + lift.liftLeft.getPower());

            if (lift.getLiftExtension() < 2.4 && armPivot.getArmAngle() < 0) {
                lift.setLiftPower(0);
                armPivot.setArmPivotPower(0);
                nextState(SuperstructureStates.HANG_BAR_1_RESTING.ordinal());
            } else {
                if (lift.getLiftExtension() > 2 && armPivot.getArmAngle()<-5) {
                    lift.setLiftPower(-1);
                }
                else {
                    lift.setLiftPower(-0.25);
                }
//                lift.setSetPoint(liftWantedHeight);
//                lift.updateLiftPosition();
                armPivot.twist.setPosition(Constants3Motor.TWIST_SERVO_HORIZONTAL_POSITION);
                armPivot.setIntakeTiltAngle(90);

                if (armPivot.getArmAngle() > -9) {
                    armPivot.setArmPivotPower(-1);
                } else {
                    armPivot.setArmPivotPower(-0.5);
                }

                //armPivot.update(-5, 1, 5,1, telemetry);
            }
        }
        if (currentState == SuperstructureStates.HANG_BAR_1_RESTING.ordinal()) {
            if (stateFinished) {
                initializeStateVariables();
            }
        }

        if (currentState == SuperstructureStates.HANG_BAR_2_PREP.ordinal()) {
            if (stateFinished) {
                //lastPitchAngle =  robot.drive.lazyImu.get().getRobotYawPitchRollAngles().getPitch();
                liftIsReadyToAttachToBar2 = false;
                liftIsReadyForBar2Pivot = false;
                okToExtendLiftToBar2 = false;
                hangPivotAngleToBarTwoPrep = 18.0;
                disableTelopGamepad1A = true;
                initializeStateVariables();
            }

            armPivot.twist.setPosition(Constants3Motor.TWIST_SERVO_HORIZONTAL_POSITION);
            armPivot.setIntakeTiltAngle(90);

            System.out.println("HANG DEBUG BAR 2 PREP Limit Switch State: " + armPivot.getLiftLimitState());
            System.out.println("HANG DEBUG BAR 2 PREP Lift In State: " + (lift.getLiftExtension() < 5));
            System.out.println("HANG DEBUG BAR 2 PREP Lift Height: " + lift.getLiftExtension());
            System.out.println("HANG DEBUG BAR 2 PREP Arm Angle: " + armPivot.getArmAngle());
            System.out.println("HANG DEBUG BAR 2 PREP Arm power: " + armPivot.armPivot.getPower());
            System.out.println("HANG DEBUG BAR 2 PREP Lift Power: " + lift.liftLeft.getPower());


            if (SystemClock.uptimeMillis() - stateStartTime > 750 && lift.getLiftExtension() > 7 && !liftIsReadyToAttachToBar2) {
                liftIsReadyForBar2Pivot = true;
            }
            if (liftIsReadyForBar2Pivot && !liftIsReadyToAttachToBar2) {
                if(ButtonPress.isGamepad1_a_pressed())
                {
                 okToExtendLiftToBar2 = true;
                }
                if(ButtonPress.isGamepad1_dpad_left_pressed())
                {
                    hangPivotAngleToBarTwoPrep += 1;
                }
                if(ButtonPress.isGamepad1_dpad_right_pressed())
                {
                    hangPivotAngleToBarTwoPrep -= 1;
                }
                if (armPivot.getArmAngle() > 13 && okToExtendLiftToBar2) {
                    lift.setSetPoint(21.5);
                    lift.updateLiftPosition();
                    armPivot.update(hangPivotAngleToBarTwoPrep, 0.4, 15, 0.3, telemetry); // target angle was 22

                    if (lift.getLiftExtension() > 20) {
                        liftIsReadyToAttachToBar2 = true;
                    }
                    //nextState(SuperstructureStates.HANG_BAR_2.ordinal());
                } else {
                    lift.setSetPoint(8);
                    lift.updateLiftPosition();
                    armPivot.update(hangPivotAngleToBarTwoPrep, 0.4, 15, 0.3, telemetry); // target angle was 22
                }

            } else if (!liftIsReadyToAttachToBar2) {
                if (SystemClock.uptimeMillis() - stateStartTime > 100) {
                    lift.setSetPoint(8);
                    lift.updateLiftPosition();
                }
            }

            if (liftIsReadyToAttachToBar2) {
                lift.setSetPoint(21.5);
                lift.updateLiftPosition();
                armPivot.update(14, 0.8, 15, 0.5, telemetry);
                if(armPivot.getArmAngle()<17){
                    disableTelopGamepad1A = false;
                    nextState(SuperstructureStates.HANG_BAR_TWO_PREP_RESTING.ordinal());
                }
            }
        }
        if (currentState == SuperstructureStates.HANG_BAR_TWO_PREP_RESTING.ordinal()) {

            System.out.println("HANG DEBUG BAR 2 RESTING Limit Switch State: " + armPivot.getLiftLimitState());
            System.out.println("HANG DEBUG BAR 2 RESTING Lift In State: " + (lift.getLiftExtension() < 5));
            System.out.println("HANG DEBUG BAR 2 RESTING Lift Height: " + lift.getLiftExtension());
            System.out.println("HANG DEBUG BAR 2 RESTING Arm Angle: " + armPivot.getArmAngle());
            System.out.println("HANG DEBUG BAR 2 RESTING Arm power: " + armPivot.armPivot.getPower());
            System.out.println("HANG DEBUG BAR 2 RESTING Lift Power: " + lift.liftLeft.getPower());

            if (stateFinished) {
                hangPivotAngleBarTwoAttachAdjustment = 14;
                initializeStateVariables();
            }
            if(ButtonPress.isGamepad1_dpad_left_pressed())
            {
                hangPivotAngleBarTwoAttachAdjustment += 1;
            }
            if(ButtonPress.isGamepad1_dpad_right_pressed())
            {
                hangPivotAngleBarTwoAttachAdjustment -= 1;
            }
            armPivot.update(hangPivotAngleBarTwoAttachAdjustment, 0.5, 15, 0.3, telemetry);
            lift.setSetPoint(21.5);
            lift.updateLiftPosition();
        }

        if (currentState == SuperstructureStates.HANG_BAR_2.ordinal()) {
            if (stateFinished) {
                liftSwitchPressedOnce = false;
                pivotAtHardstop = false;
                initializeStateVariables();
            }

            System.out.println("HANG DEBUG BAR2  Limit Switch State: " + armPivot.getLiftLimitState());
            System.out.println("HANG DEBUG BAR2  Lift In State: " + (lift.getLiftExtension() < 5));
            System.out.println("HANG DEBUG BAR2  Lift Height: " + lift.getLiftExtension());
            System.out.println("HANG DEBUG BAR2  Arm Angle: " + armPivot.getArmAngle());
            System.out.println("HANG DEBUG BAR2  Arm power: " + armPivot.armPivot.getPower());
            System.out.println("HANG DEBUG BAR2  Lift Power: " + lift.liftLeft.getPower());
            System.out.println("HANG DEBUG BAR2  Pivot Angle at hardstop: " + pivotAngleWhenAtHardstop);
            System.out.println("HANG DEBUG BAR2  Pivot adjusted angle:  " + Math.abs(armPivot.getArmAngle() - pivotAngleWhenAtHardstop));


            if (!armPivot.getLiftLimitState() && !liftSwitchPressedOnce) {
                lift.setLiftPower(-1);
                armPivot.setArmPivotPower(0); // keeps pivot at hardstop while pulling up
            }

            if (armPivot.getLiftLimitState() && !liftSwitchPressedOnce) {
                liftSwitchPressedOnce = true;
            }

            if (lift.getLiftExtension()< 8 && !pivotAtHardstop) {  // this captures arm pivot angle at hardstop so we can be more accurate on last hang State
                pivotAtHardstop = true;
                pivotAngleWhenAtHardstop=armPivot.getArmAngle();
            }

            if (liftSwitchPressedOnce) {
                if (ButtonPress.isGamepad1_b_pressed()) {
                    nextState(SuperstructureStates.HANG_BAR_2_DONE_DONE_DONE.ordinal());
                }
                lift.setSetPoint(0);  // this has to be 0 or wont go in hook
                lift.updateLiftPosition();
                armPivot.setArmPivotPower(-1);

                //armPivot.update(-5, 1, 10, 0.5, telemetry);
                /*if (armPivot.getArmAngle() < 40) {
                    nextState(SuperstructureStates.HANG_BAR_2_FINISH.ordinal());
                }*/
            }
        }

        if (currentState == SuperstructureStates.HANG_BAR_2_DONE_DONE_DONE.ordinal()) {
            if (stateFinished) {
                initializeStateVariables();
                liftSwitchPressedOnce = false;
                liftExtenedToTipBotForward =false;


            }
            System.out.println("HANG DEBUG BAR 2 DONE Limit Switch State: " + armPivot.getLiftLimitState());
            System.out.println("HANG DEBUG BAR 2 DONE Lift In State: " + (lift.getLiftExtension() < 5));
            System.out.println("HANG DEBUG BAR 2 DONE Lift Height: " + lift.getLiftExtension());
            System.out.println("HANG DEBUG BAR 2 DONE Arm Angle: " + armPivot.getArmAngle());
            System.out.println("HANG DEBUG BAR 2 DONE Arm power: " + armPivot.armPivot.getPower());
            System.out.println("HANG DEBUG BAR 2 DONE Lift LEFT Power: " + lift.liftLeft.getPower());
            System.out.println("HANG DEBUG BAR 2 DONE Lift RIGHT Power: " + lift.liftRight.getPower());
            System.out.println("HANG DEBUG BAR 2 DONE Lift LEFT Power: " + lift.upperLift.getPower());
            System.out.println("HANG DEBUG BAR 2 DONE Pivot adjusted angle:  " + Math.abs(armPivot.getArmAngle() - pivotAngleWhenAtHardstop));



//            if(armPivot.getArmAngle() < -5)
//            {
//                lift.setSetPoint(0);
//            }

            if(Math.abs(armPivot.getArmAngle() - pivotAngleWhenAtHardstop) > 80 && !liftExtenedToTipBotForward) // this helps ensure that the pivot is closed 90 deg from the hardstop in prev state
            {
                liftExtenedToTipBotForward = true;
                lift.setSetPoint(3); // put lift out a bit to tip bot forward and help pivot motor a bit
            }

            if(lift.getLiftExtension() > 2.5 && liftExtenedToTipBotForward) // this helps ensure that the pivot is closed 90 deg from the hardstop in prev state
            {
//                liftExtenedToTipBotForward = true;
                lift.setSetPoint(0); // put lift out a bit to tip bot forward and help pivot motor a bit
            }

            if (armPivot.getLiftLimitState() && lift.getLiftExtension() < 2 && liftExtenedToTipBotForward) {
                liftSwitchPressedOnce = true;
            }

            if (liftSwitchPressedOnce) {
                lift.setLiftPower(0);  // turns of power to lift motors
            } else {
                lift.updateLiftPosition();
            }

            armPivot.setArmPivotPower(0);
        }

        if (currentState == SuperstructureStates.HANG_BAR_2_FINISH.ordinal()) {
            if (stateFinished) {
                lastHangRestTime = SystemClock.uptimeMillis();
                hangCounter = 0;
                hangPastValidation = false;
                initializeStateVariables();
            }

            System.out.println("HANG DEBUG BAR 2 FINISH Limit Switch State: " + armPivot.getLiftLimitState());
            System.out.println("HANG DEBUG BAR 2 FINISH Lift In State: " + (lift.getLiftExtension() < 5));
            System.out.println("HANG DEBUG BAR 2 FINISH Lift Height: " + lift.getLiftExtension());
            System.out.println("HANG DEBUG BAR 2 FINISH Arm Angle: " + armPivot.getArmAngle());
            System.out.println("HANG DEBUG BAR 2 FINISH Lift Power: " + lift.liftLeft.getPower());

            if (!hangPastValidation && lift.getLiftExtension() > 8 && armPivot.getArmAngle() < 30) {
                hangPastValidation = true;
            }

            if (hangPastValidation) {
                lift.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                lift.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armPivot.update(-7, 1, 5, 0.5, telemetry);
                if (armPivot.getArmAngle() < -3) {
                    if (lift.getLiftExtension() < 1 && armPivot.getArmAngle() < 1) {
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
                /*if (hangCounter % 3 == 0) {
                    lift.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    lift.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                } else {
                    lift.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    lift.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }*/

                hangCounter++;

                if (armPivot.getArmAngle() < 30) {
                    armPivot.setArmPivotPower(-0.15);
                } else {
                    armPivot.setArmPivotPower(-1);
                }
                //armPivot.update(5, 1, 10, 0.85, telemetry);
                lift.setLiftPower(0.05);
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

        if (currentState == SuperstructureStates.COLLECT_SPECIMEN_PREP.ordinal()) {
            if (stateFinished) {
                targetPivotAngle = 6;   // adjust this for optimal specimen on the wall height
                liftWantedHeight = 5;
                armPivot.setIntakeTiltAngle(0);
                initializeStateVariables();
            }

            System.out.println("SPECIMEN PREP Lift In State: " + (lift.getLiftExtension() < 5));
            System.out.println("SPECIMEN PREP Lift Height: " + lift.getLiftExtension());
            System.out.println("SPECIMEN PREP Lift Arm Angle: " + armPivot.getArmAngle());
            System.out.println("SPECIMEN PREP JAW: " + armPivot.intakeJawServo.getPosition());


            if (armPivot.getArmAngle() > 3) {
                System.out.println("SPECIMEN Entered If Statement in State 11");
                armPivot.twist.setPosition(Constants3Motor.TWIST_SERVO_WALL_COLLECTION_POSITION);
                armPivot.intakeTilt.setPosition(Constants3Motor.TILT_SERVO_PARALLEL_WITH_FLOOR);
                armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_WALL_COLLECTION);
                //armPivot.vexIntake.setPower(-.8);
                lift.setSetPoint(liftWantedHeight);


                if (armPivot.getPivotLimitState()) {
                    armPivot.setArmPivotPower(0);
                }
            }
            lift.updateLiftPosition();
//            else {
//                armPivot.setIntakeTiltAngle(0);
//                armPivot.update(9, 0.8, 2,0.6, telemetry);
//            }

            armPivot.update(targetPivotAngle, 0.8, 10, 0.3, telemetry);

        }

        if (currentState == SuperstructureStates.COLLECT_SPECIMEN_WALL.ordinal()) {
            if (stateFinished) {
                armPivotReadytoGrabOffTheWall = false;
                servosGrabOffTheWall = false;
                armPivot.vexIntake.setPower(-.91);
                armPivot.intakeTilt.setPosition(Constants3Motor.TILT_SERVO_PARALLEL_WITH_PIVOT);
                armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_GRAB_POSITION);
                initializeStateVariables();
            }
            //armPivot.vexIntake.setPower(-.91);

            System.out.println("SPECIMEN WALL GRAB  Lift In State: " + (lift.getLiftExtension() < 5));
            System.out.println("SPECIMEN WALL GRAB  Lift Height: " + lift.getLiftExtension());
            System.out.println("SPECIMEN WALL GRAB  Lift Arm Angle: " + armPivot.getArmAngle());
            System.out.println("SPECIMEN WALL GRAB  Jaw: " + armPivot.intakeJawServo.getPosition());
            System.out.println("SPECIMEN WALL GRAB  VexMotor: " + armPivot.vexIntake.getPower());

//            if (SystemClock.uptimeMillis() - stateStartTime > 250 && !servosGrabOffTheWall) {
//                //armPivot.update(14, 0.9, 15, 0.31, telemetry);
//                armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_GRAB_POSITION);
//                servosGrabOffTheWall = true;
//
//            }
            if ((SystemClock.uptimeMillis() - stateStartTime > 500 &&
                    !armPivotReadytoGrabOffTheWall) || (AutoSpecimens.pickupOffWall && !armPivotReadytoGrabOffTheWall)) {
                //armPivot.update(14,    0.9, 15, 0.31, telemetry);
                targetPivotAngle = 21;
                liftWantedHeight = 4;
                //armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_GRAB_POSITION);
                armPivotReadytoGrabOffTheWall = true;

            } else if (SystemClock.uptimeMillis() - stateStartTime > 2000) {
                //armPivot.update(14, 0.9, 15, 0.31, telemetry);
                armPivot.vexIntake.setPower(0);
                liftWantedHeight = .5;
                armPivot.twist.setPosition(Constants3Motor.TWIST_SERVO_HORIZONTAL_POSITION);
                if (lift.getLiftExtension() < 2) {
                    armPivotReadytoGrabOffTheWall = false;
                    servosGrabOffTheWall = false;
                    nextState(SuperstructureStates.RESTING.ordinal()); // changed from specimen Transport
                }

            }
//            else if(!servosGrabOffTheWall){
//                armPivot.vexIntake.setPower(-.9);
//                armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_GRAB_POSITION);
//                servosGrabOffTheWall = true;
//
//            }

            armPivot.update(targetPivotAngle, 0.9, 15, 0.31, telemetry);
            lift.setSetPoint(liftWantedHeight);
            lift.updateLiftPosition();
        }

        if (currentState == SuperstructureStates.SPECIMEN_TRANSPORT.ordinal()) {
            if (stateFinished) {
                liftWantedHeight = .1;
                targetPivotAngle = -2;
                initializeStateVariables();
            }

            if (SystemClock.uptimeMillis() - stateStartTime > 250 && armPivot.getArmAngle() < 0) {
                armPivot.intakeTilt.setPosition(Constants3Motor.TILT_SERVO_90_DEGREES_UP);
                armPivot.setArmPivotPower(0);
            } else {

                if (armPivot.getPivotLimitState()) {
                    armPivot.setArmPivotPower(0);
                } else {
                    armPivot.update(targetPivotAngle, 0.9, 15, 0.1, telemetry);
                }
            }

            lift.setSetPoint(liftWantedHeight);
            lift.updateLiftPosition();
        }

        if (currentState == SuperstructureStates.SPECIMEN_HANG_PREP.ordinal()) {
            if (stateFinished) {
                targetPivotAngle = 90;
                //liftWantedHeight = 0;
                lift.setSetPoint(0);
                armPivot.vexIntake.setPower(0);
                armPivot.intakeTilt.setPosition(Constants3Motor.TILT_SERVO_PARALLEL_WITH_PIVOT);
                armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_GRAB_POSITION);
                initializeStateVariables();
            }

            System.out.println("SPECIMEN HANG PREP Lift Height: " + lift.getLiftExtension());
            System.out.println("SPECIMEN HANG PREP Lift Arm Angle: " + armPivot.getArmAngle());
            System.out.println("SPECIMEN HANG PREP JAW: " + armPivot.intakeJawServo.getPosition());

            if (SystemClock.uptimeMillis() - stateStartTime > 250 && armPivot.getArmAngle() > 60) {
                System.out.println("SPECIMEN Entered If Statement in State 14");
                armPivot.twist.setPosition(Constants3Motor.TWIST_SERVO_WALL_COLLECTION_POSITION);
            }

            if ((gamepad2.right_stick_y) < -0.01  // this is out
                    && lift.getLiftExtension() < Constants3Motor.LIFT_MAX_HORIZONTAL_POSITION_IN) {
                lift.setLiftPower(-gamepad2.right_stick_y);
                lift.setSetPoint(lift.getLiftExtension());
                //liftWantedHeight = lift.getLiftExtension();

            } else if (gamepad2.right_stick_y > 0.01 && lift.getLiftExtension() > 0) { //this is in
                lift.setLiftPower(-gamepad2.right_stick_y);
                lift.setSetPoint(lift.getLiftExtension());
                //liftWantedHeight = lift.getLiftExtension();

            } else {
                //lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();
            }

            //lift.updateLiftPosition();
            armPivot.update(targetPivotAngle, 0.8, 30, 0.25, telemetry);
        }

        if (currentState == SuperstructureStates.SPECIMEN_HANG_CHAMBER.ordinal()) {
            if (stateFinished) {
                liftWantedHeight = 1.5;// change here if problesm
                //TODO: add teleop boolean to change this lift wanted height
                lift.setSetPoint(liftWantedHeight);
                initializeStateVariables();
            }
            System.out.println("SPECIMEN HANG Lift Height: " + lift.getLiftExtension());
            System.out.println("SPECIMEN HANG Lift Arm Angle: " + armPivot.getArmAngle());
            System.out.println("SPECIMEN HANG JAW: " + armPivot.intakeJawServo.getPosition());

            if (lift.getLiftExtension() > 1.0 && SystemClock.uptimeMillis() - stateStartTime > 300) {
                targetPivotAngle = 57;
                if (armPivot.getArmAngle() < 73) {
                    armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_INTAKE_POSITION);
                }
            }

            lift.updateLiftPosition();
            armPivot.update(targetPivotAngle, .75, 20, 0.25, telemetry);
        }

        if (currentState == SuperstructureStates.SPECIMEN_HANG_CHAMBER_TELE.ordinal()) {
            if (stateFinished) {
                liftWantedHeight = lift.getLiftExtension();// change here if problesm
                //TODO: add teleop boolean to change this lift wanted height
                lift.setSetPoint(liftWantedHeight);
                initializeStateVariables();
            }
            System.out.println("SPECIMEN HANG Lift Height: " + lift.getLiftExtension());
            System.out.println("SPECIMEN HANG Lift Arm Angle: " + armPivot.getArmAngle());
            System.out.println("SPECIMEN HANG JAW: " + armPivot.intakeJawServo.getPosition());

            if (lift.getLiftExtension() > 1.0 && SystemClock.uptimeMillis() - stateStartTime > 300) {
                targetPivotAngle = 57;
                if (armPivot.getArmAngle() < 73) {
                    armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_INTAKE_POSITION);
                }
            }

            lift.updateLiftPosition();
            armPivot.update(targetPivotAngle, .5, 20, 0.25, telemetry);
        }

        if (currentState == SuperstructureStates.SPECIMEN_HANG_FRONT_PREP.ordinal()) {
            if (stateFinished) {
                targetPivotAngle = 55;
                //liftWantedHeight = 0;
                lift.setSetPoint(0);
                armPivot.intakeTilt.setPosition(Constants3Motor.TILT_SERVO_PARALLEL_WITH_PIVOT);
                armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_GRAB_POSITION);
                initializeStateVariables();
            }
            System.out.println("SPECIMEN ccEntered If Statement in State 17");

            if (SystemClock.uptimeMillis() - stateStartTime > 250 && armPivot.getArmAngle() > 20) {

                armPivot.twist.setPosition(Constants3Motor.TWIST_SERVO_WALL_COLLECTION_POSITION);
            }

            if ((gamepad2.right_stick_y) < -0.01  // this is out
                    && lift.getLiftExtension() < Constants3Motor.LIFT_MAX_HORIZONTAL_POSITION_IN) {
                lift.setLiftPower(-gamepad2.right_stick_y);
                lift.setSetPoint(lift.getLiftExtension());
                //liftWantedHeight = lift.getLiftExtension();

            } else if (gamepad2.right_stick_y > 0.01 && lift.getLiftExtension() > 0) { //this is in
                lift.setLiftPower(-gamepad2.right_stick_y);
                lift.setSetPoint(lift.getLiftExtension());
                //liftWantedHeight = lift.getLiftExtension();

            } else {
                //lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();
            }

            //lift.updateLiftPosition();
            armPivot.update(targetPivotAngle, 0.8, 30, 0.15, telemetry);
        }

        if (currentState == SuperstructureStates.SPECIMEN_HANG_FRONT_CHAMBER.ordinal()) {
            if (stateFinished) {
                //liftWantedHeight = 1.9;
                //lift.setSetPoint(liftWantedHeight);
                initializeStateVariables();
            }

            if (lift.getLiftExtension() > 1.0 && SystemClock.uptimeMillis() - stateStartTime > 300) {
                targetPivotAngle = 34;
                if (armPivot.getArmAngle() < 37) {
                    armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_INTAKE_POSITION);
                }
            }

            lift.updateLiftPosition();
            armPivot.update(targetPivotAngle, .9, 20, 0.15, telemetry);
        }


        ///////SAMPLE COLLECTION////////////////////////////////////////////

        if (currentState == SuperstructureStates.SAMPLE_COLLECTION_EXTENSTION.ordinal()) {
            if (stateFinished) {
                // init vars as needed
                taskReStartTime = false;
                collectionMode = false;
                sampleCollected = false;
                liftWantedHeight = 11.5;
                armPivot.vexIntake.setPower(0);
                armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_GRAB_POSITION);
                armPivot.setIntakeTiltAngle(90);
                initializeStateVariables();
            }

            if (!sampleCollected) {
                if (lift.getLiftExtension() > 1.5 && !collectionMode) {
                    armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_INTAKE_POSITION);
                    armPivot.vexIntake.setPower(-.91);

                    collectionMode = true;
                }

                if (collectionMode) {
                    armPivot.setIntakeTiltAngle(armPivot.intakeTiltNoArmPower(lift.getLiftExtension()));
                }
            } else {
                armPivot.setIntakeTiltAngle(90);
            }

            if (lift.getLiftExtension() > 10) { // captures task start time
                if(!taskReStartTime) {
                    taskStartTime = SystemClock.uptimeMillis();
                    taskReStartTime =true;
                }

                if(SystemClock.uptimeMillis()-taskStartTime > 650){
                    //collectionMode = false;
                    sampleCollected = true;
                    armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_GRAB_POSITION);
                    armPivot.vexIntake.setPower(0);
                    liftWantedHeight = 0;
                }
            }

            lift.setSetPoint(liftWantedHeight);
            lift.updateLiftPosition();

        }

        if (currentState == SuperstructureStates.UNJAM_INTAKE.ordinal()) {
            if (stateFinished) {
                // init vars as needed
                //liftWantedHeight = 11;
                armPivot.vexIntake.setPower(0);
                armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_GRAB_POSITION);
                armPivot.setIntakeTiltAngle(0);
                targetPivotAngle = (60);

                intakeUnJam = true; //this is for guide button press in teleOpp
                intakeUnJamTwist = false;
                intakeUnJamLimitSwitchPressed = false;
                initializeStateVariables();
            }



            if (SystemClock.uptimeMillis() - stateStartTime > 250 && armPivot.getArmAngle() > 25 && !intakeUnJamTwist) {
                intakeUnJamTwist = true;
                armPivot.twist.setPosition(Constants3Motor.TWIST_SERVO_HORIZONTAL_POSITION);

                if (!armPivot.getLiftLimitState()) {
                    lift.setLiftPower(-0.25);
                    //this resets lift encoders
//                    if(!intakeUnJamLimitSwitchPressed){   // did this to not spam hub with motor sets
//                        intakeUnJamLimitSwitchPressed = true;
//                        lift.setLiftPower(0.25);
//                    }
                } else {
                    lift.setLiftPower(0.0);
                    lift.resetLiftEncoders();
//                    if(intakeUnJamLimitSwitchPressed){
//                        intakeUnJamLimitSwitchPressed = false;
//                        lift.setLiftPower(0.0);
//                        lift.resetLiftEncoders();
//                    }
                }
            }

            if (SystemClock.uptimeMillis() - stateStartTime > 1000 && armPivot.getArmAngle() > 55 && armPivot.getLiftLimitState()) {
                intakeUnJam = false; // this lets us use teleOpp button press again
                nextState(SuperstructureStates.RESTING.ordinal());
            }

            armPivot.update(targetPivotAngle, .9, 20, 0.2, telemetry);
        }


        if (currentState == SuperstructureStates.SPECIMEN_HANG_FRONT_PREP_AUTO.ordinal()) {
            if (stateFinished) {
                targetPivotAngle = 33.6;
                liftWantedHeight = 14;
                armPivot.intakeTilt.setPosition(Constants3Motor.TILT_SERVO_PARALLEL_WITH_PIVOT);
                armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_GRAB_POSITION);
                initializeStateVariables();
            }
            System.out.println("SPECIMEN ccEntered If Statement in State 17");

            if (SystemClock.uptimeMillis() - stateStartTime > 350 && armPivot.getArmAngle() > 20) {
                armPivot.twist.setPosition(Constants3Motor.TWIST_SERVO_WALL_COLLECTION_POSITION);
            }

            if (SystemClock.uptimeMillis()-stateStartTime > 500) {
                armPivot.intakeTilt.setPosition(Constants3Motor.TILT_SERVO_PARALLEL_WITH_PIVOT+0.05);
            }

            lift.setSetPoint(liftWantedHeight);
            lift.updateLiftPosition();

            //lift.updateLiftPosition();
            armPivot.update(targetPivotAngle, 0.8, 30, 0.45, telemetry);
        }

        if (currentState == SuperstructureStates.SPECIMEN_HANG_FRONT_CHAMBER_AUTO.ordinal()) {
            if (stateFinished) {
                //liftWantedHeight = 1.9;
                //lift.setSetPoint(liftWantedHeight);
                initializeStateVariables();
            }

            if (lift.getLiftExtension() > 1.0 && SystemClock.uptimeMillis() - stateStartTime > 300) {
                targetPivotAngle = 26;
                if (armPivot.getArmAngle() < 29) {
                    armPivot.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_INTAKE_POSITION);
                }
            }

            lift.updateLiftPosition();
            armPivot.update(targetPivotAngle, .8, 20, 0.15, telemetry);
        }

        if (currentState == SuperstructureStates.AUTO_PARK.ordinal()) {
            if (stateFinished) {
                liftWantedHeight = 10;
                targetPivotAngle = 45;
                armPivot.intakeTilt.setPosition(Constants3Motor.TILT_SERVO_90_DEGREES_UP);
                lift.setSetPoint(liftWantedHeight);
                initializeStateVariables();
            }

            lift.updateLiftPosition();
            armPivot.update(targetPivotAngle, .8, 20, 0.15, telemetry);
        }

        if (currentState == SuperstructureStates.TELEOP_START.ordinal()) {
            if (stateFinished) {
                liftWantedHeight = lift.getLiftExtension();
                targetPivotAngle = armPivot.getArmAngle();
                lift.setSetPoint(liftWantedHeight);
                initializeStateVariables();
            }

            lift.updateLiftPosition();
            armPivot.update(targetPivotAngle, .8, 20, 0.15, telemetry);
        }
//        if (currentState == SuperstructureStates.RESET_PIVOT_ANGLE.ordinal()) {
//            if (stateFinished) {
//                liftWantedHeight = 0;
//                lift.setSetPoint(liftWantedHeight);
//                initializeStateVariables();
//            }
//
//            lift.updateLiftPosition();
//            if(!armPivot.getPivotLimitState())
//            {
//                armPivot.setArmPivotPower(.8);
//            }
//            else
//            {
//                armPivot.setArmPivotPower(0);
//                armPivot.armPivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                armPivot.armPivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                armPivot.armPivotRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                armPivot.armPivotLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                armPivot.armOffset = 101;
//            }
//
//
//        }

        lastUpdateTime = currentTimeMillis;
    }

}

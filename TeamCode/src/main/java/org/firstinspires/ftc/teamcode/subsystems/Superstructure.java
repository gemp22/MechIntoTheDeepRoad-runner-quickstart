package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotPosition.AngleWrap;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoSpecimens;
import org.firstinspires.ftc.teamcode.ButtonPress;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;

public class Superstructure {
    private ArmPivot armPivot;
    private Lift lift;
    private Robot robot;
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
        HANG_BAR_2_FINISH,
        ROBOT_RESTING_ON_BAR_2,
        HOLD_EVERYTHING,

        //specimen States
        COLLECT_SPECIMEN_PREP,
        COLLECT_SPECIMEN_WALL,
        SPECIMEN_TRANSPORT,
        SPECIMEN_HANG_PREP,
        SPECIMEN_HANG_CHAMBER,

        SPECIMEN_HANG_FRONT_PREP,
        SPECIMEN_HANG_FRONT_CHAMBER,

        //SAMPLE COLLECTION //////
        SAMPLE_COLLECTION_EXTENSTION,
        SAMPLE_COLLECTION_INTAKE,
        UNJAM_INTAKE;


        //manual control

    }

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
        System.out.println("Delivery Level Ordinal: " + Superstructure.SuperstructureStates.DELIVERY_LEVEL_1.ordinal());
    }

    public Superstructure(ArmPivot armPivot, Lift lift, Robot robot) {
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

        if (currentState == SuperstructureStates.RESTING.ordinal()) {
            if (stateFinished) {
                restingStateStartingAngle = armPivot.getArmAngle();
                // init vars as needed
                setToHomeResting = false;
                collectionMode = false;
                outTakeMode = false;
                taskReStartTime = false;
                taskReStartTime2 = false;
//                liftStateStartExtension = lift.getLiftExtension();
//                pivotStateStartAngle = armPivot.getArmAngle();
                armPivot.vexIntake.setPower(0);
                lift.setSetPoint(0);
                liftWantedHeight = 0;
                armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_GRAB_POSITION);
                armPivot.intakeTilt.setPosition(Constants.TILT_SERVO_PARALLEL_WITH_PIVOT);
                if(lift.getLiftExtension()<6 || armPivot.getArmAngle()<60){
                    isLiftOrPivotSmall = true;
                }

                initializeStateVariables();
            }

            if(isLiftOrPivotSmall){

                if(!taskReStartTime){
                taskStartTime = SystemClock.uptimeMillis();
                taskReStartTime = true;
                }

                if(SystemClock.uptimeMillis() - taskStartTime > 1000){ //wait for jaw and tilt
                    armPivot.twist.setPosition(Constants.TWIST_SERVO_HORIZONTAL_POSITION);
                    if (lift.getLiftExtension()<3 && armPivot.getArmAngle() > -3 && SystemClock.uptimeMillis() - taskStartTime > 1000) { //gives time for twist before Pivot goes down
                        armPivot.setIntakeTiltAngle(90);
                        armPivot.update(-3, 0.3, 40, 0.05, telemetry);
                    }else {
                        //armPivot.setIntakeTiltAngle(90);
                        armPivot.armPivotLeft.setPower(0);
                        armPivot.armPivotRight.setPower(0);
                    }
                }
            }else if (lift.getLiftExtension() < 12 ) { //this gives time for jaw and tilt to get right before twist and pivot for long lift extensions
                armPivot.twist.setPosition(Constants.TWIST_SERVO_HORIZONTAL_POSITION);
                if (lift.getLiftExtension() < 3 && armPivot.getArmAngle() > -3) {
                    armPivot.setIntakeTiltAngle(90);
                    armPivot.update(-3, 0.3, 40, 0.05, telemetry);
                }else {
                    armPivot.armPivotLeft.setPower(0);
                    armPivot.armPivotRight.setPower(0);
//                    if(!taskReStartTime2){
//                        taskStartTime2 = SystemClock.uptimeMillis();
//                        taskReStartTime2 = true;
//                    }
//                    armPivot.setIntakeTiltAngle(90);
//                    if(SystemClock.uptimeMillis() - taskStartTime > 1000){
//                    armPivot.armPivotLeft.setPower(0);
//                    armPivot.armPivotRight.setPower(0);
//                    }
                }
            }else{
                System.out.println("Angle we should be setting to: " + restingStateStartingAngle);
                armPivot.update(restingStateStartingAngle,.75,20,0.10, telemetry);
            }

            if ((gamepad2.right_stick_y) < -0.01  // this is out
                    && lift.getLiftExtension() < Constants.LIFT_MAX_HORIZONTAL_POSITION_IN) {
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
                    armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_INTAKE_POSITION);
                    armPivot.vexIntake.setPower(-.91);
                    collectionMode = true;
                } else {
                    armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_GRAB_POSITION);
                    armPivot.vexIntake.setPower(0);
                    armPivot.setIntakeTiltAngle(0);
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
                    armPivot.setIntakeTiltAngle(Constants.TILT_INTAKE_ANGLE_OUTTAKE);
                    armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_WALL_COLLECTION);
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
                armPivot.twist.setPosition(Constants.TWIST_SERVO_BASKET_DEPOSIT_POSITION);
                armPivot.intakeTilt.setPosition(Constants.TILT_SERVO_PARALLEL_WITH_FLOOR);
                lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();

                if (!armPivot.getPivotLimitState()) {
                    armPivot.setArmPivotPower(0.25);
                } else {
                    armPivot.setArmPivotPower(0);
                }
            }

            armPivot.setIntakeTiltAngle(0);
            armPivot.update2(94, 0.75, 30, 0.25, telemetry);
            armPivot.setLeftArmMotorSetPoint(armPivot.armPivotLeft.getCurrentPosition());
            armPivot.setRightArmMotorSetPoint(armPivot.armPivotRight.getCurrentPosition());

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
                armPivot.twist.setPosition(Constants.TWIST_SERVO_BASKET_DEPOSIT_POSITION);
                armPivot.intakeTilt.setPosition(Constants.TILT_SERVO_PARALLEL_WITH_FLOOR);
                lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();

                if (!armPivot.getPivotLimitState()) {
                    armPivot.setArmPivotPower(0.25);
                } else {
                    armPivot.setArmPivotPower(0);
                }
            }

                armPivot.setIntakeTiltAngle(0);
                armPivot.update2(94, 0.75, 30, 0.25, telemetry);
                armPivot.setLeftArmMotorSetPoint(armPivot.armPivotLeft.getCurrentPosition());
                armPivot.setRightArmMotorSetPoint(armPivot.armPivotRight.getCurrentPosition());

        }
        if (currentState == SuperstructureStates.DELIVERY_SAMPLE_DROP.ordinal()) {
            if (stateFinished) {
                armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_DROP_POSITION);
                armPivot.vexIntake.setPower(.3);
                armPivot.setIntakeTiltAngle(-64);
                restingStateStartingAngle = armPivot.getArmAngle();
                initializeStateVariables();
            }

            armPivot.update(restingStateStartingAngle,.75,15,0.15, telemetry);

            if (gamepad1.right_bumper) {
                armPivot.vexIntake.setPower(0);
                armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_GRAB_POSITION);
                armPivot.setIntakeTiltAngle(Constants.TILT_SERVO_PARALLEL_WITH_PIVOT);
            }


        }


        ////// Hang States /////////

        if (currentState == SuperstructureStates.HANG_BAR_1_PREP.ordinal()) {
            if (stateFinished) {
                liftWantedHeight = 16;
                initializeStateVariables();
            }
            System.out.println("HANG_BAR_1PREP DEBUG Arm Angle: " + armPivot.getArmAngle());

            lift.setSetPoint(liftWantedHeight);
            lift.updateLiftPosition();
            armPivot.twist.setPosition(Constants.TWIST_SERVO_HORIZONTAL_POSITION);
            armPivot.setIntakeTiltAngle(90);
            armPivot.update(55, 0.75, 30, 0.2, telemetry);
        }

        if (currentState == SuperstructureStates.HANG_BAR_1.ordinal()) {
            if (stateFinished) {
                initializeStateVariables();
            }

            System.out.println("HANG DEBUG BAR 1 Limit Switch State: " + armPivot.getLiftLimitState());
            System.out.println("HANG DEBUG BAR 1 Lift In State: " + (lift.getLiftExtension() < 5));
            System.out.println("HANG DEBUG BAR 1 Lift Height: " + lift.getLiftExtension());
            System.out.println("HANG DEBUG BAR 1 Arm Angle: " + armPivot.getArmAngle());

            if (lift.getLiftExtension() < 2.4 && armPivot.getArmAngle() < 5) {
                lift.setLiftPower(0);
                armPivot.setArmPivotPower(0);
                nextState(SuperstructureStates.HANG_BAR_1_RESTING.ordinal());
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
        if (currentState == SuperstructureStates.HANG_BAR_1_RESTING.ordinal()) {
            if (stateFinished) {
                initializeStateVariables();
            }
        }

        if (currentState == SuperstructureStates.HANG_BAR_2_PREP.ordinal()) {
            if (stateFinished) {
                lastPitchAngle =  robot.drive.lazyImu.get().getRobotYawPitchRollAngles().getPitch();
                liftIsReadyToAttachToBar2 = false;
                liftIsReadyForBar2Pivot = false;
                initializeStateVariables();
            }
            double elapsedTime = (double) (currentTimeMillis - lastUpdateTime)/1000.0;

            lastUpdateTime = currentTimeMillis;

            double currentPitchAngle =  robot.drive.lazyImu.get().getRobotYawPitchRollAngles().getPitch();
            double pitchVelocity = AngleWrap(currentPitchAngle-lastPitchAngle) / elapsedTime;


            armPivot.twist.setPosition(Constants.TWIST_SERVO_HORIZONTAL_POSITION);
            armPivot.setIntakeTiltAngle(90);

            System.out.println("HANG DEBUG BAR 2 PREP Limit Switch State: " + armPivot.getLiftLimitState());
            System.out.println("HANG DEBUG BAR 2 PREP Lift In State: " + (lift.getLiftExtension() < 5));
            System.out.println("HANG DEBUG BAR 2 PREP Lift Height: " + lift.getLiftExtension());
            System.out.println("HANG DEBUG BAR 2 PREP Arm Angle: " + armPivot.getArmAngle());


            if (SystemClock.uptimeMillis() - stateStartTime > 750 && lift.getLiftExtension() > 7 && !liftIsReadyToAttachToBar2) {
                liftIsReadyForBar2Pivot = true;
            }
            if (liftIsReadyForBar2Pivot && !liftIsReadyToAttachToBar2) {
                if (armPivot.getArmAngle() > 18) {
                    lift.setSetPoint(21.5);
                    lift.updateLiftPosition();
                    armPivot.update(22, 0.4, 15, 0.3, telemetry);

                    if (lift.getLiftExtension() > 20) {
                        liftIsReadyToAttachToBar2 = true;
                    }
                    //nextState(SuperstructureStates.HANG_BAR_2.ordinal());
                } else {
                    System.out.println("HANG DEBUG BAR 2 PREP Drive Pitch Angle Vel: " + pitchVelocity);
                    System.out.println("HANG DEBUG BAR 2 PREP Drive Pitch Delta Angle: " + AngleWrap(currentPitchAngle-lastPitchAngle));
                    lift.setSetPoint(8);
                    lift.updateLiftPosition();
                    armPivot.update(22, 0.4, 15, 0.3, telemetry);
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
                    nextState(SuperstructureStates.HANG_BAR_TWO_PREP_RESTING.ordinal());
                }
            }

            lastPitchAngle = currentPitchAngle;
        }
        if (currentState == SuperstructureStates.HANG_BAR_TWO_PREP_RESTING.ordinal()) {
            if (stateFinished) {
                initializeStateVariables();
            }
            armPivot.update(14, 0.5, 15, 0.3, telemetry);
            lift.setSetPoint(21.5);
            lift.updateLiftPosition();
        }

        if (currentState == SuperstructureStates.HANG_BAR_2.ordinal()) {
            if (stateFinished) {
                liftSwitchPressedOnce = false;
                initializeStateVariables();
            }

            if (!armPivot.getLiftLimitState() && !liftSwitchPressedOnce) {
                lift.setLiftPower(-1);
                armPivot.setArmPivotPower(0);
            }

            if (armPivot.getLiftLimitState()) {
                liftSwitchPressedOnce = true;
            }

            if (liftSwitchPressedOnce) {
                lift.setSetPoint(0);
                lift.updateLiftPosition();
                armPivot.update(20, 1, 10, 0.3, telemetry);
                if (armPivot.getArmAngle() < 40) {
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

            if (SystemClock.uptimeMillis() - stateStartTime > 3500) {
                lift.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                lift.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armPivot.update(-5, 1, 10, 0.5, telemetry);
                if (armPivot.getArmAngle() < 1) {
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
                if (SystemClock.uptimeMillis() - lastHangRestTime >= 10) {
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

        if (currentState == SuperstructureStates.COLLECT_SPECIMEN_PREP.ordinal()) {
            if (stateFinished) {
                targetPivotAngle = 9.5;   // adjust this for optimal specimen on the wall height
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
                armPivot.twist.setPosition(Constants.TWIST_SERVO_WALL_COLLECTION_POSITION);
                armPivot.intakeTilt.setPosition(Constants.TILT_SERVO_PARALLEL_WITH_FLOOR);
                armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_WALL_COLLECTION);
                //armPivot.vexIntake.setPower(-.8);
                lift.setSetPoint(liftWantedHeight);
                lift.updateLiftPosition();

                if (armPivot.getPivotLimitState()) {
                    armPivot.setArmPivotPower(0);
                }
            }
//            else {
//                armPivot.setIntakeTiltAngle(0);
//                armPivot.update(9, 0.8, 2,0.6, telemetry);
//            }

            armPivot.update(targetPivotAngle, 0.8, 11, 0.3, telemetry);

        }

        if (currentState == SuperstructureStates.COLLECT_SPECIMEN_WALL.ordinal()) {
            if (stateFinished) {
                armPivotReadytoGrabOffTheWall = false;
                servosGrabOffTheWall = false;
                armPivot.vexIntake.setPower(-.91);
                armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_GRAB_POSITION);
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
            if ((SystemClock.uptimeMillis() - stateStartTime > 1000 &&
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
                armPivot.twist.setPosition(Constants.TWIST_SERVO_HORIZONTAL_POSITION);
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
                armPivot.intakeTilt.setPosition(Constants.TILT_SERVO_90_DEGREES_UP);
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
                armPivot.intakeTilt.setPosition(Constants.TILT_SERVO_PARALLEL_WITH_PIVOT);
                armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_GRAB_POSITION);
                initializeStateVariables();
            }


            System.out.println("SPECIMEN HANG PREP Lift Height: " + lift.getLiftExtension());
            System.out.println("SPECIMEN HANG PREP Lift Arm Angle: " + armPivot.getArmAngle());
            System.out.println("SPECIMEN HANG PREP JAW: " + armPivot.intakeJawServo.getPosition());

            if (SystemClock.uptimeMillis() - stateStartTime > 250 && armPivot.getArmAngle() > 60) {
                System.out.println("SPECIMEN Entered If Statement in State 14");
                armPivot.twist.setPosition(Constants.TWIST_SERVO_WALL_COLLECTION_POSITION);
            }

            if ((gamepad2.right_stick_y) < -0.01  // this is out
                    && lift.getLiftExtension() < Constants.LIFT_MAX_HORIZONTAL_POSITION_IN) {
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

        if (currentState == SuperstructureStates.SPECIMEN_HANG_CHAMBER.ordinal()) {
            if (stateFinished) {
                liftWantedHeight = 1.9;// change here if problesm
                lift.setSetPoint(liftWantedHeight);
                initializeStateVariables();
            }
            System.out.println("SPECIMEN HANG Lift Height: " + lift.getLiftExtension());
            System.out.println("SPECIMEN HANG Lift Arm Angle: " + armPivot.getArmAngle());
            System.out.println("SPECIMEN HANG JAW: " + armPivot.intakeJawServo.getPosition());

            if (lift.getLiftExtension() > 1.0 && SystemClock.uptimeMillis() - stateStartTime > 300) {
                targetPivotAngle = 57;
                if (armPivot.getArmAngle() < 75) {
                    armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_INTAKE_POSITION);
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
                armPivot.intakeTilt.setPosition(Constants.TILT_SERVO_PARALLEL_WITH_PIVOT);
                armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_GRAB_POSITION);
                initializeStateVariables();
            }
            System.out.println("SPECIMEN Entered If Statement in State 17");

            if (SystemClock.uptimeMillis() - stateStartTime > 250 && armPivot.getArmAngle() > 20) {

                armPivot.twist.setPosition(Constants.TWIST_SERVO_WALL_COLLECTION_POSITION);
            }

            if ((gamepad2.right_stick_y) < -0.01  // this is out
                    && lift.getLiftExtension() < Constants.LIFT_MAX_HORIZONTAL_POSITION_IN) {
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
                targetPivotAngle = 35;
                if (armPivot.getArmAngle() < 40) {
                    armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_INTAKE_POSITION);
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
                liftWantedHeight = 11;
                armPivot.vexIntake.setPower(0);
                armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_GRAB_POSITION);
                armPivot.setIntakeTiltAngle(90);
                initializeStateVariables();
            }

            if (!sampleCollected) {
                if (lift.getLiftExtension() > 1.5 && !collectionMode) {
                    armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_INTAKE_POSITION);
                    armPivot.vexIntake.setPower(-.91);

                    collectionMode = true;
                }

                if (collectionMode) {
                    armPivot.setIntakeTiltAngle(armPivot.intakeTiltNoArmPower(lift.getLiftExtension()));
                }
            } else {
                armPivot.setIntakeTiltAngle(90);
            }

            if (lift.getLiftExtension() > 10 ) { // captures task start time
                if(!taskReStartTime) {
                    taskStartTime = SystemClock.uptimeMillis();
                    taskReStartTime =true;
                }

                if(SystemClock.uptimeMillis()-taskStartTime > 1000){
                    //collectionMode = false;
                    sampleCollected = true;
                    armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_GRAB_POSITION);
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
                armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_GRAB_POSITION);
                armPivot.setIntakeTiltAngle(0);
                targetPivotAngle = (60);

                intakeUnJam = true; //this is for guide button press in teleOpp
                intakeUnJamTwist = false;
                intakeUnJamLimitSwitchPressed = false;
                initializeStateVariables();
            }



            if (SystemClock.uptimeMillis() - stateStartTime > 250 && armPivot.getArmAngle() > 25 && !intakeUnJamTwist) {
                intakeUnJamTwist = true;
                armPivot.twist.setPosition(Constants.TWIST_SERVO_HORIZONTAL_POSITION);

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

        lastUpdateTime = currentTimeMillis;
    }

}

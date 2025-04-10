/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Superstructure;

import java.util.ArrayList;
import java.util.HashMap;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Disabled
@TeleOp(name = "ATeleopResetEncoders", group = "Mechbot")
public class ATeleopResetEncoders extends Robot {

    boolean justDidAReapproach = false;
    boolean gamepad2Trigger = false;
    boolean gamepad2TriggerPreVal = false;


    boolean intakeUnJamPreValue = false;

    double robotLiftMaxTicks = 10573;


    @Override
    public void init() {
        isAuto = false;
        resetEncoders = true;
        AutoSpecimens.pickupOffWall = false;
        super.init();      //Ask Miles what this is?

        drive = new MecanumDrive(hardwareMap, new Pose2d(17.75 / 2, 23.75, Math.PI));  // this initilizes the Odo?

    }

    @Override
    public void start() {
        super.start(); //what is super.start?

        superstructure.nextState(Superstructure.SuperstructureStates.TELEOP_START.ordinal());

        //pawright.setPosition(0.59);

        //droneAndRobotLiftRotator.setDroneAndRobotLiftRotatorPosition(0.8);


        //fourBars.setFourBarPosition(0.4);
        //pixelLift.pControllerPixelLift.setSetPoint(600);
        //startTime = System.currentTimeMillis();
    }

    @Override
    public void stop() {
        super.stop(); //what is super.stop?
        //droneAndRobotLiftRotator.droneAndRobotLiftRotator.setPwmDisable();

        //for rumble added 4/6
        //runtime.reset();
    }

    private boolean pickupAutomation = false;
    private long pickupAutomationStartTime = 0;
    private boolean liftPrepDeliveryAutomation = false;
    private long liftPrepDeliveryAutomationStarTime = 0;

    private boolean dropAutomation = false;
    private boolean dropAutomation2 = false;
    private long dropAutomationStarTime = 0;
    private boolean setIntakeAutomation = false;
    private long setIntakeAutomationStarTime = 0;
    private boolean driveTrainCorrection = false;
    double currentPoint = 0;
    boolean isRight = false;
    boolean isLeft = false;
    private boolean autoPilotEnabled = false;
    private boolean atBasket = false;
    private long timeAtBasket = 0;
    private final double SCALE_FACTOR = 2.7;
    private double startX = 0;
    private double startY = 0;

    private double startXDriveBridge = 0;
    private double startYDriveBridge = 0;

    private boolean lockHeading = false;

    private Pose2d targetPose = new Pose2d(0, 0, 0);

    private ArrayList<Double> distances = new ArrayList<>();

    private boolean intakeOn = false;
    private boolean outtakeOn = false;


    private boolean hangAutomation = false;
    private long hangAutomationStartTime = 0;

    private int state = 0;
    private int hangButton = 0;

    private boolean liftIsDown = false;
    private long liftIsDownTime = 0;

    private int targetDrop = 0;

    private HashMap<Integer, PointDouble> yellowDropBlue = new HashMap<Integer, PointDouble>() {{
        put(0, new PointDouble(30, 118));
        put(1, new PointDouble(36, 118));
        put(2, new PointDouble(42, 118));
    }};

    private HashMap<Integer, PointDouble> yellowDropRed = new HashMap<Integer, PointDouble>() {{
        put(0, new PointDouble(102, 118));
        put(1, new PointDouble(109, 118));
        put(2, new PointDouble(114, 118));
    }};

    private ArrayList<PointDouble> driveUnderBridgePoints = new ArrayList<PointDouble>() {{
        add(new PointDouble(10.3, 95));
        add(new PointDouble(32.3 + 0.5, 95));
        add(new PointDouble(104.7 + 0.5, 95));
        add(new PointDouble(128.6 + 0.5, 95));
        add(new PointDouble(10.3 + 0.5, 95 - 70));
        add(new PointDouble(32.3 + 0.5, 95 - 70));
        add(new PointDouble(104.7 + 0.5, 95 - 70));
        add(new PointDouble(128.6 + 0.5, 95 - 70));
    }};

    PointDouble closestPoint = new PointDouble(0, 0);
    private int closestPointIndex = 0;

    private boolean autoDriveToHang = false;
    //private PointDouble autoDriveToDroneLaunchPosition = new PointDouble();

    private double launchStartX = 0;
    private double launchStartY = 0;

    public enum Alliance {
        RED,
        BLUE
    }

    public Alliance alliance;

    boolean wePressed = false;

    private boolean intakeToggle = true;

    private double liftWantedHeight = 0;

    private long liftRestingStartTime = 0;

    @Override
    public void mainLoop() {
        super.mainLoop();
        telemetry.addData("arm pivot current angle", armPivot.getArmAngle());
        ButtonPress.giveMeInputs(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.dpad_up,
                gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.right_bumper,
                gamepad1.left_bumper, gamepad1.left_stick_button, gamepad1.right_stick_button,
                gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y, gamepad2.dpad_up,
                gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.right_bumper,
                gamepad2.left_bumper, gamepad2.left_stick_button, gamepad2.right_stick_button);

        if (!hangAutomation && !autoPilotEnabled && !atBasket && !gamepad1.a && !autoDriveToHang) {  // this is the mechanum field centeric control does this make it so can't drive when in auto??
            movement_y = -gamepad1.left_stick_y;
            movement_x = gamepad1.left_stick_x;
            movement_turn = -gamepad1.right_stick_x;
        }
        drive.applyMovementDirectionBased();//this applys movement useing the mecanumDrive class

//        if (gamepad1.dpad_up) {
//            wantedX += 0.05;
//        }
//
//        if (gamepad1.dpad_down && !armPivot.getLiftLimitState()) {
//            wantedX -= 0.05;
//        }

        if (armPivot.getLiftLimitState() && !liftLimitPreValue) {
            lift.resetLiftEncoders();
            wantedX =0;
        }
        liftLimitPreValue = armPivot.getLiftLimitState();

        //lift.setSetPoint(wantedX);
        //lift.updateLiftPosition();

        // lift.setLiftPower(-gamepad1.right_stick_y);

        /*if (gamepad1.right_trigger > 0.05) {
            lift.setLiftPower(gamepad1.right_trigger);
            wantedX = lift.getLiftExtension();
           // int liftLeftTicks = lift.liftLeft.getCurrentPosition();
            //int liftRightTicks = lift.liftRight.getCurrentPosition();
        }

        else if (gamepad1.left_trigger > 0.05 && !armPivot.liftLimitSwitch.getState()) {
            lift.setLiftPower(-gamepad1.left_trigger);
            wantedX = lift.getLiftExtension();
        }*/

        //if (Math.abs(gamepad2.right_stick_y)>0.05){
        // lift.setLiftPower(gamepad2.right_stick_y);
        // }

        System.out.println("Lift Encoders: " + lift.getLiftExtension());
        System.out.println("Lift Encoders RAW: " + lift.liftRight.getCurrentPosition());


        //System.out.println("Delivery Level Ordinal: " + Superstructure.SuperstructureStates.DELIVERY_LEVEL_1.ordinal());
        // Basket Delivery State Machines
        if (ButtonPress.isGamepad2_y_pressed()) {
            System.out.println("gp2 Y Triangle is Pressed");
            superstructure.nextState(Superstructure.SuperstructureStates.DELIVERY_LEVEL_2.ordinal());

        } else if (ButtonPress.isGamepad2_x_pressed()) {
            System.out.println("gp2 X/ square is Pressed");
            //scoringState = ScoringStates.SCORING_LEVEL_1;
            superstructure.nextState(Superstructure.SuperstructureStates.DELIVERY_LEVEL_1.ordinal());

        } else if (ButtonPress.isGamepad2_a_pressed()) {
            //scoringState = ScoringStates.RESTING;
            System.out.println("gp2 A/X is Pressed");
            superstructure.nextState(Superstructure.SuperstructureStates.RESTING.ordinal());
            //liftRestingStartTime = System.currentTimeMillis();

        } else if (ButtonPress.isGamepad2_b_pressed()) {
            //scoringState = ScoringStates.PICKUP;
            System.out.println("gp2 B/O is Pressed");
            superstructure.nextState(Superstructure.SuperstructureStates.SPECIMEN_HANG_FRONT_PREP.ordinal());

        } else if (gamepad2.right_trigger > 0.05 && armPivot.getArmAngle()>30&& !gamepad2TriggerPreVal) { //this is in
            superstructure.nextState(Superstructure.SuperstructureStates.SPECIMEN_HANG_FRONT_CHAMBER.ordinal());

        }else if (ButtonPress.isGamepad2_dpad_left_pressed()) { //// Specimens pick up and dro
            System.out.println("gp2 Dpad left");
            superstructure.nextState(Superstructure.SuperstructureStates.COLLECT_SPECIMEN_PREP.ordinal());

        } else if (ButtonPress.isGamepad2_dpad_right_pressed()) {
            System.out.println("gp2 Dpad right");
            //scoringState = ScoringStates.SCORING_LEVEL_1;
            superstructure.nextState(Superstructure.SuperstructureStates.SPECIMEN_HANG_PREP.ordinal());

        } else if (ButtonPress.isGamepad2_left_bumper_pressed()) {
            System.out.println("gp2 Dpad left bumper");
            superstructure.nextState(Superstructure.SuperstructureStates.COLLECT_SPECIMEN_WALL.ordinal());

        } else if (ButtonPress.isGamepad2_right_bumper_pressed()) {
            System.out.println("gp2 Dpad right bumper");
            //scoringState = ScoringStates.SCORING_LEVEL_1;
            superstructure.nextState(Superstructure.SuperstructureStates.SPECIMEN_HANG_CHAMBER_TELE.ordinal());
        }

//        if (ButtonPress.isGamepad1_a_pressed()) {
//            //superstructure.liftWantedHeight = 16;
//            superstructure.nextState(Superstructure.SuperstructureStates.HANG_BAR_1_PREP.ordinal());
//        }

//        if (ButtonPress.isGamepad1_b_pressed()) {
//            superstructure.liftWantedHeight = 0;
//            superstructure.nextState(Superstructure.SuperstructureStates.HANG_BAR_1.ordinal());
//        }

        if (ButtonPress.isGamepad1_a_pressed()) {
            hangButton++;
            if (hangButton == 1) {
                superstructure.nextState(Superstructure.SuperstructureStates.HANG_BAR_1_PREP.ordinal());
            } else if (hangButton == 2) {
                superstructure.nextState(Superstructure.SuperstructureStates.HANG_BAR_1.ordinal());
            } else if (hangButton == 3) {
                superstructure.nextState(Superstructure.SuperstructureStates.HANG_BAR_2_PREP.ordinal());
            } else if (hangButton == 4) {
                superstructure.nextState(Superstructure.SuperstructureStates.HANG_BAR_2.ordinal());
            }
        }

        telemetry.addData("Lift Limit Switch", armPivot.getLiftLimitState());


//        if (ButtonPress.isGamepad2_dpad_up_pressed()) {
//            superstructure.liftWantedHeight += 0.5;
//        } else if (ButtonPress.isGamepad2_dpad_down_pressed()) {
//            superstructure.liftWantedHeight -= 0.5;
//        }
//        if(Math.abs(gamepad2.right_stick_y) > 0.05 && lift.getLiftExtension() >= -.02
//                && lift.getLiftExtension() < Constants.LIFT_MAX_HORIZONTAL_POSITION_IN)
//        {
//            lift.setLiftPower(-gamepad2.right_stick_y);
//            superstructure.liftWantedHeight = lift.getLiftExtension();
//        }

        if (ButtonPress.isGamepad1_y_pressed()) {

            superstructure.nextState(Superstructure.SuperstructureStates.HOLD_EVERYTHING.ordinal());
        }
//        if (ButtonPress.isGamepad1_x_pressed()) {
//
//            superstructure.nextState(Superstructure.SuperstructureStates.HANG_BAR_2.ordinal());
//        }

        if(gamepad2.right_trigger > 0.05){
            gamepad2TriggerPreVal = true;
        }else {
            gamepad2TriggerPreVal = false;
        }



        /*

        }else if (scoringState == ScoringStates.PICKUP) {
            lift.setLiftPower(-gamepad2.right_stick_y);

            if (lift.getLiftExtension()>6){
                armPivot.setIntakeTiltAngle(armPivot.intakeTiltNoArmPower(lift.getLiftExtension()));
            }
        }

        if (ButtonPress.isGamepad1_a_pressed()) {
            wePressed = true;
            intakeTiltState += 1;
        }*/



        /*if (ButtonPress.isGamepad1_a_pressed() && lift.getLiftExtension() > 4.33) {
            if (intakeToggle) {
                intakeToggle = false;
            } else {
                armPivot.setIntakeTiltAngle(60);
                intakeToggle = true;
            }
        }*/

        /*if (!intakeToggle && lift.getLiftExtension()>4.33){
            armPivot.setIntakeTiltAngle(armPivot.intakeTiltNoArmPower(lift.getLiftExtension()));
        }*/

        /*if( gamepad1.right_bumper){
            armPivot.vexIntake.setPower(-.91);
        }
        */

        if(ButtonPress.isGamepad1_left_bumper_pressed()){
            superstructure.nextState(Superstructure.SuperstructureStates.DELIVERY_SAMPLE_DROP.ordinal());
        }


        if (gamepad1.guide && !intakeUnJamPreValue) {
            if (!superstructure.intakeUnJam) {
                superstructure.nextState(Superstructure.SuperstructureStates.UNJAM_INTAKE.ordinal());
            }
        }
        intakeUnJamPreValue = gamepad1.guide;

        /*
        if (gamepad1.y) {
            armPivot.twist.setPosition(0.772);
        }*/
/*
        if (gamepad1.x){
            armPivot.setIntakeTiltAngle(0);
        }

        if (gamepad1.b){
            armPivot.intakeTilt.setPosition(.118);
        }

*/
//        if (gamepad1.dpad_right){
//            armPivot.intakeJawServo.setPosition(.112);
//        }
//        if (gamepad1.dpad_left){
//            armPivot.intakeJawServo.setPosition(.72);
//        }

        superstructure.update(telemetry, gamepad1, gamepad2);
    }
}

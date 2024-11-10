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
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.HashMap;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//@TeleOp(name = "NewTeleop", group = "Mechbot")
public class   NewTeleop extends AutoMaster {

    boolean justDidAReapproach = false;

    Servo pixelHolder;

    private int height = 1;

    double robotLiftMaxTicks = 10573;

    @Override
    public void init() {
        super.init();      //Ask Miles what this is?

        drive = new MecanumDrive(hardwareMap, new Pose2d(17.75 / 2, 23.75, Math.PI));  // this initilizes the Odo?

        fourBars.setFourBarPosition(0.0);

        wrist.intakeTwist.setPosition(0); // init Twist
        wrist.intakeTilt.setPosition(0); // init Tilt

        intakeJawServo.intakeJawServo.setPosition(0);

        //pixelHolder = hardwareMap.get(Servo.class, "pixelHolder");
        //pixelHolder.setPosition(0.27);

    }

    @Override
    public void start() {
        super.start(); //what is super.start?

        //fourBars.setFourBarPosition(0.4);

        wrist.intakeTwist.setPosition(0);
        wrist.intakeTilt.setPosition(0);

        //pixelTwister.setPixelTwisterPosition(.49);

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

    private boolean pickupPixelAutomation = false;
    private long pickupPixelAutomationStartTime = 0;

    private boolean past90Post = false;

    private boolean liftPrepDeliveryAutomation = false;
    private long liftPrepDeliveryAutomationStarTime = 0;

    private boolean dropPixelAutomation = false;
    private boolean dropPixelAutomation2 = false;
    private long dropPixelAutomationStarTime = 0;


    private boolean setFourBarCenterAutomation = false;
    private long setFourBarCenterAutomationStarTime = 0;


    private boolean driveTrainCorrection = false;

    double currentPoint = 0;

    boolean isRight = false;
    boolean isLeft = false;

    private boolean autoPilotEnabled = false;
    private boolean atBackdrop = false;
    private long timeAtBackdrop = 0;

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


    private boolean shootDroneAutomation = false;
    private long shootDroneAutomationStartTime = 0;

    private int state = 0;

    private boolean liftIsDown = false;
    private long liftIsDownTime = 0;

    //public ArrayList<CurvePoint>

    // for rumble added 4/6 - rest of code on ln 599

    // ElapsedTime runtime = new ElapsedTime();
    // boolean rumblePreVal = false;

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

    private boolean autoDriveToDroneLaunch = false;
    //private PointDouble autoDriveToDroneLaunchPosition = new PointDouble();

    private double launchStartX = 0;
    private double launchStartY = 0;

    public enum Alliance {
        RED,
        BLUE
    }

    public Alliance alliance;

    @Override
    public void mainLoop() {
        ///telemetry.addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));

        //double liftPosition = pixelLift.pixelLift.getCurrentPosition();


        ButtonPress.giveMeInputs(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.dpad_up,
                gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.right_bumper,
                gamepad1.left_bumper, gamepad1.left_stick_button, gamepad1.right_stick_button,
                gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y, gamepad2.dpad_up,
                gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.right_bumper,
                gamepad2.left_bumper, gamepad2.left_stick_button, gamepad2.right_stick_button);


        /*
        if (ButtonPress.isGamepad1_right_stick_button_pressed()) {
            launchStartX = worldXPosition;
            launchStartY = worldYPosition;
        }

         */
/*
        if (gamepad1.right_stick_button) {  // this is for drone launch Auto drive

            autoDriveToDroneLaunch = true;
            ArrayList<CurvePoint> points2 = new ArrayList<>();

            points2.add(new CurvePoint(launchStartX, launchStartY,
                    0, 0, 0, 0, 0, 0));

            double wantedX = alliance == Alliance.RED ? 115 : 31;
            // 80
            double wantedY = alliance == Alliance.RED ? 84 : 89.2;

            points2.add(new CurvePoint(wantedX, wantedY,
                    0.7 * SCALE_FACTOR, 0.7 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            Movement.followCurve(points2, Math.toRadians(90), 1);

            if (Math.abs(Math.hypot(worldXPosition - wantedX, worldYPosition - 87)) < 5) {
                //-68

                double wantedheading = alliance == Alliance.RED ? -90.1 : -90; // was -112 for reed
                Movement.movementResult r = Movement.pointAngle(Math.toRadians(wantedheading), 1, Math.toRadians(30));
            }
        } else {
            autoDriveToDroneLaunch = false;
        }
        */
/*
        if (Math.abs(gamepad1.right_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1) {
            dropPixelAutomation = false;
            autoPilotEnabled = false;  // this turns auto off if sticks are touched
            atBackdrop = false;
            past90Post = false;  //this turns auto off if sticks are touched... should we make it exit out if GP2 lift stick is up too?
            autoDriveToDroneLaunch = false;
        }

*/


        /*if (ButtonPress.isGamepad2_b_pressed()) {
            currentPoint = worldAngle_rad;
        }

        if (gamepad2.b) {
            Movement.movementResult r = Movement.pointAngle(currentPoint, 0.7, Math.toRadians(20));

            movement_x = gamepad2.left_stick_x;
        }*/

        if (!dropPixelAutomation && !autoPilotEnabled && !atBackdrop && !past90Post && !gamepad1.a && !autoDriveToDroneLaunch) {  // this is the mechanum field centeric control does this make it so can't drive when in auto??
            movement_y = -gamepad1.left_stick_y;
            movement_x = gamepad1.left_stick_x;
            movement_turn = -gamepad1.right_stick_x;
        }

        drive.applyMovementDirectionBased(); //this applys movement useing the mecanumDrive class



        if (gamepad2.right_trigger > 0.1) {
            intakeJawServo.intakeJawServo.setPosition(0.0);
            intake.vexIntake.setPower(-.91);
        } else if (gamepad2.left_trigger > 0.1) {
            intakeJawServo.intakeJawServo.setPosition(0.31);
            intake.vexIntake.setPower(-.91);
        } else {
            // intake stuff
            if (ButtonPress.isGamepad1_right_bumper_pressed()) {  //intake toggle
                if (intakeOn) {
                    intakeJawServo.intakeJawServo.setPosition(0);
                    intake.vexIntake.setPower(0);
                    intakeOn = false;
                } else {
                    intakeJawServo.intakeJawServo.setPosition(0.31);
                    intake.vexIntake.setPower(-.91);
                    intakeOn = true;
                }
            } else if (ButtonPress.isGamepad1_left_bumper_pressed()) {  //outtake toggle.
                if (outtakeOn) {
                    intakeJawServo.intakeJawServo.setPosition(0);
                    intake.vexIntake.setPower(0);
                    outtakeOn = false;
                } else {
                    intakeJawServo.intakeJawServo.setPosition(0.31);
                    intake.vexIntake.setPower(-.91);
                    outtakeOn = true;
                }
            }
        }

        // lift stuff for second controller
 /*
        if (ButtonPress.isGamepad2_b_pressed()) {
            height += 1;
        } else if (ButtonPress.isGamepad2_a_pressed()) {
            height -= 1;
        }

        height = Range.clip(height, 1, 10);

        if (ButtonPress.isGamepad2_right_bumper_pressed()) {   // this is for pixel Lift placement height
            //pixelLift.pControllerPixelLift.setSetPoint(700 + height * 300);   // how is this intitated? I only see it in Auto master?
            //fourBars.setFourBarPosition(0);
            intake.vexIntake.setPower(0);
            liftPrepDeliveryAutomation = true;
            liftPrepDeliveryAutomationStarTime = SystemClock.uptimeMillis(); // captures run time and sets to start time variable

        } else if (ButtonPress.isGamepad2_left_bumper_pressed()) {  // this is for pixel lift home state. gets twister to safe position
            //pixelTwister.setPixelTwisterPosition(.49);

            //setFourBarCenterAutomation = true;
            //setFourBarCenterAutomationStarTime = SystemClock.uptimeMillis();
        }

        if (liftPrepDeliveryAutomation) {  // this gives time for twister to get safe, and pixel lift is in safe elevation then sets 4 bar rotator to placement position
            if (SystemClock.uptimeMillis() - liftPrepDeliveryAutomationStarTime > 250 && liftPosition > 400) {
                //fourBarRotator.setFourBarRotatorPosition(.45); //was .45 !!!!!!!!!!!!!!!!!!!!!!!!!! change requested by drive team on 04/04/24
                //liftPrepDeliveryAutomation = false;
            }
        }
        */

// this is for pixel ready (home) position for grab.. can't grab if lift isnt down...
//
/*
        if (ButtonPress.isGamepad2_y_pressed() && liftPosition < 100) {
            wrist.intakeTwist.setPosition(.16);  //
            wrist.intakeTilt.setPosition(.21);

            //pixelTwister.setPixelTwisterPosition(.49);

            //setFourBarCenterAutomation = true;
            //setFourBarCenterAutomationStarTime = SystemClock.uptimeMillis();
        }
*/
        //telemetry.addData("Is Touched", limitSwitch.getState());
/*
        if (setFourBarCenterAutomation) {
            if (SystemClock.uptimeMillis() - setFourBarCenterAutomationStarTime > 500) { // gives grabbers and twister time to get to home position
                fourBars.setFourBarPosition(0.4);
                fourBarRotator.setFourBarRotatorPosition(0.82);
                if (liftPosition < 250 || SystemClock.uptimeMillis() - setFourBarCenterAutomationStarTime > 3500) {
                    pixelLift.pixelLift.setPower(-0.15);
                    //pixelLift.pControllerPixelLift.setSetPoint(liftposition);
                    boolean lsLiftState = limitSwitch.getState();
                    if (lsLiftState || SystemClock.uptimeMillis() - setFourBarCenterAutomationStarTime > 5000) {
                        pixelLift.pixelLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        pixelLift.pixelLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        pixelLift.pixelLift.setPower(0);
                        setFourBarCenterAutomation = false;
                    }
                } else {
                    pixelLift.pControllerPixelLift.setSetPoint(0);
                    pixelLift.updateLiftPosition();
                }
            }
        }
        */

//THis is for the pixel grab
        /*
        if (ButtonPress.isGamepad2_x_pressed() && liftPosition < 100) {
            //pixelTwister.setPixelTwisterPosition(.49);


            wrist.intakeTwist.setPosition(.16); //
            wrist.intakeTilt.setPosition(.21);

            //fourBars.setFourBarPosition(0.75);
            //fourBarRotator.setFourBarRotatorPosition(0.825);

            intake.vexIntake.setPower(-.91); // turns on intake to have pixels lined up correctly
            pickupPixelAutomation = true;  // starts the pixel pick up state
            pickupPixelAutomationStartTime = SystemClock.uptimeMillis();
        }

         */

        // this is for the pixel pick up
/*
        if (pickupPixelAutomation) {
            double wantedPos = 1;    // this is for the slow pick up target servo pos.
            double currentPos = fourBars.getLastSetPosition();
            currentPos += 0.01; //increments servo pos. each loop
            if (SystemClock.uptimeMillis() - pickupPixelAutomationStartTime > 400 && currentPos >= 0.95) { //  waits for grippers, 4 bars, rotator to get into collection state before grabbing
                //fourBars.setFourBarPosition(1);
                intake.vexIntake.setPower(0);
                wrist.intakeTwist.setPosition(.66);  //grab pixels
                wrist.intakeTilt.setPosition(.74);
                pickupPixelAutomation = false; //turns off pick Auto.
            } else {
                if (SystemClock.uptimeMillis() - pickupPixelAutomationStartTime > 250) {  // this changes the 4 bars with each loop when it gets .95 it'll do the above if satatement
                    if (currentPos < wantedPos) {
                        fourBars.setFourBarPosition(currentPos);
                    }
                    //fourBars.setFourBarPosition(1);
                }
            }
        }
        */
/*
        if (Math.abs(gamepad2.right_stick_y) > 0.01) {
            setFourBarCenterAutomation = false;
        }
 */
/*
        if (Math.abs(gamepad2.right_stick_y) < 0.01 && !setFourBarCenterAutomation) {  //for pixel lift control
            //if (pixelLift.pixelLiftPosition)
            //pixelLift.updateLiftPosition();
            // TODO: IS THIS OKAY?
            pixelLift.updateLiftPosition(); // only update if controller is ).01
        } else if (!setFourBarCenterAutomation) {
            pixelLift.pControllerPixelLift.setSetPoint(liftPosition);  // holds lift at last position using p controller
            pixelLift.pixelLift.setPower(-gamepad2.right_stick_y); // sets power of the lift equal to the stick value.
        }

        if (ButtonPress.isGamepad1_x_pressed() && liftPosition > 300) {  // game pad 1 for dropping
            dropPixelAutomation = true; // toggles to the pixel drop automatin 1 and 2 at the same time
            dropPixelAutomation2 = true;
            dropPixelAutomationStarTime = SystemClock.uptimeMillis();
        }
        */
/*
        if (ButtonPress.isGamepad1_a_pressed()) {
            double minDistance = Double.POSITIVE_INFINITY; // start his off crazy high
            int i = 0;
            for (PointDouble point : driveUnderBridgePoints) {
                double distance = Math.hypot(worldXPosition - point.x, worldYPosition - point.y);
               /* Log.i("DEBUG -----------------------------", "----------");
                Log.i("DEBUG Current Index: ", String.valueOf(i));
                Log.i("DEBUG Bridge Point X: ", String.valueOf(point.x));
                Log.i("DEBUG Bridge Point Y: ", String.valueOf(point.y));
                Log.i("DEBUG Bridge Distance: ", String.valueOf(distance));
                Log.i("DEBUG -----------------------------", "----------");*/
        /*
                if (distance < minDistance) {
                    minDistance = distance;
                    closestPoint = new PointDouble(point.x, point.y);
                    closestPointIndex = i;
                }
                i++;
            }
            */

            /*Log.i("DEBUG ----THIS IS THE ONE WE WANT TO GO TO------", "----------");
            Log.i("DEBUG Current Index: ", String.valueOf(closestPointIndex));
            Log.i("DEBUG Bridge Point X: ", String.valueOf(closestPoint.x));
            Log.i("DEBUG Bridge Point Y: ", String.valueOf(closestPoint.y));
            Log.i("DEBUG -----------------------------", "----------");*/
        /*
            startXDriveBridge = worldXPosition;
            startYDriveBridge = worldYPosition;
        }
        */

        //telemetry.addData("Closest Point X", closestPoint.x);
        //telemetry.addData("Closest Point Y", closestPoint.y);
/*
        if (gamepad1.a) {
            ArrayList<CurvePoint> points = new ArrayList<>();

            points.add(new CurvePoint(startXDriveBridge, startYDriveBridge,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(closestPoint.x, closestPoint.y,
                    0.5 * SCALE_FACTOR, 0.5 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            points.add(new CurvePoint(closestPoint.x, closestPoint.y + (closestPointIndex > 3 ? 70 : -70),
                    0.5 * SCALE_FACTOR, 0.5 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            if (Math.abs(Math.hypot(worldXPosition - closestPoint.x, worldYPosition - closestPoint.y)) < 20) {
                lockHeading = true;
            }

            //drive.applyMovementDirectionBased();

            if (Movement.followCurve(points, closestPointIndex > 3 ? Math.toRadians(-90) : Math.toRadians(90), 3)) {
                drive.stopAllMovementDirectionBased();
            }

            if (lockHeading) {
                Movement.movementResult r = Movement.pointAngle(Math.toRadians(-90), 1, Math.toRadians(30));
            }
        } else {
            lockHeading = false;
        }
*/
/*
        if (dropPixelAutomation2) {
            if (SystemClock.uptimeMillis() - dropPixelAutomationStarTime > 1250) { // was 1250 - added extra time when we added async drops
                pixelTwister.setPixelTwisterPosition(.49);

                setFourBarCenterAutomation = true; // run the 4 bar center automation on line 267. waits 1/2 sec then sets rotator, lift, and 4 bars to placement position
                setFourBarCenterAutomationStarTime = SystemClock.uptimeMillis();

                dropPixelAutomation2 = false;
            }
        }
        */
        /*
        if (dropPixelAutomation) {  // releases the the pixels

            if (pixelTwister.pixelTwister.getPosition() > 0.95 || pixelTwister.pixelTwister.getPosition() < 0.1) //added statements to drop top pixel after waiting
            {

                grippers.setBackGripperPosition(.16);

                if (SystemClock.uptimeMillis() - dropPixelAutomationStarTime > 450) {  // waits .45 secs to drop top pixel
                    grippers.setFrontGripperPosition(.21);
                }

            } else if (pixelTwister.pixelTwister.getPosition() > 0.55 || pixelTwister.pixelTwister.getPosition() < 0.45) //added statements to drop top pixel after waiting
            {

                grippers.setFrontGripperPosition(.21);

                if (SystemClock.uptimeMillis() - dropPixelAutomationStarTime > 450) {  // waits .45 secs to drop top pixel
                    grippers.setBackGripperPosition(.16);
                }

            } else {
                grippers.setFrontGripperPosition(.16);
                grippers.setBackGripperPosition(.21);

            }
            if (SystemClock.uptimeMillis() - dropPixelAutomationStarTime > 750) {  // waits 1 sec then uses odo to back away
                movement_y = 0.25;
            }
            if (SystemClock.uptimeMillis() - dropPixelAutomationStarTime > 1250) { // waits 1.8  sec then ... moves forward? ask Miles about this?
                movement_y = 0;

                dropPixelAutomation = false;
            }
        }
        */

        //telemetry.addData("Lift position", liftPosition);
        //telemetry.addData("In Four Bar Automation", setFourBarCenterAutomation);

        // TODO: REMOVE

      /*
        if (liftPosition > 300 && fourBars.right4Bar.getPosition() < 0.2) {  // if lift is higher than 300 and 4 bars are out can rotate pixel twister
            if (ButtonPress.isGamepad2_dpad_left_pressed()) {  // ask Miles about the .get curent pos in the if statement. this doesnt call each loop?
                pixelTwister.setPixelTwisterPosition(.775);  // twists left to 90
                isLeft = true;
                isRight = false;
            }
            if (ButtonPress.isGamepad2_dpad_right_pressed()) {
                pixelTwister.setPixelTwisterPosition(.22); // twists right to 90tw
                isLeft = false;
                isRight = true;
            }
            if (ButtonPress.isGamepad2_dpad_up_pressed()) {  // defines what to do when at 90 and up is pressed
                if (isRight) {
                    pixelTwister.setPixelTwisterPosition(0); // goes up when right
                } else if (isLeft) {
                    pixelTwister.setPixelTwisterPosition(1); // goes up when left
                } else {
                    pixelTwister.setPixelTwisterPosition(.49); // goes back to home when up is pressed and not already twisted to the correct side.
                }
            }
        }
*/
        /*
        if (ButtonPress.isGamepad2_dpad_down_pressed()) {  // if down is pressed go to down.
            pixelTwister.setPixelTwisterPosition(.49);
            isLeft = false;
            isRight = false;
        }
        */

/*
        if (ButtonPress.isGamepad1_left_stick_button_pressed()) {    // This is teleOpp recal
            //25.4
            // TODO: ADD IN NEW SENSOR

            if (alliance == Alliance.BLUE) {
                double xDir = getRightDistanceIn() + (16.25 / 2); //sensorDistance.getDistance(DistanceUnit.MM) / 25.4; //gets measurement from wall

                drive.pose = new Pose2d(xDir, 121, Math.toRadians(-90)); // this defines the robots pos in front of backdrop... uses xDIR to define x pos
                targetPose = drive.pose; //sets the tele auto drive target postion
            } else {
                double xDir = getLeftDistanceIn() - (16.25 / 2) + 6; //sensorDistance.getDistance(DistanceUnit.MM) / 25.4; //gets measurement from wall

                drive.pose = new Pose2d(131 - xDir, 121, Math.toRadians(-90)); // this defines the robots pos in front of backdrop... uses xDIR to define x pos
                targetPose = drive.pose; //sets the tele auto drive target postion
            }
        }
*/
        // telemetry.addData("RED DISTANCE SENSOR", getLeftDistanceIn());


        /*
        if (ButtonPress.isGamepad1_dpad_right_pressed()) {
            //targetPose = new Pose2d(targetPose.position.x-0.5, targetPose.position.y, targetPose.heading.toDouble());  // this moves target pose right 1/2 inch

            targetDrop = 2;
        }
        */
/*
        if (ButtonPress.isGamepad1_dpad_left_pressed()) {
            //targetPose = new Pose2d(targetPose.position.x+0.5, targetPose.position.y, targetPose.heading.toDouble()); // this moves target pose left 1/2 inch

            targetDrop = 0;
        }
        */
/*
        if (ButtonPress.isGamepad1_dpad_up_pressed()) {
            //targetPose = new Pose2d(targetPose.position.x+0.5, targetPose.position.y, targetPose.heading.toDouble()); // this moves target pose left 1/2 inch

            targetDrop = 1;
        }
        */

        //telemetry.addData("Target X", targetPose.position.x);

        /*

        if (ButtonPress.isGamepad1_y_pressed()) {  // when y is pressed start auto drive
            if (targetPose.position.x != 0 && targetPose.position.y != 0) {
                startX = worldXPosition;  // gets current pos.
                startY = worldYPosition;
                autoPilotEnabled = true;
            }
        }
        */
/*
        if (autoPilotEnabled) {     // auto drive points.
            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(startX, startY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(65, 37,
                    0.8 * SCALE_FACTOR, 0.8 * SCALE_FACTOR, 25, 25,
                    Math.toRadians(60), 0.6));

            points.add(new CurvePoint(65, 83,
                    0.8 * SCALE_FACTOR, 0.8 * SCALE_FACTOR, 25, 25,
                    Math.toRadians(60), 0.6));

            HashMap<Integer, PointDouble> expectedMapping = alliance == Alliance.RED ? yellowDropRed : yellowDropBlue;

            points.add(new CurvePoint(expectedMapping.get(targetDrop).x, expectedMapping.get(targetDrop).y - 15,
                    0.7 * SCALE_FACTOR, 0.7 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            points.add(new CurvePoint(expectedMapping.get(targetDrop).x, expectedMapping.get(targetDrop).y,
                    0.15 * SCALE_FACTOR, 0.15 * SCALE_FACTOR, 15, 8,
                    Math.toRadians(60), 0.6));

            if (worldYPosition > 86) {   // past gate, start auto lift
                past90Post = true;
            }

            if (past90Post) {
                pixelLift.pControllerPixelLift.setSetPoint(700 + height * 300);
                fourBars.setFourBarPosition(0);
                intake.conveyor.setPower(0);

                if (pixelLift.pixelLiftPosition > 400) {
                    fourBarRotator.setFourBarRotatorPosition(.45);
                    past90Post = false;
                }
            }

            double distance = Math.sqrt(Math.pow(worldXPosition - targetPose.position.x, 2) + Math.pow(worldYPosition - targetPose.position.y, 2));

            if (!atBackdrop) {
                if (Movement.followCurve(points, Math.toRadians(-90), 3)) {
                    drive.stopAllMovementDirectionBased();
                    timeAtBackdrop = SystemClock.uptimeMillis();
                    atBackdrop = false;
                    autoPilotEnabled = false;
                    //atBackdrop = true;
                }
            }
            */
/*
            if (atBackdrop) {
                movement_y = -0.2;
                if (SystemClock.uptimeMillis()-timeAtBackdrop > 500) {
                    atBackdrop = false;
                    autoPilotEnabled = false;
                }
            }*/
/*
            if (Math.abs(distance) < 15) {
                Movement.movementResult r = Movement.pointAngle(Math.toRadians(-90), 1, Math.toRadians(30));
            }
            */


        ////////////// THIS IS HANGING STUFF GET GOOD /////
/*
        robotLiftPosition = robotLift.getCurrentPosition();

        if (ButtonPress.isGamepad1_b_pressed()) {

            state += 1;

            if (state == 1) {
                //pControllerRobotLift.setSetPoint(500); - commented out to prevent damage of hoist
                droneAndRobotLiftRotator.setDroneAndRobotLiftRotatorPosition(0.62); // 0.62 works great // bigger number is steeper angle
                shootDroneAutomation = true;
                shootDroneAutomationStartTime = SystemClock.uptimeMillis();
            } else if (state == 2) {
                pControllerRobotLift.setSetPoint(robotLiftMaxTicks);
                droneAndRobotLiftRotator.setDroneAndRobotLiftRotatorPosition(0.3);
                shootDroneAutomation = false;
            } else if (state == 3) {
                droneAndRobotLiftRotator.setDroneAndRobotLiftRotatorPosition(0.8);
                pControllerRobotLift.setSetPoint(100);
                shootDroneAutomation = false;
                state = 0;
            }
        }

        if (shootDroneAutomation) {
            if (SystemClock.uptimeMillis() - shootDroneAutomationStartTime > 500) {
                droneLauncher.setDroneLauncherPosition(0.19);  //launch drone (trigger)
                if (SystemClock.uptimeMillis() - shootDroneAutomationStartTime > 850) {
                    droneAndRobotLiftRotator.setDroneAndRobotLiftRotatorPosition(0.8);
                    droneLauncher.setDroneLauncherPosition(0.1);
                    shootDroneAutomation = false;
                }
            }
        }

        if (gamepad1.right_trigger > .5 && robotLiftPosition < robotLiftMaxTicks) {   ///move lift up and sets controller position

            robotLift.setPower(0.2);
            pControllerRobotLift.setSetPoint(robotLiftPosition);
            shootDroneAutomation = false;

        } else if (gamepad1.left_trigger > .5 && robotLiftPosition > 30) {  //move lift down and sets controller position

            robotLift.setPower(-1);
            pControllerRobotLift.setSetPoint(robotLiftPosition);
            shootDroneAutomation = false;

        } else {                                       //uses proportional controller to hold lift in correct spot
            if (robotLiftPosition < pControllerRobotLift.setPoint) {

                robotLift.setPower(minPowerLift +
                        pControllerRobotLift.getComputedOutput(robotLiftPosition));
            } else {
                robotLift.setPower(minPowerLift -
                        pControllerRobotLift.getComputedOutput(robotLiftPosition));
            }
        }

        //////////////////////////////////////////////////

//        // rumble - endgame * * * * * * * * * * * * * * * * *
//        if (runtime.seconds()>80 && !rumblePreVal) {
//
//            gamepad1.rumble(1,1 ,1000); //(double rumble1, double rumble2, int durationMS);
//            gamepad2.rumble(1,1 ,1000);
//
//            rumblePreVal = true;
//        }
//        //* * * * * * * * * * * * * * * * * * * * * * * * * *


        telemetry.addData("Robot Lift Encoder", robotLiftPosition);

        telemetry.addData("world x", worldXPosition);
        telemetry.addData("world y", worldYPosition);
        telemetry.addData("world ang", Math.toDegrees(worldAngle_rad));
        telemetry.addData("Height", height);
        //  telemetry.addData("runtime:", runtime);
        //telemetry.addData("Wanted Lift Pos", pControllerLift.)

    */
    }

}






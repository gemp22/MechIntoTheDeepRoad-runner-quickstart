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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "TestTeleop", group = "Mechbot")
public class TestTeleop extends AutoMaster {

    boolean justDidAReapproach = false;

    Servo pixelHolder;

    private int height = 1;
    double robotLiftMaxTicks = 10573;
    Wrist wrist = null;
    IntakeJawServo intakeJawServo = null;

    @Override
    public void init() {
        super.init();

        //Ask Miles what this is?

        //drive = new MecanumDrive(hardwareMap, new Pose2d(17.75 / 2, 23.75, Math.PI));  // this initilizes the Odo?

        //fourBars.setFourBarPosition(0.0);

        wrist = new Wrist(hardwareMap);
        intakeJawServo = new IntakeJawServo(hardwareMap);


        wrist.intakeTwist.setPosition(.5); // init Twist
        wrist.intakeTilt.setPosition(.5); // init Tilt

        intakeJawServo.intakeJawServo.setPosition(1);

        //pixelHolder = hardwareMap.get(Servo.class, "pixelHolder");
        //pixelHolder.setPosition(0.27);

    }

    @Override
    public void start() {
        super.start(); //what is super.start?

        wrist.intakeTwist.setPosition(0);
        wrist.intakeTilt.setPosition(.5);


        //startTime = System.currentTimeMillis();
    }

    @Override
    public void stop() {
        super.stop(); //what is super.stop?

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

//    private HashMap<Integer, PointDouble> yellowDropBlue = new HashMap<Integer, PointDouble>() {{
//        put(0, new PointDouble(30, 118));
//        put(1, new PointDouble(36, 118));
//        put(2, new PointDouble(42, 118));
//    }};
//
//    private HashMap<Integer, PointDouble> yellowDropRed = new HashMap<Integer, PointDouble>() {{
//        put(0, new PointDouble(102, 118));
//        put(1, new PointDouble(109, 118));
//        put(2, new PointDouble(114, 118));
//    }};
//
//    private ArrayList<PointDouble> driveUnderBridgePoints = new ArrayList<PointDouble>() {{
//        add(new PointDouble(10.3, 95));
//        add(new PointDouble(32.3 + 0.5, 95));
//        add(new PointDouble(104.7 + 0.5, 95));
//        add(new PointDouble(128.6 + 0.5, 95));
//        add(new PointDouble(10.3 + 0.5, 95 - 70));
//        add(new PointDouble(32.3 + 0.5, 95 - 70));
//        add(new PointDouble(104.7 + 0.5, 95 - 70));
//        add(new PointDouble(128.6 + 0.5, 95 - 70));
//    }};
//
//    PointDouble closestPoint = new PointDouble(0, 0);
//    private int closestPointIndex = 0;
//
    private boolean autoDriveToDroneLaunch = false;
//    //private PointDouble autoDriveToDroneLaunchPosition = new PointDouble();
//
//    private double launchStartX = 0;
//    private double launchStartY = 0;
//
//    public enum Alliance {
//        RED,
//        BLUE
//    }
//
//    public Alliance alliance;

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


        if (!dropPixelAutomation && !autoPilotEnabled && !atBackdrop && !past90Post && !gamepad1.a && !autoDriveToDroneLaunch) {  // this is the mechanum robot centeric control does this make it so can't drive when in auto??
            movement_y = -gamepad1.left_stick_y;
            movement_x = gamepad1.left_stick_x;
            movement_turn = -gamepad1.right_stick_x;
        }

        drive.applyMovementDirectionBased(); //this applys movement useing the mecanumDrive class


            // intake stuff
            if (ButtonPress.isGamepad1_right_bumper_pressed()) {  //intake toggle
                if (intakeOn) {
                    intakeJawServo.intakeJawServo.setPosition(1);
                    intake.vexIntake.setPower(0);
                    intakeOn = false;

                } else {
                    intakeJawServo.intakeJawServo.setPosition(.75);
                    intake.vexIntake.setPower(-.9);
                    intakeOn = true;

                }
            }
            else if (ButtonPress.isGamepad1_left_bumper_pressed()) {  //outtake toggle
                if (outtakeOn) {
                    intakeJawServo.intakeJawServo.setPosition(.7);
                    intake.vexIntake.setPower(0);
                    outtakeOn = false;

                } else {
                    intakeJawServo.intakeJawServo.setPosition(.7);
                    intake.vexIntake.setPower(.9);
                    outtakeOn = true;
                }
            }
    }
}






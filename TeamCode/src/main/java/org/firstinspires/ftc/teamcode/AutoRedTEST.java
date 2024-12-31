package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.MILLIAMPS;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.HashMap;

@Autonomous
public class AutoRedTEST extends AutoMaster {

    private final double SCALE_FACTOR = 1.6;

    private long startTime = 0;

    private int cycle = 0;

    public enum progStates {
        driveForward,
        placePixel,
        //backAwayFromPixel,
        driveToCentralizedPosition,
        turnToStack,
        pickupFirstPixel,
        wait,
        wait2,

        driveToBackdrop,
        dropPixel,
        driveToPixels,

        reapproachToStack,


        endBehavior
    }

    private long prevTimeStamp = 0;

    private boolean intakeCurrentToHighPrev = false;
    private boolean runIntakeCurrentToHighAutomation = false;
    private long intakeCurrentToHighStartTime = 0;
    private int pixelsCounted = 0;
    private boolean prevIntakeBeamBreakState = false;

    private boolean turnOnJamAlgorithm = false;

    boolean justDidAReapproach = false;


    @Override
    public void init() {
        super.init();


        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));



    }

    private int timeDelay = 0;

    @Override
    public void init_loop() {
        super.init_loop();

        if (ButtonPress.isGamepad1_right_bumper_pressed()) {
            timeDelay += 1;
        } else if (ButtonPress.isGamepad1_left_bumper_pressed()) {
            timeDelay -= 1;
        }


        telemetry.addData("Delay", timeDelay);
    }

    @Override
    public void start() {
        super.start();




        startTime = SystemClock.uptimeMillis();

        //pixelDropLocation = Vision.getCubeLocation();

        drive.pose = new Pose2d(0, 0, 0);


    }

    private int pixelDropLocation = 0;

    private HashMap<Integer, PointDouble> purpleDrop = new HashMap<Integer, PointDouble>() {{
        put(0, new PointDouble(105, 26));
        put(1, new PointDouble(103.24, 35));
        put(2, new PointDouble(109.62, 36.8));
    }};


    private boolean hasGrabbedPixels = false;

    private double cutOffTime = 22.5;

    @Override
    public void mainLoop() {
        telemetry.addData("pixels", pixelsCounted);
        boolean jamDetected = false;//pixelJamAndCounting();

        if (programStage == progStates.driveForward.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(150, stateStartingY,
                    0.8 * SCALE_FACTOR, 0.8 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            drive.applyMovementDirectionBased();

            if (Movement.followCurve(points, Math.toRadians(-90), 4)) {
                drive.stopAllMovementDirectionBased();
                nextStage();
            }
        }

//        if (programStage == progStates.placePixel.ordinal()) {
//            if (stageFinished) {
//                initializeStateVariables();
//            }
//
//            ArrayList<CurvePoint> points = new ArrayList<>();
//            points.add(new CurvePoint(stateStartingX, stateStartingY,
//                    0, 0, 0, 0, 0, 0));
//
//            points.add(new CurvePoint(120, 120,
//                    0.8 * SCALE_FACTOR, 0.8 * SCALE_FACTOR, 15, 15,
//                    Math.toRadians(60), 0.6));
//
//
//            //points.add(new CurvePoint(purpleDrop.get(pixelDropLocation).x, purpleDrop.get(pixelDropLocation).y,
//              //      0.9 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 15, 15,
//                //    Math.toRadians(60), 0.6));
//
//            //Movement.movementResult r = Movement.pointAngle(Math.toRadians(purpleDropWantedHeadings.get(pixelDropLocation)), 1, Math.toRadians(30));
//
//            drive.applyMovementDirectionBased();
//
//            if (Movement.followCurve(points, Math.toRadians(-90), 4)) {
//                drive.stopAllMovementDirectionBased();
//                wrist.intakeTilt.setPosition(0.50);
//                nextStage();
//            }
//        }

/*
        if (programStage == progStates.driveToCentralizedPosition.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }

            if (SystemClock.uptimeMillis() - stateStartTime < 500) {
                movement_y = 0.4;
                drive.applyMovementDirectionBased();
            } else {
                ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(stateStartingX, stateStartingY,
                        0, 0, 0, 0, 0, 0));

                points.add(new CurvePoint(118, 30,
                        0.9 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 15, 15,
                        Math.toRadians(60), 0.6));

                drive.applyMovementDirectionBased();

                if (Movement.followCurve(points, Math.toRadians(90), 3)) {
                    drive.stopAllMovementDirectionBased();
                    nextStage();
                }
            }
        }
*/
/*
        if (programStage == progStates.turnToStack.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
               wrist.setIntakeTiltPosition(0.5);
            }

            Movement.movementResult r = Movement.pointAngle(Math.atan2(5 - stateStartingY, 105.5 - stateStartingX), 1, Math.toRadians(30));

            drive.applyMovementDirectionBased();

            if (Math.abs(r.turnDelta_rad) < Math.toRadians(4)) {
                drive.stopAllMovementDirectionBased();

                nextStage();
            }
        }

 */
/*
        if (programStage == progStates.pickupFirstPixel.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();

                wrist.setIntakeTiltPosition(0.5);

                if (!jamDetected) {
                    intake.vexIntake.setPower(-.91);
                }
                //turnOnJamAlgorithm = true;
                intake.vexIntake.setPower(.91);
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(104, 12.5,
                    0.85 * SCALE_FACTOR, 0.85 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            if (worldYPosition < 24) {
                Movement.movementResult r = Movement.pointAngle(Math.toRadians(-90), 0.75, Math.toRadians(20));
            }

            drive.applyMovementDirectionBased();

            if (Movement.followCurve(points, Math.toRadians(90), 1.5) || SystemClock.uptimeMillis() - stateStartTime > 2500) {
                drive.stopAllMovementDirectionBased();
                nextStage();
            }
        }
*/
 /*
        if (programStage == progStates.wait.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
                drive.stopAllMovementDirectionBased();
            }


            movement_y = 0.175;


            drive.applyMovementDirectionBased();

            boolean gotDaPixel = false;

            if ((cycle == 0 && pixelsCounted >= 1) || (cycle >= 1 && pixelsCounted >= 2)) {
                gotDaPixel = true;
            }

            if (SystemClock.uptimeMillis() - stateStartTime > 750 || gotDaPixel || pixelsInConveyor() == 2) {
                drive.stopAllMovementDirectionBased();
                if (!gotDaPixel) {
                    if (!jamDetected) {
                        //intake.intake.setPower(-1);
                    }
                }

                wrist.intakeTilt.setPosition(.5);
                nextStage();
            }
        }
*/
 /*
        if (programStage == progStates.wait2.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }

            Movement.movementResult r = Movement.pointAngle(Math.toRadians(-90), 0.7, Math.toRadians(20));

            movement_y = -0.2;
            movement_x = -0.2;

            drive.applyMovementDirectionBased();

            if (SystemClock.uptimeMillis() - stateStartTime > 750) {
                drive.stopAllMovementDirectionBased();
                nextStage();
            }
        }
*/
 /*
        if (programStage == progStates.driveToBackdrop.ordinal()) {
            if (stageFinished) {
                //pixelLift.pControllerPixelLift.setSetPoint(100);

                if (cycle <= 1) {
                    wrist.intakeTilt.setPosition(.5);
                }

                intake.vexIntake.setPower(1);

                hasGrabbedPixels = false;

                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));
1

            points.add(new CurvePoint(128.25, 25,
                    0.8 * SCALE_FACTOR, 0.8 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            points.add(new CurvePoint(128.25, 83,
                    0.8 * SCALE_FACTOR, 0.8 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));



            if (worldYPosition > 20 && worldYPosition < 25) {
                intake.vexIntake.setPower(-.91);
            }

            if (worldYPosition > 35 && worldYPosition < 45) {
                //fourBars.setFourBarPosition(1);
                intake.vexIntake.setPower(1);
            }

        }
        */
/*
        if (programStage == progStates.dropPixel.ordinal()) {
            if (stageFinished) {
                cycle = cycle + 1;
                initializeStateVariables();
            }

            armPivot.updateLiftPosition();

            Movement.movementResult r = Movement.pointAngle(Math.toRadians(-90), 0.2, Math.toRadians(20));


            double timeDropPixel = cycle == 1 ? 350 : 250;



            double timeDropDriveBack = cycle == 1 ? 750 : 650;
            double timeDropDriveForward = cycle == 1 ? 950 : 850;



            drive.applyMovementDirectionBased();
        }

        */


/*
        if (programStage == progStates.driveToPixels.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
                pixelsCounted = 0;
                pixelTwister.setPixelTwisterPosition(.49);
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(128.25, 95,
                    0.8 * SCALE_FACTOR, 0.8 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            points.add(new CurvePoint(128.25, 35,
                    0.8 * SCALE_FACTOR, 0.8 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            points.add(new CurvePoint(103, 17,
                    0.8 * SCALE_FACTOR, 0.8 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            points.add(new CurvePoint(103, 12.5,
                    0.3 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            if (Movement.followCurve(points, Math.toRadians(90), 2)) {
                drive.stopAllMovementDirectionBased();

                nextStage(progStates.wait.ordinal());
            }

            drive.applyMovementDirectionBased();
        }
        */

        telemetry.addData("world x", worldXPosition);
        telemetry.addData("world y", worldYPosition);
        telemetry.addData("world ang", worldAngle_rad);


        telemetry.addData("cycle", cycle);

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
    }
}


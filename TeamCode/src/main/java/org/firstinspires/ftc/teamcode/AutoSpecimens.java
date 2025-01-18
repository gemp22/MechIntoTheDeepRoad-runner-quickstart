package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;


import static org.firstinspires.ftc.teamcode.RobotPosition.AngleWrap;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Superstructure;

import java.util.ArrayList;
import java.util.HashMap;

@Autonomous
public class AutoSpecimens extends Robot {

    private final double SCALE_FACTOR = 1.6;

    private long startTime = 0;

    private int cycle = 0;

    public enum progStates {

        driveBackwardToChamberAndHang,
        hangSpecimen,
        driveToPlayerStation,
        grabSpecimen,

        driveToChamber,



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
        //drive.pose = new Pose2d(0, 0, 0);
        GoBildaOdo.setRobotPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, Math.PI));
    }

    private int pixelDropLocation = 0;

    private HashMap<Integer, PointDouble> purpleDrop = new HashMap<Integer, PointDouble>() {{
        put(0, new PointDouble(105, 26));
        put(1, new PointDouble(103.24, 35));
        put(2, new PointDouble(109.62, 36.8));
    }};


    private boolean hasGrabbedPixels = false;

    private double cutOffTime = 22.5;
    private int currentState = AutoSpecimens.programStage;

    private boolean past5In = false;

    @Override
    public void mainLoop() {
        telemetry.addData("pixels", pixelsCounted);
        boolean jamDetected = false;//pixelJamAndCounting();
        telemetry.addData("Superstructure State", currentState);
        System.out.println("Superstructure State: " + currentState);

        if (programStage == progStates.driveBackwardToChamberAndHang.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();

            }
            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(20, 0,
                    0.3 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            points.add(new CurvePoint(27.4, 0,
                    0.15 * SCALE_FACTOR, 0.15 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            if (Math.abs(Math.hypot(worldXPosition - 10, worldYPosition - 0)) < 5) {
                if (!past5In) {
                    superstructure.nextState(Superstructure.SuperstructureStates.SPECIMEN_HANG_PREP.ordinal());
                    past5In = true;
                }
            }

            if (Movement.followCurve(points, Math.toRadians(-90))) {
                drive.stopAllMovementDirectionBased();
                nextStage(progStates.hangSpecimen.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state
        }


        if (programStage == progStates.hangSpecimen.ordinal()) {
            if (stageFinished) {
                superstructure.nextState(Superstructure.SuperstructureStates.SPECIMEN_HANG_CHAMBER.ordinal());
                initializeStateVariables();
            }

            if (SystemClock.uptimeMillis()-stateStartTime > 500) {
                ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(stateStartingX, stateStartingY,
                        0, 0, 0, 0, 0, 0));

                points.add(new CurvePoint(stateStartingX-4, stateStartingY,
                        0.3 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 15,
                        Math.toRadians(60), 0.6));


                if (Movement.followCurve(points, Math.toRadians(90))) {
                    drive.stopAllMovementDirectionBased();

                    nextStage(progStates.driveToPlayerStation.ordinal());
                }

                drive.applyMovementDirectionBased();
            } else {
                drive.stopAllMovementDirectionBased();
            }
        }

        if (programStage == progStates.driveToPlayerStation.ordinal()) {
            if (stageFinished) {
                past5In = false;
                superstructure.nextState(Superstructure.SuperstructureStates.RESTING.ordinal());

                System.out.println("Starting X" + stateStartingX);
                System.out.println("Starting Y" + stateStartingY);
                initializeStateVariables();
            }
            if (SystemClock.uptimeMillis()-stateStartTime > 100) {
                ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(stateStartingX, stateStartingY,
                        0, 0, 0, 0, 0, 0));

                points.add(new CurvePoint(26, -45,
                        0.7 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 15, 15,
                        Math.toRadians(60), 0.6));

                /*points.add(new CurvePoint(15, -45,
                        0.7 * SCALE_FACTOR, 0.7 * SCALE_FACTOR, 15, 15,
                        Math.toRadians(60), 0.6));*/

                points.add(new CurvePoint(10, -45,
                        0.3 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 15, 10,
                        Math.toRadians(60), 0.6));

                /*
                *
                * points.add(new CurvePoint(25, -45,
                        0.8 * SCALE_FACTOR, 0.8 * SCALE_FACTOR, 20, 20,
                        Math.toRadians(60), 0.6));

                points.add(new CurvePoint(10, -45,
                        0.7 * SCALE_FACTOR, 0.7 * SCALE_FACTOR, 15, 15,
                        Math.toRadians(60), 0.6));
                * */

//                if (worldYPosition<-30) {
//                    if (!past5In) {
//                        superstructure.nextState(Superstructure.SuperstructureStates.COLLECT_SPECIMEN_PREP.ordinal());
//                        past5In = true;
//                    }
//                }

                double relativePointAngle = AngleWrap(Math.toRadians(180) - worldAngle_rad);

                if (Movement.followCurve(points, Math.toRadians(90)) && Math.abs(relativePointAngle) < Math.toRadians(4)) {
                    drive.stopAllMovementDirectionBased();

                    nextStage(progStates.grabSpecimen.ordinal());
                    //nextStage(progStates.endBehavior.ordinal());
                }

                drive.applyMovementDirectionBased();
            }
        }

        if (programStage == progStates.grabSpecimen.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }
            if (SystemClock.uptimeMillis()-stateStartTime > 0) {

                if (!past5In) {
                    superstructure.nextState(Superstructure.SuperstructureStates.COLLECT_SPECIMEN_PREP.ordinal());
                    past5In = true;
                }

                /*ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(10, -45,
                        0, 0, 0, 0, 0, 0));

                points.add(new CurvePoint(6, -45,
                        0.3 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 10, 10,
                        Math.toRadians(60), 0.6));*/

                movement_y = 0.25;


                if (Math.abs(Math.hypot(worldXPosition - 10, worldYPosition - (-45))) < 1) {
                    if (!past5In) {
                        //superstructure.nextState(Superstructure.SuperstructureStates.COLLECT_SPECIMEN_WALL.ordinal());
                        past5In = true;
                    }
                }

                if (SystemClock.uptimeMillis()-stateStartTime>1000) {
                    superstructure.nextState(Superstructure.SuperstructureStates.COLLECT_SPECIMEN_WALL.ordinal());
                    drive.stopAllMovementDirectionBased();
                    nextStage(progStates.driveToChamber.ordinal());
                }

                drive.applyMovementDirectionBased();
            }
        }

        if (programStage == progStates.driveToChamber.ordinal()) {
            if (stageFinished) {
                past5In = false;
                //superstructure.nextState(Superstructure.SuperstructureStates.SPECIMEN_TRANSPORT.ordinal());

                initializeStateVariables();
            }
            if (SystemClock.uptimeMillis()-stateStartTime > 700) {
                ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(stateStartingX, stateStartingY,
                        0, 0, 0, 0, 0, 0));


                points.add(new CurvePoint(15, -45,
                        0.7 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 10, 10,
                        Math.toRadians(60), 0.6));

                points.add(new CurvePoint(18, -5,
                        0.7 * SCALE_FACTOR, 0.9 * SCALE_FACTOR, 10, 10,
                        Math.toRadians(60), 0.6));

                points.add(new CurvePoint(27.2, -5,
                        0.15 * SCALE_FACTOR, 0.15 * SCALE_FACTOR, 10, 10,
                        Math.toRadians(60), 0.6));


                /*if (Math.abs(Math.hypot(worldXPosition - 25, worldYPosition - (-5))) < 5) {
                    if (!past5In) {
                        superstructure.nextState(Superstructure.SuperstructureStates.SPECIMEN_HANG_CHAMBER.ordinal());
                        past5In = true;
                    }
                }*/

                double relativePointAngle = AngleWrap(Math.toRadians(180) - worldAngle_rad);

                if (Movement.followCurve(points, Math.toRadians(-90)) && Math.abs(relativePointAngle) < Math.toRadians(4)) {
                    drive.stopAllMovementDirectionBased();

                    nextStage(progStates.endBehavior.ordinal());
                    //nextStage(progStates.endBehavior.ordinal());
                }

                drive.applyMovementDirectionBased();
            }
        }

        if (programStage == progStates.endBehavior.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }
            drive.stopAllMovementDirectionBased();

        }

        superstructure.update(telemetry, gamepad1, gamepad2);
    }
}


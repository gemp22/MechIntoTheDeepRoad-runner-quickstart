package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_turn;


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
public class AutoRedTEST extends Robot {

    private final double SCALE_FACTOR = 1.6;

    private long startTime = 0;

    private int cycle = 0;

    public enum progStates {
        driveForward,



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
    private int currentState = AutoRedTEST.programStage;

    private boolean past5In = false;

    @Override
    public void mainLoop() {
        telemetry.addData("pixels", pixelsCounted);
        boolean jamDetected = false;//pixelJamAndCounting();
        telemetry.addData("Superstructure State", currentState);
        System.out.println("Superstructure State: " + currentState);

        if (programStage == progStates.driveForward.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(stateStartingX+20, stateStartingY,
                    0.3 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            points.add(new CurvePoint(stateStartingX+27.3, stateStartingY,
                    0.15 * SCALE_FACTOR, 0.15 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            if (Math.abs(Math.hypot(worldXPosition - 10, worldYPosition - 0)) < 5) {
                if (!past5In) {
                    superstructure.nextState(Superstructure.SuperstructureStates.SPECIMEN_HANG_PREP.ordinal());
                    past5In = true;
                }
            }

            drive.applyMovementDirectionBased();

            if (Movement.followCurve(points, Math.toRadians(-90))) {
                drive.stopAllMovementDirectionBased();
                nextStage(progStates.endBehavior.ordinal());
            }

        }

        if (programStage == progStates.endBehavior.ordinal()) {
            if (stageFinished) {
                superstructure.nextState(Superstructure.SuperstructureStates.SPECIMEN_HANG_CHAMBER.ordinal());
                initializeStateVariables();
            }

            if (SystemClock.uptimeMillis()-stateStartTime > 2000) {
                ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(stateStartingX, stateStartingY,
                        0, 0, 0, 0, 0, 0));

                points.add(new CurvePoint(stateStartingX-10, stateStartingY,
                        0.3 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 15, 15,
                        Math.toRadians(60), 0.6));

                drive.applyMovementDirectionBased();

                if (Movement.followCurve(points, Math.toRadians(90))) {
                    drive.stopAllMovementDirectionBased();
                }
            } else {
                drive.stopAllMovementDirectionBased();
            }
        }

        superstructure.update(telemetry, gamepad1, gamepad2);
    }
}


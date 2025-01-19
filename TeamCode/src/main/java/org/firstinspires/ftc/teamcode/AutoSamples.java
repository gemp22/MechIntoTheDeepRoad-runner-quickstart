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
public class AutoSamples extends Robot {

    private final double SCALE_FACTOR = 1.6;

    private long startTime = 0;

    private int cycle = 0;

    public enum progStates {

        driveToBaskets,
        driveToOurSamples,
        deliverBaskets,
        driveToOpponentsSamples,
        driveBackToBaskets,
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
        armPivot.intakeTilt.setPosition(Constants.TILT_SERVO_90_DEGREES_UP);
        armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_GRAB_POSITION);
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
        GoBildaOdo.setRobotPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS,0));
    }

    private int pixelDropLocation = 0;

    private HashMap<Integer, PointDouble> purpleDrop = new HashMap<Integer, PointDouble>() {{
        put(0, new PointDouble(105, 26));
        put(1, new PointDouble(103.24, 35));
        put(2, new PointDouble(109.62, 36.8));
    }};


    private boolean hasGrabbedPixels = false;

    private double cutOffTime = 22.5;
    private int currentState = AutoSamples.programStage;

    private boolean past5In = false;

    @Override
    public void mainLoop() {
        telemetry.addData("pixels", pixelsCounted);
        boolean jamDetected = false;//pixelJamAndCounting();
        telemetry.addData("Superstructure State", currentState);
        System.out.println("Superstructure State: " + currentState);

        if (programStage == progStates.driveToBaskets.ordinal()) {
            if (stageFinished) {
                past5In = false;
                superstructure.nextState(Superstructure.SuperstructureStates.DELIVERY_LEVEL_2.ordinal());
                initializeStateVariables();
            }
            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(13, 13,
                    0.6 * SCALE_FACTOR, 0.6 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            if (Math.abs(Math.hypot(worldXPosition - 13, worldYPosition - 13)) < 5) {
                if (!past5In) {

                    past5In = true;
                }
            }
            boolean completed = Movement.followCurve(points, Math.toRadians(90), 1.5);

            if (worldYPosition > 5) {
                Movement.movementResult r = Movement.pointAngle(Math.toRadians(-45), 0.7, Math.toRadians(20));

                if (completed && Math.abs(r.turnDelta_rad) < Math.toRadians(3)) {
                    drive.stopAllMovementDirectionBased();

                    if(lift.getLiftExtension()>23){
                        superstructure.nextState(Superstructure.SuperstructureStates.DELIVERY_SAMPLE_DROP.ordinal());

                            nextStage(progStates.driveToOurSamples.ordinal());

                    }
                }
            }
            drive.applyMovementDirectionBased(); // always put at end of state
        }

        if (programStage == progStates.driveToOurSamples.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }
            /*ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(23, 15,
                    0.6 * SCALE_FACTOR, 0.6 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            if (Math.abs(Math.hypot(worldXPosition - 23, worldYPosition - 15)) < 5) {
                if (!past5In) {
                    //superstructure.nextState(Superstructure.SuperstructureStates.DELIVERY_LEVEL_2.ordinal());
                    past5In = true;
                }
            }
            boolean completed = Movement.followCurve(points, Math.toRadians(90), 1.5);

            if (worldYPosition > 5) {
                Movement.movementResult r = Movement.pointAngle(Math.toRadians(0), 0.7, Math.toRadians(20));

                if (completed && Math.abs(r.turnDelta_rad) < Math.toRadians(3)) {
                    drive.stopAllMovementDirectionBased();

                    if(lift.getLiftExtension()>23){
                        superstructure.nextState(Superstructure.SuperstructureStates.DELIVERY_SAMPLE_DROP.ordinal());
                        nextStage(progStates.endBehavior.ordinal());
                    }
                }
            }*/
            if (SystemClock.uptimeMillis()-stateStartTime > 1500) {
                if (!past5In) {
                    superstructure.nextState(Superstructure.SuperstructureStates.RESTING.ordinal());
                    past5In = true;
                }

                Movement.movementResult r = Movement.pointAngle(
                        Math.atan2(11 - stateStartingY, 36 - stateStartingX),
                        0.7,
                        Math.toRadians(30));

                if (Math.abs(r.turnDelta_rad) < Math.toRadians(3)) {
                    nextStage(progStates.endBehavior.ordinal());
                }

                drive.applyMovementDirectionBased(); // always put at end of state
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


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
        park,
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

    private boolean past5In = false;

    @Override
    public void mainLoop() {
        telemetry.addData("pixels", pixelsCounted);

        if (programStage == progStates.driveToBaskets.ordinal()) {
            if (stageFinished) {
                past5In = false;
                superstructure.nextState(Superstructure.SuperstructureStates.DELIVERY_LEVEL_2.ordinal());
                initializeStateVariables();
            }
            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(12.5, 12.0,
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

                    System.out.println("lift extension in delivery state: " + lift.getLiftExtension());

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

                System.out.println("world X" + worldXPosition);
                System.out.println("world y" + worldYPosition);
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
            if (SystemClock.uptimeMillis()-stateStartTime > 500) {
                if (!past5In) {
                    superstructure.nextState(Superstructure.SuperstructureStates.RESTING.ordinal());
                    past5In = true;
                }

                ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(stateStartingX, stateStartingY,
                        0, 0, 0, 0, 0, 0));

                points.add(new CurvePoint(18, 18,
                        0.6 * SCALE_FACTOR, 0.6 * SCALE_FACTOR, 15, 15,
                        Math.toRadians(60), 0.6));

                boolean completed = Movement.followCurve(points, Math.toRadians(90), 1.5);

                double pickupXPosition = (cycle * 11.5) + 4;
                Movement.movementResult r = Movement.pointAngle(
                        Math.atan2(pickupXPosition - stateStartingY, 34 - stateStartingX),
                        0.7,
                        Math.toRadians(30));

                if (completed &&
                        Math.abs(r.turnDelta_rad) < Math.toRadians(3) &&
                        lift.getLiftExtension()<1 && armPivot.getArmAngle()<-3) {
                    superstructure.sampleCollected = false;
                    if(cycle<3) {
                        superstructure.nextState(Superstructure.SuperstructureStates.SAMPLE_COLLECTION_EXTENSTION.ordinal());
                    }
                    nextStage(progStates.endBehavior.ordinal());
                }

                drive.applyMovementDirectionBased(); // always put at end of state
            }
        }

        if (programStage == progStates.endBehavior.ordinal()) {
            if (stageFinished) {
                past5In = false;
                cycle++;
                initializeStateVariables();
            }
            if (superstructure.sampleCollected && lift.getLiftExtension() < 1 ) {
                nextStage(progStates.driveToBaskets.ordinal());
            }
            drive.stopAllMovementDirectionBased();

        }

        if (programStage == progStates.park.ordinal()) {
            if (stageFinished) {
                past5In = false;
                cycle++;
                initializeStateVariables();
            }
            if (superstructure.sampleCollected && lift.getLiftExtension() < 1) {
                nextStage(progStates.driveToBaskets.ordinal());
            }
            drive.stopAllMovementDirectionBased();

        }

        superstructure.update(telemetry, gamepad1, gamepad2);
    }
}


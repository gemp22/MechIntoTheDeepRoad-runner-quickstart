package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;
import android.util.Pair;

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
public class InsaneWontonPingPongDingDong extends Robot {

    private final double SCALE_FACTOR = 1;

    private long startTime = 0;

    private int cycle = 0;

    public enum progStates {

        driveToSpecDrop,
        driveToOurSamples,
        deliverBaskets,

        pickupRemainingTwo,
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
        Robot.resetEncoders = true;
        super.init();

        isAuto = true;
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

    private HashMap<Integer, Pair<PointDouble, Double>> pickupPoints = new HashMap<Integer, Pair<PointDouble, Double>>() {{
        put(0, new Pair<>(new PointDouble(31.0603, 24.9123), 70.422)); // -31.3774
        put(1, new Pair<>(new PointDouble(31.422,32.45), 74.1515)); // 50.0452
        put(2, new Pair<>(new PointDouble(18, 18), 0.0));
    }};

    private boolean hasGrabbedPixels = false;

    private double cutOffTime = 22.5;

    private boolean past5In = false;

    @Override
    public void mainLoop() {
        telemetry.addData("pixels", pixelsCounted);

        if (programStage == progStates.driveToSpecDrop.ordinal()) {
            if (stageFinished) {
                past5In = false;
                superstructure.nextState(Superstructure.SuperstructureStates.SPECIMEN_HANG_FRONT_PREP_AUTO.ordinal());
                initializeStateVariables();
            }
            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(26, 0,
                    0.6 * SCALE_FACTOR, 0.6 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            boolean completed = Movement.followCurve(points, Math.toRadians(90), 3);

            Movement.movementResult r = Movement.pointAngle(Math.toRadians(-28), 0.7, Math.toRadians(20));

            if (worldXPosition > 20) {
                drive.stopAllMovementDirectionBased();
                superstructure.nextState(Superstructure.SuperstructureStates.SPECIMEN_HANG_FRONT_CHAMBER_AUTO.ordinal());
                nextStage(progStates.driveToOurSamples.ordinal());
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

            if (SystemClock.uptimeMillis()-stateStartTime>750) {
                if (!past5In) {
                    superstructure.nextState(Superstructure.SuperstructureStates.RESTING.ordinal());
                    past5In = true;
                }

                if (SystemClock.uptimeMillis()-stateStartTime>1500) {
                    ArrayList<CurvePoint> points = new ArrayList<>();
                    points.add(new CurvePoint(stateStartingX, stateStartingY,
                            0, 0, 0, 0, 0, 0));

                    Pair<PointDouble, Double> wantedPos = pickupPoints.get(cycle);

                    points.add(new CurvePoint(28, 14.5,
                            0.6 * SCALE_FACTOR, 0.6 * SCALE_FACTOR, 15, 15,
                            Math.toRadians(60), 0.6));

                    boolean completed = Movement.followCurve(points, Math.toRadians(90), 3);

                    Movement.movementResult r = Movement.pointAngle(
                            Math.toRadians(65), //Math.atan2(pickupYPosition - stateStartingY, 34 - stateStartingX),
                            0.7,
                            Math.toRadians(30));

                    if (completed &&
                            Math.abs(r.turnDelta_rad) < Math.toRadians(5)) {
                        drive.stopAllMovementDirectionBased();
                        superstructure.sampleCollected = false;
                        superstructure.nextState(Superstructure.SuperstructureStates.SAMPLE_COLLECTION_EXTENSTION.ordinal());
                        nextStage(progStates.deliverBaskets.ordinal());
                    }

                    drive.applyMovementDirectionBased(); // always put at end of state
                }
            }
        }

        if (programStage == progStates.deliverBaskets.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }
            if (superstructure.sampleCollected) {
                if (!past5In) {
                    superstructure.nextState(Superstructure.SuperstructureStates.DELIVERY_LEVEL_2.ordinal());
                    past5In = true;
                }
                ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(stateStartingX, stateStartingY,
                        0, 0, 0, 0, 0, 0));

                points.add(new CurvePoint(12, 38.2,
                        0.6 * SCALE_FACTOR, 0.6 * SCALE_FACTOR, 15, 15,
                        Math.toRadians(60), 0.6));

                boolean completed = Movement.followCurve(points, Math.toRadians(-90), 3);

                Movement.movementResult r = Movement.pointAngle(
                        Math.toRadians(-45), //Math.atan2(pickupYPosition - stateStartingY, 34 - stateStartingX),
                        0.7,
                        Math.toRadians(30));

                if (completed &&
                        Math.abs(r.turnDelta_rad) < Math.toRadians(5)) {
                    drive.stopAllMovementDirectionBased();

                    if(lift.getLiftExtension()>22){
                        superstructure.nextState(Superstructure.SuperstructureStates.DELIVERY_SAMPLE_DROP.ordinal());

                        nextStage(progStates.pickupRemainingTwo.ordinal());
                    }
                }

                drive.applyMovementDirectionBased();
            }
        }

        if (programStage == progStates.pickupRemainingTwo.ordinal()) {
            if (stageFinished) {
                past5In = false;

                System.out.println("world X" + worldXPosition);
                System.out.println("world y" + worldYPosition);
                initializeStateVariables();
            }
            if (SystemClock.uptimeMillis()-stateStartTime > 650) {
                if (!past5In) {
                    superstructure.nextState(Superstructure.SuperstructureStates.RESTING.ordinal());
                    past5In = true;
                }
                if(SystemClock.uptimeMillis() - stateStartTime > 1250)
                {
                    ArrayList<CurvePoint> points = new ArrayList<>();
                    points.add(new CurvePoint(stateStartingX, stateStartingY,
                            0, 0, 0, 0, 0, 0));

                    Pair<PointDouble, Double> wantedPos = pickupPoints.get(cycle);

                    points.add(new CurvePoint(wantedPos.first.x, wantedPos.first.y,
                            0.6 * SCALE_FACTOR, 0.6 * SCALE_FACTOR, 15, 15,
                            Math.toRadians(60), 0.6));

                    boolean completed = Movement.followCurve(points, Math.toRadians(90), 3);

                    Movement.movementResult r = Movement.pointAngle(
                            Math.toRadians(wantedPos.second), //Math.atan2(pickupYPosition - stateStartingY, 34 - stateStartingX),
                            0.7,
                            Math.toRadians(30));

                    if (completed &&
                            Math.abs(r.turnDelta_rad) < Math.toRadians(5) &&
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
        }

        if (programStage == progStates.endBehavior.ordinal()) {
            if (stageFinished) {
                past5In = false;
                cycle++;
                initializeStateVariables();
            }
            if (superstructure.sampleCollected && lift.getLiftExtension() < 1 ) {
                nextStage(progStates.deliverBaskets.ordinal());
            }
            drive.stopAllMovementDirectionBased();

        }

        telemetry.addData("State", programStage);


        tp4.markStart();

        superstructure.update(telemetry, gamepad1, gamepad2);

        tp4.markEnd();

        System.out.println("Time Profiler 4 Average Time: " + tp4.getAverageTimePerUpdateMillis());

    }
}

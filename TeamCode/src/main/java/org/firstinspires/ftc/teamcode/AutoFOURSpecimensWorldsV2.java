package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.RobotPosition.AngleWrap;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;
import android.util.Pair;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Superstructure3Motor;

import java.util.ArrayList;
import java.util.HashMap;

@Autonomous
@Disabled
public class AutoFOURSpecimensWorldsV2 extends Robot3Motor {

    private final double SCALE_FACTOR = 1;

    private long startTime = 0;

    private int cycle = 0;
    private int driveToGetSampleCycle = 0;

    public enum progStates {

        driveBackwardToChamberAndHang,
        hangSpecimen,
        driveToPlayerStation,
        grabSpecimen,

        driveToChamber,
        driveToOurSamples,
        driveUpToSamples,
        strafeToSamples,
        pushSamplesToPlayerStation,
        pushSampleToPickUpSpecimen,
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

    boolean hasPushSamplesIn = false;
    private double currentServoPosition = 0;


    @Override
    public void init() {
        Robot3Motor.resetEncoders = true;
        super.init();

        isAuto = true;

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        armPivot3Motor.intakeTilt.setPosition(Constants3Motor.TILT_SERVO_90_DEGREES_UP);
        armPivot3Motor.intakeTiltTwo.setPosition(Constants3Motor.TILT_SERVO_90_DEGREES_UP);
        armPivot3Motor.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_GRAB_POSITION);
        armPivot3Motor.twist.setPosition(Constants3Motor.TWIST_SERVO_HORIZONTAL_POSITION);
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

        if (ButtonPress.isGamepad1_a_pressed()) {
            armPivot3Motor.intakeTilt.setPosition(currentServoPosition+= 0.005);
            armPivot3Motor.intakeTiltTwo.setPosition(currentServoPosition+= 0.005); ;
        } else if (ButtonPress.isGamepad1_b_pressed()) {
            armPivot3Motor.intakeTilt.setPosition(currentServoPosition-= 0.005);
            armPivot3Motor.intakeTiltTwo.setPosition(currentServoPosition-= 0.005);
        }

        if (ButtonPress.isGamepad1_x_pressed()) {
            armPivot3Motor.intakeJawServo.setPosition(currentServoPosition+= 0.005);

        } else if (ButtonPress.isGamepad1_y_pressed()) {
            armPivot3Motor.intakeJawServo.setPosition(currentServoPosition-= 0.005);

        }

        telemetry.addData("tilt servo 2 position", armPivot3Motor.intakeTiltTwo.getPosition());
        telemetry.addData("tilt servo 1 position", armPivot3Motor.intakeTilt.getPosition());
        telemetry.addData("Jaw position", armPivot3Motor.intakeJawServo.getPosition());

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

    private HashMap<Integer, Pair<PointDouble, Double>> pickUpPoints = new HashMap<Integer, Pair<PointDouble, Double>>() {{
        put(0, new Pair<>(new PointDouble(25.5, -9), 55.0)); // -31.3774
        put(1, new Pair<>(new PointDouble(25.1792,2.5118), 56.5)); // 50.0452
        put(2, new Pair<>(new PointDouble(24.9231, 10.9599), 57.5)); //55.3688
        put(3, new Pair<>(new PointDouble(18, 18), 0.0));
    }};

    private boolean hasGrabbedPixels = false;

    private double cutOffTime = 22.5;
    private int currentState = AutoFOURSpecimensWorldsV2.programStage;

    private boolean past5In = false;

    public static boolean pickupOffWall = false;

    public int overallCycleToChamber = 0;

    @Override
    public void mainLoop() {

        boolean jamDetected = false;//pixelJamAndCounting();
        telemetry.addData("Superstructure State", currentState);
        System.out.println("Superstructure State: " + currentState);

        if (programStage == progStates.driveBackwardToChamberAndHang.ordinal()) {
            if (stageFinished) {
                past5In = false;
                superstructure.nextState(Superstructure3Motor.SuperstructureStates.SPECIMEN_HANG_PREP.ordinal());
                initializeStateVariables();
            }

            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(15, 0,
                    0.4 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 20, 10, // changed move speed from .35 to .45
                    Math.toRadians(60), 0.6));

            points.add(new CurvePoint(25.75, 0,
                    0.3 * SCALE_FACTOR, 0.2 * SCALE_FACTOR, 20, 10, // changed move speed from .2 to .25
                    Math.toRadians(60), 0.6));

            if (Movement.followCurve(points, Math.toRadians(-90))) {
                drive.stopAllMovementDirectionBased();
                nextStage(progStates.hangSpecimen.ordinal());
            }

            drive.applyMovementDirectionBased(); // always put at end of state
        }

        if (programStage == progStates.hangSpecimen.ordinal()) {
            if (stageFinished) {
                superstructure.nextState(Superstructure3Motor.SuperstructureStates.SPECIMEN_HANG_CHAMBER_AUTO.ordinal());
                drive.stopAllMovementDirectionBased();
                initializeStateVariables();
            }

            if (SystemClock.uptimeMillis()-stateStartTime > 400) { // chaged from 750 to speed up
                ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(stateStartingX, stateStartingY,
                        0, 0, 0, 0, 0, 0));

                points.add(new CurvePoint(stateStartingX-8, stateStartingY,
                        0.5 * SCALE_FACTOR, 0.4 * SCALE_FACTOR, 15, 15, // changed move speed from .4 to .5
                        Math.toRadians(60), 0.6));

                if (Movement.followCurve(points, Math.toRadians(90), 4)) { //Change tol from 3 to 4
                    drive.stopAllMovementDirectionBased();

                    if (overallCycleToChamber == 3) { //changed from 2 to 3 to get one more cycle
                        superstructure.nextState(Superstructure3Motor.SuperstructureStates.GOTO_RESTING_WORLDS.ordinal());
                        nextStage(progStates.endBehavior.ordinal());
                    } else {
                        nextStage(progStates.driveToPlayerStation.ordinal());
                    }
                }

                drive.applyMovementDirectionBased();
            } else {
                movement_y = -0.25;  // this makes robot drive toward submersible until time above is met
                drive.applyMovementDirectionBased();
            }
        }

        if (programStage == progStates.driveToPlayerStation.ordinal()) {
            if (stageFinished) {
                past5In = false;
                if (cycle == 1) {
                    superstructure.nextState(Superstructure3Motor.SuperstructureStates.GOTO_RESTING_WORLDS.ordinal());
                } else {
                    superstructure.nextState(Superstructure3Motor.SuperstructureStates.COLLECT_SPECIMEN_PREP.ordinal());
                }

                System.out.println("Starting X" + stateStartingX);
                System.out.println("Starting Y" + stateStartingY);
                initializeStateVariables();
            }
            if (SystemClock.uptimeMillis()-stateStartTime > 100) {
                ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(stateStartingX, stateStartingY,
                        0, 0, 0, 0, 0, 0));

                double wantedX = cycle == 1 ? 26 : 15;

                points.add(new CurvePoint(wantedX, -45,
                        1 * SCALE_FACTOR, 1 * SCALE_FACTOR, 15, 15,
                        Math.toRadians(60), 0.6));

                boolean completed = Movement.followCurve(points, Math.toRadians(90), 3); // changed tol from 2 to 3

                double relativePointAngle = AngleWrap(Math.toRadians(180) - worldAngle_rad);

                if (worldYPosition < -15 && cycle == 1) {
                    Movement.movementResult r = Movement.pointAngle(
                            Math.toRadians(180),
                            1,
                            Math.toRadians(30));
                } else  if (worldYPosition < -25) {
                    Movement.movementResult r = Movement.pointAngle(
                            Math.toRadians(180),
                            1,
                            Math.toRadians(30));
                }

                if (completed && Math.abs(relativePointAngle) < Math.toRadians(4)) {
                    drive.stopAllMovementDirectionBased();
                    cycle++;
                    //superstructure.nextState(Superstructure.SuperstructureStates.COLLECT_SPECIMEN_PREP.ordinal());

                    if(cycle<3) { // changed from 2 to 3
                        nextStage(progStates.grabSpecimen.ordinal());
                    }else {
                        nextStage(progStates.driveToOurSamples.ordinal());
                    }
                }

                drive.applyMovementDirectionBased();
            }
        }

        if (programStage == progStates.grabSpecimen.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }
            if (SystemClock.uptimeMillis()-stateStartTime > 200) { // changed from 250 to 200

                if (!past5In) {
                    //superstructure.nextState(Superstructure.SuperstructureStates.COLLECT_SPECIMEN_PREP.ordinal());
                    past5In = true;
                }

                movement_y = 0.20;

                if (Math.abs(Math.hypot(worldXPosition - 10, worldYPosition - (-45))) < 1) {
                    if (!past5In) {
                        //superstructure.nextState(Superstructure.SuperstructureStates.COLLECT_SPECIMEN_WALL.ordinal());
                        past5In = true;
                    }
                }

                if (SystemClock.uptimeMillis()-stateStartTime>1000) { // 2050
                    pickupOffWall = false;
                    superstructure.nextState(Superstructure3Motor.SuperstructureStates.COLLECT_SPECIMEN_WALL.ordinal());
                    drive.stopAllMovementDirectionBased();
                    nextStage(progStates.driveToChamber.ordinal());
                }

                drive.applyMovementDirectionBased();
            }
        }

        if (programStage == progStates.driveToChamber.ordinal()) {
            if (stageFinished) {
                past5In = false;
                armPivot3Motor.intakeJawServo.setPosition(Constants3Motor.JAW_SERVO_GRAB_POSITION);
                //superstructure.nextState(Superstructure.SuperstructureStates.SPECIMEN_TRANSPORT.ordinal());
                initializeStateVariables();
            }
            if (SystemClock.uptimeMillis()-stateStartTime > 300) { // was 400
                pickupOffWall = true;
            }
            if (SystemClock.uptimeMillis()-stateStartTime > 400) { // was 500

                ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(stateStartingX, stateStartingY,
                        0, 0, 0, 0, 0, 0));


                double destinationX = Math.abs(Math.hypot(worldXPosition - 29, worldYPosition - (-7 + (overallCycleToChamber * -7)))) < 10 ? 28 : 20;

                points.add(new CurvePoint(destinationX, -7 + (overallCycleToChamber * -7),
                        1 * SCALE_FACTOR, 1 * SCALE_FACTOR, 15, 10,
                        Math.toRadians(60), 0.6));

                double relativePointAngle = AngleWrap(Math.toRadians(180) - worldAngle_rad);

                if (Movement.followCurve(points, Math.toRadians(-90)) && Math.abs(relativePointAngle) < Math.toRadians(4)) {
                    drive.stopAllMovementDirectionBased();
                    overallCycleToChamber++;

                    System.out.println("WorldPosXHang: " + worldXPosition);
                    System.out.println("WorldPosYHang: " + worldYPosition);

                    nextStage(progStates.hangSpecimen.ordinal());

                    //nextStage(progStates.endBehavior.ordinal());
                }

                if (Math.abs(Math.hypot(worldXPosition - 29, worldYPosition - (-7 + (overallCycleToChamber * -7)))) < 28) {
                    if (!past5In) {
                        superstructure.nextState(Superstructure3Motor.SuperstructureStates.SPECIMEN_HANG_PREP.ordinal());
                        past5In = true;
                    }

                    Movement.movementResult r = Movement.pointAngle(
                            Math.toRadians(180),
                            1,
                            Math.toRadians(30));
                }
                drive.applyMovementDirectionBased();
            }
        }

        if (programStage == AutoSamplesWorlds.progStates.driveToOurSamples.ordinal()) {  //copied from sample auto
            if (stageFinished) {
                past5In = false;

                System.out.println("world X" + worldXPosition);
                System.out.println("world y" + worldYPosition);
                initializeStateVariables();
            }

            if (SystemClock.uptimeMillis()-stateStartTime > 650) { // this gives time for sample to drop
                if (!past5In) {
                    superstructure.nextState(Superstructure3Motor.SuperstructureStates.GOTO_RESTING_WORLDS.ordinal());
                    past5In = true;
                }
                if(SystemClock.uptimeMillis() - stateStartTime > 1250)
                {
                    //armPivot3Motor.setIntakeTiltAngle(0);
                    ArrayList<CurvePoint> points = new ArrayList<>();
                    points.add(new CurvePoint(stateStartingX, stateStartingY,
                            0, 0, 0, 0, 0, 0));

                    Pair<PointDouble, Double> wantedPos = pickUpPoints.get(cycle);

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
                            lift3Motor.getLiftExtension()<1 && armPivot3Motor.getArmAngle()<0) {
                        superstructure.sampleCollected = false;
                        if(cycle<3) {
                            superstructure.nextState(Superstructure3Motor.SuperstructureStates.SAMPLE_COLLECTION_EXTENSTION.ordinal());
                        }

                        nextStage(AutoSamplesWorlds.progStates.endBehavior.ordinal());
                    }

                    drive.applyMovementDirectionBased(); // always put at end of state
                }
            }
        }


        if (programStage == progStates.endBehavior.ordinal()) {
            if (stageFinished) {
                past5In = false;
                initializeStateVariables();
            }
            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(12, -55,
                    1 * SCALE_FACTOR, 1 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(60), 0.6));

            if (Movement.followCurve(points, Math.toRadians(90), 3)) {
                drive.stopAllMovementDirectionBased();
            }

            drive.applyMovementDirectionBased();
        }

        tp4.markStart();

        superstructure.update(telemetry, gamepad1, gamepad2);

        tp4.markEnd();

        System.out.println("Time Profiler 4 Average Time: " + tp4.getAverageTimePerUpdateMillis());

    }
}


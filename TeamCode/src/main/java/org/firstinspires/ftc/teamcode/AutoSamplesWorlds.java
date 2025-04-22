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
import org.firstinspires.ftc.teamcode.subsystems.Superstructure3Motor;

import java.util.ArrayList;
import java.util.HashMap;

@Autonomous
public class AutoSamplesWorlds extends Robot3Motor {

    private final double SCALE_FACTOR = 1;

    private long startTime = 0;

    private int cycle = 0;

    private boolean autoParkSuperstruture = false;

    private double currentServoPosition = 0;

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



        telemetry.addData("tilt servo", armPivot3Motor.intakeTilt.getPosition());
        telemetry.addData("tilt angle", armPivot3Motor.getIntakeTiltAngle());



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
        put(0, new Pair<>(new PointDouble(25.5, -8.9), 55.2)); // -31.3774
        put(1, new Pair<>(new PointDouble(25.1792,2.8118), 57.7)); // 50.0452
        put(2, new Pair<>(new PointDouble(24.9231, 11.8), 58.7)); //55.3688
        put(3, new Pair<>(new PointDouble(18, 18), 0.0));
    }};

//    put(0, new Pair<>(new PointDouble(28.5487, -10.6718), 67.3253)); // -31.3774
//    put(1, new Pair<>(new PointDouble(25.7552,5.5614 ), 60.0452)); // 50.0452
//    put(2, new Pair<>(new PointDouble(25.7771, 11.4609), 62.00)); //55.3688

    private boolean hasGrabbedPixels = false;

    private double cutOffTime = 22.5;

    private boolean past5In = false;

    @Override
    public void mainLoop() {
        telemetry.addData("auto state", programStage);

        if (programStage == progStates.driveToBaskets.ordinal()) {
            if (stageFinished) {
                past5In = false;

                superstructure.nextState(Superstructure3Motor.SuperstructureStates.DELIVERY_LEVEL_2_AUTO.ordinal());
                initializeStateVariables();
            }
            ArrayList<CurvePoint> points = new ArrayList<>();
            points.add(new CurvePoint(stateStartingX, stateStartingY,
                    0, 0, 0, 0, 0, 0));

            points.add(new CurvePoint(12.2, 13.6,
                    0.7 * SCALE_FACTOR, 0.7 * SCALE_FACTOR, 15, 15,
                    Math.toRadians(61), 0.6));

            if (Math.abs(Math.hypot(worldXPosition - 13, worldYPosition - 13)) < 5) {
                if (!past5In) {

                    past5In = true;
                }
            }
            boolean completed = Movement.followCurve(points, Math.toRadians(-90), 1.5);

            if (worldYPosition > 5) {
                Movement.movementResult r = Movement.pointAngle(Math.toRadians(-45), 0.7, Math.toRadians(20));

                if (completed && Math.abs(r.turnDelta_rad) < Math.toRadians(5)) {
                    drive.stopAllMovementDirectionBased();

                    System.out.println("lift extension in delivery state: " + lift3Motor.getLiftExtension());

                    if(lift3Motor.getLiftExtension()>22){
                        superstructure.nextState(Superstructure3Motor.SuperstructureStates.DELIVERY_SAMPLE_DROP.ordinal());

                        if (cycle < 3) {
                            nextStage(progStates.driveToOurSamples.ordinal());
                        } else {
                            nextStage(progStates.park.ordinal());
                        }
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
                            lift3Motor.getLiftExtension()<1 && armPivot3Motor.getArmAngle()<0) {
                        superstructure.sampleCollected = false;
                        if(cycle<3) {
                            superstructure.nextState(Superstructure3Motor.SuperstructureStates.SAMPLE_COLLECTION_EXTENSTION.ordinal());
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
            if (superstructure.sampleCollected && lift3Motor.getLiftExtension() < 1 ) {
                nextStage(progStates.driveToBaskets.ordinal());
            }
            drive.stopAllMovementDirectionBased();
        }

        if (programStage == progStates.park.ordinal()) {
            if (stageFinished) {
                past5In = false;
                autoParkSuperstruture = false;
                cycle++;
                initializeStateVariables();
            }
            if (SystemClock.uptimeMillis()-stateStartTime > 600) { //650
                if (!past5In) {
                    superstructure.nextState(Superstructure3Motor.SuperstructureStates.GOTO_RESTING_WORLDS.ordinal());
                    past5In = true;
                }
                // x = 60 y = -15 thetea = -90

                ArrayList<CurvePoint> points = new ArrayList<>();
                points.add(new CurvePoint(stateStartingX, stateStartingY,
                        0, 0, 0, 0, 0, 0));

                points.add(new CurvePoint(70, -15,
                        0.8 * SCALE_FACTOR, 0.8 * SCALE_FACTOR, 15, 15,
                        Math.toRadians(60), 0.6));

                boolean completed = Movement.followCurve(points, Math.toRadians(90), 3);

                if (Math.abs(Math.hypot(worldXPosition - 60, worldYPosition - (-15))) < 24) {
                    if (!autoParkSuperstruture) {
                        superstructure.nextState(Superstructure3Motor.SuperstructureStates.AUTO_PARK.ordinal());
                        autoParkSuperstruture = true;
                    }
                    Movement.movementResult r = Movement.pointAngle(
                            Math.toRadians(-90), //Math.atan2(pickupYPosition - stateStartingY, 34 - stateStartingX),
                            0.8,
                            Math.toRadians(30));
                }

                drive.applyMovementDirectionBased();
            }
        }

        tp4.markStart();

        superstructure.update(telemetry, gamepad1, gamepad2);

        tp4.markEnd();

        System.out.println("Time Profiler 4 Average Time: " + tp4.getAverageTimePerUpdateMillis());

    }
}


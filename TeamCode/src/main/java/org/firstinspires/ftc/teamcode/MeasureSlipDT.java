package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.RobotPosition.AngleWrap;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;
import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@Autonomous(name = "MeasureSlipDT", group = "auto1")
public class MeasureSlipDT extends Robot {

    //speed the robot will go before the slide
    private static double GOING_FORWARDS_SPEED = 1;
    private static double GOING_SIDEWAYS_SPEED = 1;
    private static double TURNING_SPEED = 1;


    //how long the robot will accelerate for before performing the slip
    private static final int ACCELERATION_TIME = 500;
    //how long the robot will wait until taking the measurement
    private static final int SLIP_TIME = 2000;


    //how fast (meters per second) we were going at end of movement_y
    private double movement_speed_y = 0.0;
    private double movement_speed_x = 0.0;
    private double turn_speed = 0.0;


    public enum progStates {

        goingForwards,//goes fast forwards
        measuringForwardsSlip,//stops, measuring the slip distance
        goingSideways,//goes fast sideways
        measuringSidewaysSlip,//stops, measuring the slip distance
        turning,//turns fast
        measuringTurningSlip,//stops, measures radians turned
        endProgram//does nothing, so values are still displayed
    }

    @Override
    public void init() {
        super.init();
        //start
        programStage = progStates.goingForwards.ordinal();

        //since we are measuring, start at 0,0,0
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();

        GoBildaOdo.setRobotPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS,0));
    }


    @Override
    public void loop() {
        super.loop();

        telemetry.addLine("Y power: " + GOING_FORWARDS_SPEED);
        telemetry.addLine("X power: " + GOING_SIDEWAYS_SPEED);
        telemetry.addLine("Turn power: " + TURNING_SPEED);


        /*if(ButtonPress.isGamepad1_dpad_up_pressed()){
            GOING_FORWARDS_SPEED += 0.02;
        }
        if(ButtonPress.isGamepad1_dpad_down_pressed()){
            GOING_FORWARDS_SPEED -= 0.02;
        }

        if(ButtonPress.isGamepad1_dpad_right_pressed()){
            GOING_SIDEWAYS_SPEED += 0.02;
        }
        if(ButtonPress.isGamepad1_dpad_left_pressed()){
            GOING_SIDEWAYS_SPEED -= 0.02;
        }
        if(ButtonPress.isGamepad1_y_pressed()){
            TURNING_SPEED += 0.02;
        }
        if(ButtonPress.isGamepad1_a_pressed()){
            TURNING_SPEED -= 0.02;
        }*/


        telemetry.addLine("y speed: " + movement_speed_y);
        telemetry.addLine("x speed: " + movement_speed_x);
        telemetry.addLine("turn speed: " + turn_speed);
    }

    @Override
    public void mainLoop() {
        //display the values that were calibrated
        telemetry.addData("ySlipDistanceFor1CMPS", SpeedOmeter.ySlipDistanceFor1CMPS);
        telemetry.addData("xSlipDistanceFor1CMPS", SpeedOmeter.xSlipDistanceFor1CMPS);
        telemetry.addData("turnSlipAmountFor1RPS", SpeedOmeter.turnSlipAmountFor1RPS);

        telemetry.addData("-------", "------");
        telemetry.addData("Speed X", SpeedOmeter.getSpeedX());
        telemetry.addData("Speed Y", SpeedOmeter.getSpeedY());
        telemetry.addData("Speed Angle Rad", SpeedOmeter.getRadPerSecond());

        telemetry.addData("Stage", programStage);

        //movement_y = -gamepad1.left_stick_y;
        //movement_x = gamepad1.left_stick_x;
        //movement_turn = -gamepad1.right_stick_x;

        //drive.applyMovementDirectionBased();

        long currTimeMillis = SystemClock.uptimeMillis();


        if (programStage == progStates.goingForwards.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }
            movement_y = GOING_FORWARDS_SPEED;
            movement_x = 0.0;
            movement_turn = 0.0;

//            System.out.println("Vel X: " + SpeedOmeter.getSpeedX());
//            System.out.println("Vel Y: " + SpeedOmeter.getSpeedY());
//            System.out.println("Vel DEG: " + SpeedOmeter.getDegPerSecond());

            //drive.applyMovementDirectionBased();
            Log.i("MOVEMENT SPEED Y BEFORE", String.valueOf(SpeedOmeter.getSpeedX()));

            if (currTimeMillis - stateStartTime > ACCELERATION_TIME) {
                movement_speed_y = SpeedOmeter.getSpeedY();
                Log.i("MOVEMENT SPEED Y", String.valueOf(movement_speed_y));
                drive.stopAllMovementDirectionBased();
                //nextStage();
            }
        }
        if (programStage == progStates.measuringForwardsSlip.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }

            double distance = Math.sqrt(Math.pow(worldXPosition - stateStartingX, 2) + Math.pow(worldYPosition - stateStartingY, 2));

            //Log.i("DISTANCE FORWARD", String.valueOf(distance));

            drive.hardStopMotors();


            if (currTimeMillis - stateStartTime > SLIP_TIME) {
                SpeedOmeter.ySlipDistanceFor1CMPS = distance / movement_speed_y;
                drive.stopAllMovementDirectionBased();
                nextStage();
            }
        }

        if (programStage == progStates.goingSideways.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }
            movement_y = 0.0;
            movement_x = GOING_SIDEWAYS_SPEED;
            movement_turn = 0.0;

            System.out.println("Vel X: " + SpeedOmeter.getSpeedX());
            System.out.println("Vel Y: " + SpeedOmeter.getSpeedY());
            System.out.println("Vel DEG: " + SpeedOmeter.getDegPerSecond());

            drive.applyMovementDirectionBased();

            if (currTimeMillis - stateStartTime > ACCELERATION_TIME) {
                movement_speed_x = SpeedOmeter.getSpeedX();
                drive.stopAllMovementDirectionBased();
                nextStage();
            }
        }
        if (programStage == progStates.measuringSidewaysSlip.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }

            double distance = Math.sqrt(Math.pow(worldXPosition - stateStartingX, 2) + Math.pow(worldYPosition - stateStartingY, 2));

            drive.hardStopMotors();

            if (currTimeMillis - stateStartTime > SLIP_TIME) {
                SpeedOmeter.xSlipDistanceFor1CMPS = distance / movement_speed_x;
                nextStage();
            }
        }
        if (programStage == progStates.turning.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }
            movement_y = 0.0;
            movement_x = 0.0;
            movement_turn = TURNING_SPEED;

            System.out.println("State X: " + SpeedOmeter.getSpeedX());
            System.out.println("State Y: " + SpeedOmeter.getSpeedY());
            System.out.println("State 3 Vel DEG: " + SpeedOmeter.getDegPerSecond());

            drive.applyMovementDirectionBased();

            if (currTimeMillis - stateStartTime > ACCELERATION_TIME) {
                turn_speed = SpeedOmeter.getRadPerSecond();
                drive.stopAllMovementDirectionBased();
                nextStage();
            }
        }
        if (programStage == progStates.measuringTurningSlip.ordinal()) {
            if (stageFinished) {
                initializeStateVariables();
            }

            double radsTurned = AngleWrap(worldAngle_rad - stateStartingAngle_rad);
            telemetry.addData("degreesTurned: ", Math.toDegrees(radsTurned));
            telemetry.addData("Turn speed deg:", Math.toDegrees(turn_speed));

            drive.hardStopMotors();

            if (currTimeMillis - stateStartTime > SLIP_TIME) {
                SpeedOmeter.turnSlipAmountFor1RPS = radsTurned / turn_speed;
                //nextStage(0);
            }
        }
    }


}
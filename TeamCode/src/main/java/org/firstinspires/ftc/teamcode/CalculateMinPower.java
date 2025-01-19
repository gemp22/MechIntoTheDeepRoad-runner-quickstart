package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.RobotPosition.AngleWrap;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@Autonomous(name = "CalculateMinPower")
@Disabled
public class CalculateMinPower extends Robot {

    public enum progStates{
        rampingForwards,
        measurnigForwardsSlip,
        rampingSideways,
        measuringSidewaysSlip,
        rampingTurning,
        measuringTurningSlip
    }

    @Override
    public void init(){
        super.init();
        //start
        programStage = progStates.rampingForwards.ordinal();

        //since we are measuring, start at 0,0,0
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void init_loop(){
        super.init_loop();
    }

    @Override
    public void start(){
        super.start();
    }

    @Override
    public void mainLoop() {
        telemetry.addData("MIN POWER Y: ", Movement.movement_y_min);
        telemetry.addData("MIN POWER X: ", Movement.movement_x_min);
        telemetry.addData("MIN POWER Turn: ", Movement.movement_turn_min);

        telemetry.addData("Speed X: ", SpeedOmeter.getSpeedX());
        telemetry.addData("Speed Y: ", SpeedOmeter.getSpeedY());
        telemetry.addData("Speed Ang Rad: ", SpeedOmeter.getRadPerSecond());

        long currTimeMillis = SystemClock.uptimeMillis();

        if(programStage == progStates.rampingForwards.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            //get the current time
            long elapsedMillis = currTimeMillis - stateStartTime;
            double elapsedSeconds = (double) elapsedMillis/1000.0;

            //it will take five seconds of measuring to ramp the power to the approximate minimum
            double approxSecondsToMin = 5.0;
            //use the current value as a good guess
            double estimatedMin = Movement.movement_y_min;
            movement_y = elapsedSeconds * estimatedMin / approxSecondsToMin;

            drive.applyMovementDirectionBased();

            double distance = Math.sqrt(Math.pow(worldXPosition-stateStartingX,2) + Math.pow(worldYPosition-stateStartingY,2));
            if(distance >= 2){
                Movement.movement_y_min = movement_y;
                drive.stopAllMovementDirectionBased();
                nextStage();
            }
        }
        if(programStage == progStates.measurnigForwardsSlip.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            if(currTimeMillis - stateStartTime > 1000){
                nextStage();
            }
        }

        if(programStage == progStates.rampingSideways.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            //get the current time
            long elapsedMillis = currTimeMillis - stateStartTime;
            double elapsedSeconds = (double) elapsedMillis/1000.0;

            //it will take five seconds of measuring to ramp the power to the approximate minimum
            double approxSecondsToMin = 5.0;
            //use the current value as a good guess
            double estimatedMin = Movement.movement_x_min;
            movement_x = elapsedSeconds * estimatedMin / approxSecondsToMin;

            drive.applyMovementDirectionBased();

            double distance = Math.sqrt(Math.pow(worldXPosition-stateStartingX,2) + Math.pow(worldYPosition-stateStartingY,2));
            if(distance >= 2){
                Movement.movement_x_min = movement_x;
                drive.stopAllMovementDirectionBased();
                nextStage();
            }
        }
        if(programStage == progStates.measuringSidewaysSlip.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            if(currTimeMillis - stateStartTime > 1000){
                nextStage();
            }
        }


        if(programStage == progStates.rampingTurning.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            //get the current time
            long elapsedMillis = currTimeMillis - stateStartTime;
            double elapsedSeconds = (double) elapsedMillis/1000.0;

            //it will take five seconds of measuring to ramp the power to the approximate minimum
            double approxSecondsToMin = 5.0;
            //use the current value as a good guess
            double estimatedMin = Movement.movement_turn_min;
            movement_turn = elapsedSeconds * estimatedMin / approxSecondsToMin;

            drive.applyMovementDirectionBased();

            double angle = AngleWrap(worldAngle_rad - stateStartingAngle_rad);
            if(Math.abs(angle) >= Math.toRadians(3)){
                Movement.movement_turn_min = movement_turn;
                drive.stopAllMovementDirectionBased();
                nextStage();
            }
        }
        if(programStage == progStates.measuringTurningSlip.ordinal()){
            if(stageFinished){
                initializeStateVariables();
            }
            if(currTimeMillis - stateStartTime > 1000){
                nextStage();
            }
        }
    }
}

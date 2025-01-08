package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;
import android.util.Log;
import android.util.Pair;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.ArmPivot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Superstructure;

import java.util.ArrayList;
import java.util.HashMap;

public abstract class Robot extends OpMode {
    MecanumDrive drive = null;
    ArmPivot armPivot;
    Lift lift;
    Superstructure superstructure;
    HashMap<String, Pair<Servo, Double>> servoMap = new HashMap<>();
    public boolean isAuto = false;
    double startingTiltPos = 0;
    double startingJawPos = 0;
    double startingTwistPos = 0;
    boolean guideToggle = false;
    boolean guidePreValue = false;
    AnalogInput rightDistance;
    AnalogInput leftDistance;

    //////// STATE MACHINE STUFF BELOW DO NOT TOUCH ////////
    public boolean stageFinished = true;
    public long stateStartTime = 0;

    public long programStartTime = 0;//time the program starts
    public static int programStage = 0;

    /**
     * STATE STARTING VARIABLES
     */
    public double stateStartingX = 0;
    public double stateStartingY = 0;
    public double stateStartingAngle_rad = 0;
    private final boolean DEBUGGING = false;
    private boolean inDebugState = false;

    //holds the stage we are going to next
    int nextStage = 0;

    public void nextStage(int ordinal) {
        nextStage = ordinal;
        //waits for a if on debug mode
        if (!DEBUGGING) {
            incrementStage();
            inDebugState = false;
        }

        //go into debug mode
        if (DEBUGGING) {
            inDebugState = true;
        }
    }

    /**
     * Increments the programStage
     */
    public void nextStage() {
        nextStage(programStage + 1);

    }

    private void incrementStage() {
        programStage = nextStage;
        stageFinished = true;
    }
    ///////////////////////////////

    DigitalChannel frontPixelReceiver;
    DigitalChannel backPixelReceiver;

    boolean liftLimitToggle = false;
    boolean liftLimitPreValue = false;

    public ArrayList<CurvePoint> mirrorPoints(ArrayList<CurvePoint> points) {
        ArrayList<CurvePoint> newPoints = new ArrayList<>();
        for (CurvePoint point : points) {
            newPoints.add(new CurvePoint(-point.x, point.y, point.moveSpeed, point.turnSpeed, point.followDistance, point.pointLength, point.slowDownTurnRadians, point.slowDownTurnRadians));
        }
        return newPoints;
    }

    @Override
    public void init() {

        armPivot = new ArmPivot(hardwareMap, servoMap);
        lift = new Lift(hardwareMap);
        lift.initLiftPController();
        armPivot.InitArmPivotPIDController();

        superstructure = new Superstructure(armPivot, lift, this);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        startingTiltPos = armPivot.intakeTilt.getPosition();
        startingJawPos = armPivot.intakeJawServo.getPosition();
        startingTwistPos = armPivot.twist.getPosition();
    }

    public double mmToIn(double in) {
        return in / 25.4;
    }

    private int pixelData = 0;
    private ArrayList<Boolean> count = new ArrayList<>();

    private int position = 1;

    double wantedX = 0;
    double wantedTiltPos = 0;
    double wantedJawPos = 0;
    double wantedTwistPos = 0;
    int intakeTiltState = 0;
    boolean gamepad1abool = false;

    @Override
    public void init_loop() {
        ButtonPress.giveMeInputs(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.dpad_up,
                gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.right_bumper,
                gamepad1.left_bumper, gamepad1.left_stick_button, gamepad1.right_stick_button,
                gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y, gamepad2.dpad_up,
                gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.right_bumper,
                gamepad2.left_bumper, gamepad2.left_stick_button, gamepad2.right_stick_button);

        telemetry.addData("tilt angle", armPivot.getIntakeTiltAngle());
        telemetry.addData("tilt servo Start pos", startingTiltPos);
        telemetry.addData("intake tilt Start servo pos", startingTiltPos);
        telemetry.addData("intake twist servo pos", startingTwistPos);

        telemetry.addData("jaw servo Start pos", startingJawPos);
        telemetry.addData("intake jaw servo pos", wantedJawPos+startingTiltPos);

    }


    @Override
    public void start() {
        programStage = 0;
    }

    @Override
    public void loop() {
        double startLoopTime = SystemClock.uptimeMillis();
       // PoseVelocity2d currentPoseVel = drive.updatePoseEstimate();

        telemetry.addLine("---------- GENERAL TELEMETRY BELOW ----------");
        telemetry.addData("Position Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);

        worldXPosition = drive.pose.position.x;
        worldYPosition = drive.pose.position.y;
        worldAngle_rad = drive.pose.heading.toDouble();

        // DO NOT CHANGE THIS LINE
        //SpeedOmeter.update(currentPoseVel.linearVel.y, currentPoseVel.linearVel.x, currentPoseVel.angVel);

        telemetry.addData("Velocity Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);

        superstructure.update(telemetry, gamepad1, gamepad2);

        mainAutoLoop();

        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));

        telemetry.addData("lift pos", lift.getLiftExtension());
        telemetry.addData("arm angle", armPivot.getArmAngle());
        telemetry.addData("tilt angle", armPivot.getIntakeTiltAngle());
        telemetry.addData("calc", armPivot.intakeTiltNoArmPower(lift.getLiftExtension()));

        telemetry.addData("tilt state", intakeTiltState);
        telemetry.addData("Button a is pressed", ButtonPress.isGamepad1_a_pressed());

        telemetry.addData("intake tilt servo pos", armPivot.intakeTilt.getPosition());

        telemetry.addData("twist servo Start pos", startingTwistPos);
        telemetry.addData("twist servo pos", wantedTwistPos+startingTwistPos);

        telemetry.addData("intake pow", armPivot.vexIntake.getPower());

        telemetry.addData("jaw servo Start pos", startingJawPos);
        telemetry.addData("intake jaw servo pos", wantedJawPos+startingTiltPos);

        telemetry.addData("lift limit Switch state", armPivot.getLiftLimitState());
        telemetry.addData("pivot limit switch state", armPivot.getPivotLimitState());

        telemetry.addData("wanted X", wantedX);
        telemetry.addData("lift position",lift.liftRight.getCurrentPosition());
        telemetry.addData("lift limit preVal",liftLimitPreValue);
    }

    public void initializeStateVariables() {
        stateStartingX = worldXPosition;
        stateStartingY = worldYPosition;
        stateStartingAngle_rad = worldAngle_rad;
        stateStartTime = SystemClock.uptimeMillis();
        Movement.initCurve();
        stageFinished = false;
    }

    private void mainAutoLoop() {
        if (inDebugState) {
            drive.stopAllMovementDirectionBased();
            // ControlMovement(); CHANGE THIS

            telemetry.addLine("in debug state");
            if (gamepad1.a) {
                incrementStage();
                inDebugState = false;
            }
        } else {
            mainLoop();
        }
    }

    public abstract void mainLoop();
}

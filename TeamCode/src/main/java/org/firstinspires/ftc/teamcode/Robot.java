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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.ArmPivot;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaOdo;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Superstructure;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Locale;

public abstract class Robot extends OpMode {
    MecanumDrive drive = null;
    GoBildaOdo GoBildaOdo; // Declare OpMode member for the Odometry Computer
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
        armPivot.intakeTilt.setPosition(Constants.TILT_SERVO_90_DEGREES_UP);
        armPivot.intakeJawServo.setPosition(Constants.JAW_SERVO_GRAB_POSITION);

        superstructure = new Superstructure(armPivot, lift, this);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        GoBildaOdo = new GoBildaOdo(hardwareMap);  //this is for GoBilda pinpoint

//        startingTiltPos = armPivot.intakeTilt.getPosition();
//        startingJawPos = armPivot.intakeJawServo.getPosition();
//        startingTwistPos = armPivot.twist.getPosition();
    }

    public double mmToIn(double in) {
        return in / 25.4;
    }


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

        telemetry.addData("World X", worldXPosition);
        telemetry.addData("World Y", worldYPosition);
        telemetry.addData("World Angle", worldAngle_rad);
        telemetry.addData("tilt angle", armPivot.getIntakeTiltAngle());

    }

    @Override
    public void start() {
        programStage = 0;
    }

    @Override
    public void loop() {
        double startLoopTime = SystemClock.uptimeMillis();
        //PoseVelocity2d currentPoseVel = drive.updatePoseEstimate();

        telemetry.addLine("---------- GENERAL TELEMETRY BELOW ----------");

//        worldXPosition = drive.pose.position.x;
//        worldYPosition = drive.pose.position.y;
//        worldAngle_rad = drive.pose.heading.toDouble();

        //GoBildaPinPoint

         Pair<Pose2D,Pose2D> goBildaPose = GoBildaOdo.GoBildaGetPose2D();

        worldXPosition = goBildaPose.first.getX(DistanceUnit.INCH);
        worldYPosition = goBildaPose.first.getY(DistanceUnit.INCH);
        worldAngle_rad = goBildaPose.first.getHeading(AngleUnit.RADIANS);

//        worldXPosition = goBildaPose.first.getY(DistanceUnit.INCH);
//        worldYPosition = goBildaPose.first.getX(DistanceUnit.INCH);
//        worldAngle_rad = goBildaPose.first.getHeading(AngleUnit.RADIANS);
        // DO NOT CHANGE THIS LINE
        SpeedOmeter.update(
                goBildaPose.second.getY(DistanceUnit.INCH),
                goBildaPose.second.getX(DistanceUnit.INCH),
                goBildaPose.second.getHeading(AngleUnit.RADIANS));

        //superstructure.update(telemetry, gamepad1, gamepad2);

        mainAutoLoop();

        telemetry.addData("Loop Time", SystemClock.uptimeMillis() - startLoopTime);
        telemetry.addData("World X", worldXPosition);
        telemetry.addData("World Y", worldYPosition);
        telemetry.addData("World Angle", Math.toDegrees(worldAngle_rad));

        telemetry.addData("vel x", goBildaPose.second.getX(DistanceUnit.INCH));
        telemetry.addData("vel y", goBildaPose.second.getY(DistanceUnit.INCH));
        telemetry.addData("vel heading", goBildaPose.second.getHeading(AngleUnit.RADIANS));



        Log.i("Loop Time", String.valueOf(SystemClock.uptimeMillis() - startLoopTime));
       // System.out.println("Loop Time: " + (SystemClock.uptimeMillis() - startLoopTime));

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

    public void mainLoop() {

    }
}

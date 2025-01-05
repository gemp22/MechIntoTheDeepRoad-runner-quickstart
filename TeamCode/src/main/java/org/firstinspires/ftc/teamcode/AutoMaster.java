package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotPosition.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldXPosition;
import static org.firstinspires.ftc.teamcode.RobotPosition.worldYPosition;

import android.os.SystemClock;
import android.util.Log;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public abstract class AutoMaster extends OpMode {
    MecanumDrive drive = null;

    public DcMotorEx robotLift = null;

    ArmPivot armPivot;
    Lift lift;

    double armSetPointLeft = 0;
    double armSetPointRight = 0;
    //int armPivotLeftPosition = 0;
    //int armPivotRightPosition = 0;
    int pivotMaxTicks = 2370;
    int stage = 0;
    int velocity = 0;
    double startingTiltPos = 0;
    double startingJawPos = 0;
    double startingTwistPos = 0;

    boolean guideToggle = false;
    boolean guidePreValue = false;

    AnalogInput rightDistance;
    AnalogInput leftDistance;


//clocks

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime waitTimer1 = new ElapsedTime();
    ElapsedTime waitTimer2 = new ElapsedTime();

    //Lift P controllers


    Servo pawright;

    // Vision for Tensor
/*
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model_20231227_193805.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "TPB",
            "TPR",
    };
    */

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    //public TfodProcessor tfod; commented this out for the intothedeep season since there is not vision

    /**
     * The variable to store our instance of the vision portal.
     */
    //private VisionPortal visionPortal;


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

    Vision vision;

    @Override
    public void init() {

        armPivot = new ArmPivot(hardwareMap);
        lift = new Lift(hardwareMap);
        lift.initLiftPController();
        armPivot.InitArmPivotPIDController();

        startingTiltPos = armPivot.intakeTilt.getPosition();
        startingJawPos = armPivot.intakeJawServo.getPosition();
        startingTwistPos = armPivot.twist.getPosition();

    }

    public double mmToIn(double in) {
        return in / 25.4;
    }

    public double getRightDistanceIn() {
        return mmToIn((rightDistance.getVoltage() / (3.3 / 1024)) * 6 - 300) + 4;
    }

    public double getLeftDistanceIn() {
        return mmToIn((leftDistance.getVoltage() / (3.3 / 1024)) * 6 - 300) + 4;
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
        PoseVelocity2d currentPoseVel = drive.updatePoseEstimate();

        mainAutoLoop();


        telemetry.addLine("---------- GENERAL TELEMETRY BELOW ----------");
        telemetry.addData("Position Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);

        worldXPosition = drive.pose.position.x;
        worldYPosition = drive.pose.position.y;
        worldAngle_rad = drive.pose.heading.toDouble();

        // DO NOT CHANGE THIS LINE
        //SpeedOmeter.update(currentPoseVel.linearVel.y, currentPoseVel.linearVel.x, currentPoseVel.angVel);

        telemetry.addData("Velocity Calculation Loop Time", SystemClock.uptimeMillis() - startLoopTime);

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

    /*
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.70f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }
    */

}

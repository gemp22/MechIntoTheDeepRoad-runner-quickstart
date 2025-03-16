package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotPosition.AngleWrap;

import android.os.SystemClock;
import android.util.Pair;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants3Motor;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.PController;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.Robot3Motor;

import java.util.HashMap;

public class ArmPivot3Motor {  //this is a subsystem Class used in Auto. its based on example for RR actions.
    public DcMotorEx armPivot;

    public CRServo vexIntake = null;
    public Servo intakeTilt;
    public Servo intakeJawServo;
    public Servo twist;

    public DigitalChannel liftLimitSwitch = null;
    public DigitalChannel pivotLimitSwitch = null;
    public int armPivotPosition = 0;
    private final double armPivotMaxTicks = 2370;

    // for lift pController input
    public final double minPowerArmPivot = .001;
    public final double maxPowerArmPivot = 0.99;

    // variable for RR1.0 actions
    //private double pixelLiftPosition; // variable for regular use like on init.. humm may not need since its defined in the method?
    private boolean exitArmPivotPControllerLoop = false;

    public PController pControllerArmPivot = new PController(0.005);

    public PIDController pidControllerArmPivot = new PIDController(0.005,0.00007,0);

    private double angularVelocity = 0.0;
    private double lastAngle = 0.0;

    private long lastUpdateStartTime = 0;

    public static double turnSlipAmountFor1RPS = 0.05;
    public static double armOffset = 0.0;

    public ArmPivot3Motor(HardwareMap hardwareMap, HashMap<String, Pair<Servo, Double>> servoMap) {
        armOffset = -8;
        vexIntake = hardwareMap.get(CRServo.class, "vexIntake");
        intakeTilt = hardwareMap.get(Servo.class, "intakeTilt");
        intakeTilt.setDirection(Servo.Direction.REVERSE);
        //intakeTilt.setPosition(Constants.TILT_SERVO_PARALLEL_WITH_FLOOR);

        armPivot = hardwareMap.get(DcMotorEx.class, "pivot");

        intakeJawServo = hardwareMap.get(ServoImplEx.class, "intakeJaw");
        intakeJawServo.setDirection(Servo.Direction.REVERSE);
        //intakeJawServo.setPosition(Constants.JAW_SERVO_INTAKE_POSITION);

        twist = hardwareMap.get(ServoImplEx.class, "twist");
        //twist.setPosition(Constants.TWIST_SERVO_HORIZONTAL_POSITION);

        liftLimitSwitch = hardwareMap.get(DigitalChannel.class, "liftLimitSwitch");
        pivotLimitSwitch = hardwareMap.get(DigitalChannel.class, "pivotLimitSwitch");

        armPivot.setDirection(DcMotor.Direction.FORWARD);
        armPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armPivot.setPower(0);

        if(Robot3Motor.resetEncoders) {
            armPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        servoMap.put("Tilt Servo", new Pair<>(intakeTilt, Constants3Motor.TILT_SERVO_PARALLEL_WITH_FLOOR));
        servoMap.put("Twist Servo", new Pair<>(twist, Constants3Motor.TWIST_SERVO_HORIZONTAL_POSITION));
        servoMap.put("Jaw Servo", new Pair<>(intakeJawServo, Constants3Motor.JAW_SERVO_INTAKE_POSITION));
    }

    public void InitArmPivotPController(){

        pControllerArmPivot.setInputRange(0, armPivotMaxTicks);
        pControllerArmPivot.setSetPoint(0);
        pControllerArmPivot.setOutputRange(minPowerArmPivot, maxPowerArmPivot);
        pControllerArmPivot.setThresholdValue(5);

    }
    public void InitArmPivotPIDController(){

        pidControllerArmPivot.setInputRangePID(0, armPivotMaxTicks);
        pidControllerArmPivot.setSetPointPID(0);
        pidControllerArmPivot.setOutputRangePID(minPowerArmPivot, maxPowerArmPivot);
        pidControllerArmPivot.setThresholdValuePID(5);

    }

    public void setArmPivotPosition() {
        armPivotPosition =armPivot.getCurrentPosition();

    }
    public void setArmPivotPower(double armPivotPower) {
        armPivot.setPower(armPivotPower);

    }
    public void setIntakeTiltAngle(double angle){
        double intakeTilt0Deg = Constants3Motor.TILT_SERVO_PARALLEL_WITH_FLOOR;
        double intakeTilt90Deg = Constants3Motor.TILT_SERVO_90_DEGREES_UP;
        //double output = (((intakeTilt90Deg - intakeTilt0Deg) / 90) * angle) + intakeTilt0Deg;
        double output = intakeTilt90Deg+((angle-90)*(intakeTilt0Deg-intakeTilt90Deg)/(0-90));

        intakeTilt.setPosition(Range.clip(output, intakeTilt90Deg, Constants3Motor.TILT_SERVO_CLOSE_TO_ROBOT_COLLECTION_POSITION));
    }

    public double getIntakeTiltAngle (){
        double intakeTilt0Deg = Constants3Motor.TILT_SERVO_PARALLEL_WITH_FLOOR;
        double intakeTilt90Deg = Constants3Motor.TILT_SERVO_90_DEGREES_UP;

        //return ((intakeTilt.getPosition() - intakeTilt0Deg) / ((intakeTilt90Deg - intakeTilt0Deg) / 90));

        return (90 + ((intakeTilt.getPosition() - intakeTilt90Deg) * (0 - 90) / (intakeTilt0Deg - intakeTilt90Deg)));

    }
    private double armMotorPower = 0;

    public void update(double targetAngle, double power, double slowDown, double slowDownPower, Telemetry telemetry) {
        long currentTime = SystemClock.uptimeMillis();

        double elapsedTime = (double) (currentTime - lastUpdateStartTime)/1000.0;

        lastUpdateStartTime = currentTime;

        armPivotPosition =armPivot.getCurrentPosition();

        double armAngle = (armPivotPosition / Constants3Motor.ARM_PIVOT_TICKS_PER_DEG) + armOffset; // degrees

        angularVelocity = AngleWrap(Math.toRadians(armAngle)-lastAngle) / elapsedTime;

        lastAngle = Math.toRadians(armAngle);

        double relativePointAngle = Math.toRadians(targetAngle) - Math.toRadians(armAngle); //AngleWrap(point_angle-worldAngle_rad);
        double velocityAdjustedRelativePointAngle = relativePointAngle-currSlipAngle();

        telemetry.addData("Relative Point Angle", relativePointAngle);
        telemetry.addData("Arm Angle", armAngle);


        //Scale down the relative angle by 40 and multiply by point speed
        double turnSpeed = (velocityAdjustedRelativePointAngle/Math.toRadians(slowDown))*power;

        //now just clip the result to be in range
        double powerOut = Range.clip(turnSpeed, -power, power);

        armMotorPower = Movement.minPower(powerOut, slowDownPower);
        telemetry.addData("Output Power raw", armMotorPower);
        //smooths down the last bit to finally settle on an angle
        armMotorPower *= Range.clip(Math.abs(relativePointAngle)/Math.toRadians(3),0,1); // was 3

        telemetry.addData("Output Power", armMotorPower);

        setArmPivotPower(armMotorPower);
    }

    public void update2(double targetAngle, double power, double slowDown, double slowDownPower, Telemetry telemetry) {
        long currentTime = SystemClock.uptimeMillis();
        double elapsedTime = (double) (currentTime - lastUpdateStartTime) / 1000.0;
        lastUpdateStartTime = currentTime;

        // Read encoder positions and calculate current arm angle
        armPivotPosition = armPivot.getCurrentPosition();


        double armAngle = (armPivotPosition / Constants3Motor.ARM_PIVOT_TICKS_PER_DEG) + armOffset;

        // Calculate angular velocity
        angularVelocity = AngleWrap(Math.toRadians(armAngle) - lastAngle) / elapsedTime;
        lastAngle = Math.toRadians(armAngle);

        // Calculate the error and relative target angle
        double relativePointAngle = Math.toRadians(targetAngle) - Math.toRadians(armAngle);
        double velocityAdjustedRelativePointAngle = relativePointAngle - currSlipAngle();

        telemetry.addData("Relative Point Angle", relativePointAngle);
        telemetry.addData("Arm Angle", armAngle);

        // PID-like adjustments
        double proportional = velocityAdjustedRelativePointAngle / Math.toRadians(slowDown);
        double derivative = -angularVelocity; // Negative to counter overshooting
        double smoothingFactor = Math.abs(relativePointAngle) / Math.toRadians(3);

        // Combine components with weights
        double turnSpeed = proportional * power + derivative * 0.00001; // Tweak the 0.5 weight as needed
        double powerOut = Range.clip(turnSpeed, -power, power);

        // Apply smoothing for final output
        //armMotorPower = Movement.minPower(powerOut, slowDownPower) * Range.clip(smoothingFactor, 0, 1);
        armMotorPower = Movement.minPower(powerOut, slowDownPower);


        armMotorPower *= Range.clip(Math.abs(relativePointAngle)/Math.toRadians(3),0,1); // was 3

        telemetry.addData("Output Power Raw", powerOut);
        telemetry.addData("Output Power Smoothed", armMotorPower);

        // Set motor power
        setArmPivotPower(armMotorPower);
    }

    // Class-level variables (ensure these are declared in your class)

    private double integralError = 0; // Cumulative integral error
    private static final double INTEGRAL_MAX = 10; // Anti-windup limit for the integral term

    public void update3 (double targetAngle, double maxPower, double slowDownRadius, double slowDownPower, Telemetry telemetry) {
        // Record current time and calculate elapsed time
        long currentTime = SystemClock.uptimeMillis();
        double elapsedTime = (currentTime - lastUpdateStartTime) / 1000.0;
        lastUpdateStartTime = currentTime;

        // Calculate the current arm angle based on encoder positions
        armPivotPosition = armPivot.getCurrentPosition();

        double armAngle = (armPivotPosition / Constants3Motor.ARM_PIVOT_TICKS_PER_DEG) - 8;

        // Calculate angular velocity
        angularVelocity = (AngleWrap(Math.toRadians(armAngle) - lastAngle)) / elapsedTime;
        lastAngle = Math.toRadians(armAngle);

        // Calculate the error to the target angle
        double relativeError = Math.toRadians(targetAngle) - Math.toRadians(armAngle);
        double velocityAdjustedError = relativeError - currSlipAngle();

        // Update integral term (sum of errors over time)
        integralError += relativeError * elapsedTime;

        // Apply anti-windup to prevent integral error from growing excessively
        //integralError = Range.clip(integralError, -INTEGRAL_MAX, INTEGRAL_MAX);

        // Add telemetry data for debugging
        telemetry.addData("Arm Angle", armAngle);
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Relative Error (Rad)", relativeError);
        telemetry.addData("Angular Velocity (Rad/s)", angularVelocity);
        telemetry.addData("Integral Error", integralError);

        // PID calculations
        double proportional = (velocityAdjustedError / Math.toRadians(slowDownRadius));
        double derivative = -angularVelocity; // Negative to counteract overshooting
        double integral = integralError * 0.00001; // Integral gain, tweak as necessary

        // Weight smoothing factor based on proximity to target
        //double smoothingFactor = Math.min(1.0, Math.abs(relativeError) / Math.toRadians(3));

        // Combine PID components for power calculation
        double rawPower = proportional * maxPower + integral + derivative * 0.000001; // Adjust weights as needed
        double clampedPower = Range.clip(rawPower, -maxPower, maxPower);

        // Apply smoothing and slow-down power constraints near the target
        //double smoothedPower = Movement.minPower(clampedPower, slowDownPower) * smoothingFactor;

        armMotorPower = Movement.minPower(clampedPower, slowDownPower);
        telemetry.addData("Output Power raw", armMotorPower);
        //smooths down the last bit to finally settle on an angle
        armMotorPower *= Range.clip(Math.abs(relativeError)/Math.toRadians(3),0,1); // was 3

        // Set motor power
        if(Math.abs(relativeError) <= Math.toRadians(5)){
            setArmPivotPower(Math.signum(armMotorPower)*.05);
        } else {
        setArmPivotPower(armMotorPower);
        }

        // Additional telemetry for debugging
        telemetry.addData("Raw Power", rawPower);
        telemetry.addData("Clamped Power", clampedPower);
        telemetry.addData("Smoothed Power", armMotorPower);

    }

    public double getArmAngle() {
        armPivotPosition =armPivot.getCurrentPosition();

        return (armPivotPosition / 17.6326) - 8;
    }
    public double getRadPerSecond() {
        return angularVelocity;
    }
    public double currSlipAngle() {
        return getRadPerSecond() * turnSlipAmountFor1RPS;
    }
    public void setArmPivotSetPointTicks(int ticks) {
        pControllerArmPivot.setSetPoint(ticks);
    }

    public void updatePControlArmPivotPosition() {
        armPivotPosition =armPivot.getCurrentPosition();

        if (armPivotPosition < pControllerArmPivot.setPoint) {

            armPivot.setPower(minPowerArmPivot +
                    pControllerArmPivot.getComputedOutput(armPivotPosition));
        } else {
            armPivot.setPower(minPowerArmPivot -
                    pControllerArmPivot.getComputedOutput(armPivotPosition));
        }

    }
    public void updateArmPivotPositionPID() {
        armPivotPosition =armPivot.getCurrentPosition();

        if (armPivotPosition < pidControllerArmPivot.setPoint) {

            armPivot.setPower(minPowerArmPivot +
                    pidControllerArmPivot.getComputedOutputPID(armPivotPosition));
        } else {
            armPivot.setPower(minPowerArmPivot -
                    pidControllerArmPivot.getComputedOutputPID(armPivotPosition));
        }

    }

    public void updateArmPivotPositionPIDwMotionProf() {
        armPivotPosition =armPivot.getCurrentPosition();

        double pidThreshold = 2* Constants3Motor.ARM_PIVOT_TICKS_PER_DEG; //2 degrees * 10.3920133 tick/deg = ticks see Excel calcs
        double decelSlope = 4;
        double setPoint = pidControllerArmPivot.setPoint;
        int position = armPivotPosition;


        if (setPoint-pidThreshold > position || setPoint+pidThreshold < position){
            double armPivotvelocity = Math.min(1000, Math.abs(0+(decelSlope*(position-setPoint))));// point slope formula to get velocity profile see excel file
            double liftVelocity = armPivotvelocity/5.061079545;
        }
        else {

            if (armPivotPosition < pidControllerArmPivot.setPoint) {

                armPivot.setPower(minPowerArmPivot +
                        pidControllerArmPivot.getComputedOutputPID(armPivotPosition));
            } else {
                armPivot.setPower(minPowerArmPivot -
                        pidControllerArmPivot.getComputedOutputPID(armPivotPosition));
            }

        }

    }
    public void setArmPivotVelocity(double velocity)
    {
        armPivot.setVelocity(velocity);

    }

    public boolean getLiftLimitState(){
        return (liftLimitSwitch.getState());

    }

    public boolean getPivotLimitState(){
        return (pivotLimitSwitch.getState());

    }

    public double intakeTiltNoArmPower(double liftExtension) {

        //linear interpolation to return tilt angle for sample collection off the floor
        double intakeTiltAngle = Constants3Motor.TILT_INTAKE_ANGLE_CLOSE_TO_BOT+
                (liftExtension-Constants3Motor.LIFT_EXTENSION_FOR_COLLECTION_CLOSE_TO_BOT)* (Constants3Motor.TILT_INTAKE_ANGLE_FAR_FROM_BOT-Constants3Motor.TILT_INTAKE_ANGLE_CLOSE_TO_BOT)/
                (Constants3Motor.LIFT_EXTENSION_FOR_COLLECTION_FAR_FROM_BOT-Constants3Motor.LIFT_EXTENSION_FOR_COLLECTION_CLOSE_TO_BOT);
//intakeTilt90Deg+((angle-90)*(intakeTilt0Deg-intakeTilt90Deg)/(0-90));
        return intakeTiltAngle;
    }











}

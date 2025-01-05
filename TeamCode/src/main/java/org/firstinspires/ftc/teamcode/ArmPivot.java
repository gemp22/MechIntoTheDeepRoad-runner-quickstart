package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotPosition.AngleWrap;

import android.os.SystemClock;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmPivot {  //this is a subsystem Class used in Auto. its based on example for RR actions.
    public DcMotorEx armPivotLeft;
    public DcMotorEx armPivotRight;
    public CRServo vexIntake = null;
    public Servo intakeTilt;
    public Servo intakeJawServo;
    public Servo twist;

    DigitalChannel liftLimitSwitch = null;
    DigitalChannel pivotLimitSwitch = null;




    public int armPivotLeftPosition = 0;
    public int armPivotRightPosition = 0;
    private double armPivotLeftMaxTicks = 2370;
    private double armPivotRightMaxTicks = 2370;
    // for lift pController input
    public final double minPowerArmPivotLeft = .001;
    public final double minPowerArmPivotRight = .001;
    public final double maxPowerArmPivotLeft = 0.99;
    public final double maxPowerArmPivotRight = 0.99;
    private double armPivotLeftActionPosition;
    private double armPivotRightActionPosition;

    // variable for RR1.0 actions
    //private double pixelLiftPosition; // variable for regular use like on init.. humm may not need since its defined in the method?
    private boolean exitArmPivotLeftPControllerLoop = false;
    private boolean exitArmPivotRightPControllerLoop = false;
    public PController pControllerArmPivotLeft = new PController(0.005);
    public PController pControllerArmPivotRight = new PController(0.005);

    public PIDController pidControllerArmPivotLeft = new PIDController(0.005,0.00007,0);
    public PIDController pidControllerArmPivotRight = new PIDController(0.005,0.00007,0);

    private double angularVelocity = 0.0;
    private double lastAngle = 0.0;

    private long lastUpdateStartTime = 0;

    public static double turnSlipAmountFor1RPS = 0.05;

    public ArmPivot(HardwareMap hardwareMap) {
        vexIntake = hardwareMap.get(CRServo.class, "vexIntake");
        intakeTilt = hardwareMap.get(Servo.class, "intakeTilt");
        //intakeTilt.setPosition(0.5);

        armPivotLeft = hardwareMap.get(DcMotorEx.class, "leftPivot");

        intakeJawServo = hardwareMap.get(ServoImplEx.class, "intakeJaw");
        intakeJawServo.setDirection(Servo.Direction.REVERSE);
        //intakeJawServo.setPosition(.7);

        twist = hardwareMap.get(ServoImplEx.class, "twist");
       // twist.setPosition(0.2);

        liftLimitSwitch = hardwareMap.get(DigitalChannel.class, "liftLimitSwitch");
        pivotLimitSwitch = hardwareMap.get(DigitalChannel.class, "pivotLimitSwitch");


        armPivotLeft.setDirection(DcMotor.Direction.FORWARD);
        armPivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armPivotLeft.setPower(0);
        armPivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPivotLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        armPivotRight = hardwareMap.get(DcMotorEx.class, "rightPivot");
        armPivotRight.setDirection(DcMotor.Direction.REVERSE);
        armPivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armPivotRight.setPower(0);
        armPivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPivotRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void InitArmPivotPController(){

        pControllerArmPivotLeft.setInputRange(0, armPivotLeftMaxTicks);
        pControllerArmPivotLeft.setSetPoint(0);
        pControllerArmPivotLeft.setOutputRange(minPowerArmPivotLeft, maxPowerArmPivotLeft);
        pControllerArmPivotLeft.setThresholdValue(5);


        pControllerArmPivotRight.setInputRange(0, armPivotRightMaxTicks);
        pControllerArmPivotRight.setSetPoint(0);
        pControllerArmPivotRight.setOutputRange(minPowerArmPivotRight, maxPowerArmPivotRight);
        pControllerArmPivotRight.setThresholdValue(5);
        //armPivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // this is for lift left, change Kp to calibrate
    }
    public void InitArmPivotPIDController(){

        pidControllerArmPivotLeft.setInputRangePID(0, armPivotLeftMaxTicks);
        pidControllerArmPivotLeft.setSetPointPID(0);
        pidControllerArmPivotLeft.setOutputRangePID(minPowerArmPivotLeft, maxPowerArmPivotLeft);
        pidControllerArmPivotLeft.setThresholdValuePID(5);


        pidControllerArmPivotRight.setInputRangePID(0, armPivotRightMaxTicks);
        pidControllerArmPivotRight.setSetPointPID(0);
        pidControllerArmPivotRight.setOutputRangePID(minPowerArmPivotRight, maxPowerArmPivotRight);
        pidControllerArmPivotRight.setThresholdValuePID(5);
        //armPivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // this is for lift left, change Kp to calibrate
    }

    public void setArmPivotPosition() {
        armPivotLeftPosition=armPivotLeft.getCurrentPosition();
        armPivotRightPosition=armPivotRight.getCurrentPosition();

    }
    public void setArmPivotPower(double armPivotPower) {
        armPivotLeft.setPower(armPivotPower);
        armPivotRight.setPower(armPivotPower);

    }
    public void setIntakeTiltAngle (double angle){
        double intakeTilt0Deg = 0.44;
        double intakeTilt90Deg = 0.156;


        //double output = (((intakeTilt90Deg - intakeTilt0Deg) / 90) * angle) + intakeTilt0Deg;
        double output = intakeTilt90Deg+((angle-90)*(intakeTilt0Deg-intakeTilt90Deg)/(0-90));

        intakeTilt.setPosition(Range.clip(output, 0.19, intakeTilt90Deg));
    }

    public double getIntakeTiltAngle (){

        double intakeTilt0Deg = 0.44;
        double intakeTilt90Deg = 0.156;

        //return ((intakeTilt.getPosition() - intakeTilt0Deg) / ((intakeTilt90Deg - intakeTilt0Deg) / 90));

        return (90 + ((intakeTilt.getPosition() - intakeTilt90Deg) * (0 - 90) / (intakeTilt0Deg - intakeTilt90Deg)));

    }
    private double armMotorPower = 0;

    public void update(double targetAngle, double power, double slowDown, double slowDownPower, Telemetry telemetry) {
        long currentTime = SystemClock.uptimeMillis();

        double elapsedTime = (double) (currentTime - lastUpdateStartTime)/1000.0;

        lastUpdateStartTime = currentTime;

        armPivotLeftPosition=armPivotLeft.getCurrentPosition();
        armPivotRightPosition=armPivotRight.getCurrentPosition();

        double averageArmPosition = (armPivotLeftPosition+armPivotRightPosition)/2.0;

        double armAngle = (averageArmPosition / 10.390208333) - 8; // degrees

        angularVelocity = AngleWrap(Math.toRadians(armAngle)-lastAngle) / elapsedTime;

        lastAngle = Math.toRadians(armAngle);

        double relativePointAngle = Math.toRadians(targetAngle) - Math.toRadians(armAngle); //AngleWrap(point_angle-worldAngle_rad);
        double velocityAdjustedRelativePointAngle = relativePointAngle-currSlipAngle();

        telemetry.addData("Relative Point Angle", relativePointAngle);

        //Scale down the relative angle by 40 and multiply by point speed
        double turnSpeed = (velocityAdjustedRelativePointAngle/Math.toRadians(slowDown))*power;

        //now just clip the result to be in range
        double powerOut = Range.clip(turnSpeed, -power, power);

        armMotorPower = Movement.minPower(powerOut, slowDownPower);

        //smooths down the last bit to finally settle on an angle
        armMotorPower *= Range.clip(Math.abs(relativePointAngle)/Math.toRadians(2),0,1); // was 3

        telemetry.addData("Output Power", armMotorPower);

        setArmPivotPower(armMotorPower);
    }

    public double getArmAngle() {
        armPivotLeftPosition=armPivotLeft.getCurrentPosition();
        armPivotRightPosition=armPivotRight.getCurrentPosition();

        double averageArmPosition = (armPivotLeftPosition+armPivotRightPosition)/2.0;

        return (averageArmPosition / 10.390208333) - 5;
    }

    public double getRadPerSecond() {
        return angularVelocity;
    }

    public double currSlipAngle() {
        return getRadPerSecond() * turnSlipAmountFor1RPS;
    }

    public void updateArmPivotPosition() {
        armPivotLeftPosition=armPivotLeft.getCurrentPosition();
        armPivotRightPosition=armPivotRight.getCurrentPosition();

        if (armPivotLeftPosition < pControllerArmPivotLeft.setPoint) {

            armPivotLeft.setPower(minPowerArmPivotLeft +
                    pControllerArmPivotLeft.getComputedOutput(armPivotLeftPosition));
        } else {
            armPivotLeft.setPower(minPowerArmPivotLeft -
                    pControllerArmPivotLeft.getComputedOutput(armPivotLeftPosition));
        }


        if (armPivotRightPosition < pControllerArmPivotRight.setPoint) {

            armPivotRight.setPower(minPowerArmPivotRight +
                    pControllerArmPivotRight.getComputedOutput(armPivotRightPosition));
        } else {
            armPivotRight.setPower(minPowerArmPivotRight -
                    pControllerArmPivotRight.getComputedOutput(armPivotRightPosition));
        }
    }
    public void updateArmPivotPositionPID() {
        armPivotLeftPosition=armPivotLeft.getCurrentPosition();
        armPivotRightPosition=armPivotRight.getCurrentPosition();

        if (armPivotLeftPosition < pidControllerArmPivotLeft.setPoint) {

            armPivotLeft.setPower(minPowerArmPivotLeft +
                    pidControllerArmPivotLeft.getComputedOutputPID(armPivotLeftPosition));
        } else {
            armPivotLeft.setPower(minPowerArmPivotLeft -
                    pidControllerArmPivotLeft.getComputedOutputPID(armPivotLeftPosition));
        }


        if (armPivotRightPosition < pidControllerArmPivotRight.setPoint) {

            armPivotRight.setPower(minPowerArmPivotRight +
                    pidControllerArmPivotRight.getComputedOutputPID(armPivotRightPosition));
        } else {
            armPivotRight.setPower(minPowerArmPivotRight -
                    pidControllerArmPivotRight.getComputedOutputPID(armPivotRightPosition));
        }
    }

    public void updateArmPivotPositionPIDwMotionProf() {
        armPivotLeftPosition=armPivotLeft.getCurrentPosition();
        armPivotRightPosition=armPivotRight.getCurrentPosition();
        double pidThreshold = 2*10.3920833; //2 degrees * 10.3920133 tick/deg = ticks see Excel calcs
        double decelSlope = 4;
        double setPointAvg = (pidControllerArmPivotLeft.setPoint + pidControllerArmPivotRight.setPoint)/2;
        int positionAvg = (armPivotLeftPosition+armPivotRightPosition)/2;


        if (setPointAvg-pidThreshold > positionAvg || setPointAvg+pidThreshold < positionAvg){
            double armPivotvelocity = Math.min(1000, Math.abs(0+(decelSlope*(positionAvg-setPointAvg))));// point slope formula to get velocity profile see excel file
            double liftVelocity = armPivotvelocity/5.061079545;
        }
        else {

            if (armPivotLeftPosition < pidControllerArmPivotLeft.setPoint) {

                armPivotLeft.setPower(minPowerArmPivotLeft +
                        pidControllerArmPivotLeft.getComputedOutputPID(armPivotLeftPosition));
            } else {
                armPivotLeft.setPower(minPowerArmPivotLeft -
                        pidControllerArmPivotLeft.getComputedOutputPID(armPivotLeftPosition));
            }


            if (armPivotRightPosition < pidControllerArmPivotRight.setPoint) {

                armPivotRight.setPower(minPowerArmPivotRight +
                        pidControllerArmPivotRight.getComputedOutputPID(armPivotRightPosition));
            } else {
                armPivotRight.setPower(minPowerArmPivotRight -
                        pidControllerArmPivotRight.getComputedOutputPID(armPivotRightPosition));
            }
        }

    }
    public void setArmPivotVelocity(double velocity)
    {
        armPivotLeft.setVelocity(velocity);
        armPivotRight.setVelocity(velocity);
    }

    public boolean getLiftLimitState(){
        return (liftLimitSwitch.getState());

    }

    public boolean getPivotLimitState(){
        return (pivotLimitSwitch.getState());

    }

    public double intakeTiltNoArmPower(double liftExtension) {

        double intakeTiltAngle = -69+(liftExtension-4.33)*(-25-(-69))/(16.5-4.33);

        return intakeTiltAngle;
    }











}

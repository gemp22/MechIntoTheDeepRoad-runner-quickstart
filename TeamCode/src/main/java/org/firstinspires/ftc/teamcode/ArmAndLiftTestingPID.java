/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="armAndLift PID testing :)", group="Iterative OpMode")
public class ArmAndLiftTestingPID extends OpMode
{

    //gitTest
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
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
    boolean liftLimitToggle = false;
    boolean liftLimitPreValue = false;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "init");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        armPivot = new ArmPivot(hardwareMap);
        lift = new Lift(hardwareMap);
        lift.initLiftPController();
        armPivot.InitArmPivotPIDController();


        startingTiltPos = armPivot.intakeTilt.getPosition();
        startingJawPos = armPivot.intakeJawServo.getPosition();
        startingTwistPos = armPivot.twist.getPosition();


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        // Tell the driver that initialization is complete.

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addData("tilt angle", armPivot.getIntakeTiltAngle());
        telemetry.addData("tilt servo Start pos", startingTiltPos);
        telemetry.addData("intake tilt Start servo pos", startingTiltPos);
        telemetry.addData("intake twist servo pos", startingTwistPos);

        telemetry.addData("jaw servo Start pos", startingJawPos);
        telemetry.addData("intake jaw servo pos", wantedJawPos+startingTiltPos);
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }
    double wantedPos = 0.5;

    public static double groundToPivot = 5.988;

    public void intakeMagic(double x, Telemetry telemetry) {
        double liftX = 0 + 1.133 * Math.sin(Math.toRadians(armPivot.getArmAngle()));
        double liftY = 5.988 + 1.133 * Math.cos(Math.toRadians(armPivot.getArmAngle()));

        double targetX = x+liftX;
        double targetY = 5-liftY;

        double liftWantedExtension = Math.sqrt(Math.pow(targetX, 2) + Math.pow(targetY + 5.988, 2)); // ref is right side of bot
        double armWantedAngle = Math.toDegrees(Math.atan2(targetY, targetX));
        double intakeWantedAngle = armWantedAngle * -1;

        armWantedAngle = Range.clip(armWantedAngle, -2, 90);
        armPivot.update(armWantedAngle, 0.5, 30,0.2, telemetry);

        liftWantedExtension = Range.clip(liftWantedExtension, -8, 25);
        lift.setSetPoint(liftWantedExtension);

        armPivot.setIntakeTiltAngle(intakeWantedAngle);

        telemetry.addData("liftWantedExtension", liftWantedExtension);
        telemetry.addData("armWantedAngle", armWantedAngle);
        telemetry.addData("intakeWantedAngle", intakeWantedAngle);
        telemetry.addData("Lift X", liftX);
        telemetry.addData("Lift Y", liftY);

        telemetry.addData("arm angle", armPivot.getArmAngle());
    }

    public double intakeTiltNoArmPower(double liftExtension) {

        double intakeTiltAngle = -69+(liftExtension-4.33)*(-25-(-69))/(16.5-4.33);
        telemetry.addData("Wanted intake", intakeTiltAngle);
        return intakeTiltAngle;
    }

    double wantedX = 0;
    double wantedTiltPos = 0;
    double wantedJawPos = 0;
    double wantedTwistPos = 0;




    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*if (gamepad1.right_bumper) {
            armPivot.update(90, 0.5, 30,0.2, telemetry);
        } else if (gamepad1.left_bumper) {
            armPivot.update(0, 0.15, 60, 0.02, telemetry);
        }
*/
        //lift.setLiftPower(gamepad1.right_stick_y);

/*
        if (gamepad1.dpad_right) {
            wantedTiltPos += 0.002;

        } else if (gamepad1.dpad_left) {
            wantedTiltPos -= 0.002;

        } else {
            armPivot.intakeTilt.setPosition(startingTiltPos+wantedTiltPos);
        }

 */

        if (gamepad1.cross) {
            wantedJawPos += 0.002;

        } else if (gamepad1.circle) {
            wantedJawPos -= 0.002;

        } else {
            armPivot.intakeJawServo.setPosition(startingJawPos+wantedJawPos);
        }

        if (gamepad1.triangle) {
            wantedTwistPos += 0.002;

        } else if (gamepad1.square) {
            wantedTwistPos -= 0.002;

        } else {
            armPivot.twist.setPosition(startingTwistPos+wantedTwistPos);
        }

        //intake reverser
        if (gamepad1.guide && !guidePreValue) {

            guideToggle = !guideToggle; // reverse boolean

            if (guideToggle) {
                armPivot.vexIntake.setPower(-.91); //on
            } else {
                armPivot.vexIntake.setPower(0); //off
            }
        }
        guidePreValue = gamepad1.guide;


//
//        if (gamepad1.a) {
//            lift.setSetPoint(15);
//
//        }
//        lift.updateLiftPosition();


        if (gamepad1.dpad_up) {
            wantedX += 0.05;
        }
        if (gamepad1.dpad_down) {
            wantedX -= 0.05;
        }

        if (lift.getLiftExtension()>4.33) {
            armPivot.setIntakeTiltAngle(intakeTiltNoArmPower(lift.getLiftExtension()));
            //armPivot.setIntakeTiltAngle(-60);
        }
        else  {
            armPivot.setIntakeTiltAngle(90);
        }


        if (armPivot.getLiftLimitState() && !liftLimitPreValue) {
            lift.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wantedX =0;
        }
        liftLimitPreValue = armPivot.getLiftLimitState();


        lift.setSetPoint(wantedX);
        lift.updateLiftPosition();


        telemetry.addData("lift pos", lift.getLiftExtension());
        telemetry.addData("arm angle", armPivot.getArmAngle());
        telemetry.addData("tilt angle", armPivot.getIntakeTiltAngle());
        telemetry.addData("calc", intakeTiltNoArmPower(lift.getLiftExtension()));

        telemetry.addData("tilt servo Start pos", startingTiltPos);
        telemetry.addData("intake tilt servo pos", armPivot.intakeTilt.getPosition());

        telemetry.addData("twist servo Start pos", startingTwistPos);
        telemetry.addData("twist servo pos", wantedTwistPos+startingTwistPos);

        telemetry.addData("intake pow", armPivot.vexIntake.getPower());

        telemetry.addData("jaw servo Start pos", startingJawPos);
        telemetry.addData("intake jaw servo pos", wantedJawPos+startingTiltPos);

        telemetry.addData("lift limit Switch state", armPivot.getLiftLimitState());
        telemetry.addData("limit limit switch state", armPivot.getPivotLimitState());

        telemetry.addData("wanted X", wantedX);
        telemetry.addData("lift position",lift.liftRight.getCurrentPosition());
        telemetry.addData("lift limit preVal",liftLimitPreValue);


//        wantedPos = Range.clip(wantedPos, 0, 1);
//        armPivot.intakeTilt.setPosition(wantedPos);
//        telemetry.addData("servo pos", wantedPos);



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    public static boolean isInvalidPoint(double armSetPointLeft, double armSetPointRight)
    {
        if(armSetPointLeft < 0 || armSetPointRight < 0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }


}

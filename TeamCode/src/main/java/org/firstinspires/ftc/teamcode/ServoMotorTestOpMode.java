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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="ServoMotorTestOpMode", group="Iterative OpMode")
//@Disabled
public class ServoMotorTestOpMode extends OpMode {
    // Declare OpMode members.
    Wrist wrist = null;
    Intake intake = null;
    IntakeJawServo intakeJawServo = null;
    ElapsedTime runtime = new ElapsedTime();
    boolean isGrabbingSample;
    boolean isOpeningIntake;
    boolean isTiltUp;
    boolean isTwistLeft;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        wrist = new Wrist(hardwareMap);
        intakeJawServo = new IntakeJawServo(hardwareMap);
        intake = new Intake(hardwareMap);
        //smaller #s is right, larger #s is left
        wrist.intakeTwist.setPosition(0); //init twist
        wrist.intakeTilt.setPosition(.5); //init Tilt

        intakeJawServo.intakeJawServo.setPosition(.5);
        //larger #s open, smaller #s close
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

        telemetry.addData("twist", wrist.intakeTwist.getPosition());
        telemetry.addData("tilt", wrist.intakeTilt.getPosition());
        telemetry.addData("jaw", intake.vexIntake.getPower());
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        telemetry.addData("Status","Run Time:"+runtime.toString());

        ButtonPress.giveMeInputs(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.dpad_up,
                gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.right_bumper,
                gamepad1.left_bumper, gamepad1.left_stick_button, gamepad1.right_stick_button,
                gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y, gamepad2.dpad_up,
                gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.right_bumper,
                gamepad2.left_bumper, gamepad2.left_stick_button, gamepad2.right_stick_button);


        if (ButtonPress.isGamepad1_right_bumper_pressed()) {

            if (isGrabbingSample) {
                intakeJawServo.intakeJawServo.setPosition(.7);
                intake.vexIntake.setPower(0);
                wrist.intakeTilt.setPosition(.45);
                isGrabbingSample = false;

            } else {
                intakeJawServo.intakeJawServo.setPosition(.5);
                intake.vexIntake.setPower(-.9);
                isGrabbingSample = true;
            }
        } else if (ButtonPress.isGamepad1_left_bumper_pressed()) {
            if (isOpeningIntake) {
                intakeJawServo.intakeJawServo.setPosition(.7);
                intake.vexIntake.setPower(0);
                isOpeningIntake = false;

            } else {
                intakeJawServo.intakeJawServo.setPosition(.5);
                intake.vexIntake.setPower(.9);
                isOpeningIntake = true;
            }
        }
//tilt
        if (ButtonPress.isGamepad1_a_pressed()) {
            if (isTiltUp) {
                wrist.intakeTilt.setPosition(.35);
                isTiltUp = false;

            } else {
                wrist.intakeTilt.setPosition(.50);
                isTiltUp = true;
            }
        }
        //twist
        else if (ButtonPress.isGamepad1_b_pressed()) {

            if (isTwistLeft) {
                wrist.intakeTwist.setPosition(0);
                isTwistLeft = false;
            } else {
                wrist.intakeTwist.setPosition(1);
                isTwistLeft = true;
            }
        }


        telemetry.addData("twist", wrist.intakeTwist.getPosition());
        telemetry.addData("tilt", wrist.intakeTilt.getPosition());
        telemetry.addData("jaw", intakeJawServo.intakeJawServo.getPosition());
        telemetry.addData("intake", intake.vexIntake.getPower());
        telemetry.update();
    }
        /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

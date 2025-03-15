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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ArmPivot3Motor;
import org.firstinspires.ftc.teamcode.subsystems.Lift3Motor;

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

@TeleOp(name="3MotorPivot and lift test", group="Iterative OpMode")

public class PivotTesting3Motor extends Robot
{

    //gitTest
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    ArmPivot3Motor armPivot3Motor;
    Lift3Motor lift3Motor;
    double armSetPointLeft = 0;
    double armSetPointRight = 0;
    //int armPivotLeftPosition = 0;
    //int armPivotRightPosition = 0;
    int pivotMaxTicks = 2370;
    int stage = 0;




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "init");

        armPivot3Motor = new ArmPivot3Motor(hardwareMap, servoMap);
        lift3Motor = new Lift3Motor(hardwareMap);


        armPivot3Motor.armPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPivot3Motor.armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPivot3Motor.armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift3Motor.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift3Motor.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift3Motor.liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift3Motor.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift3Motor.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift3Motor.liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift3Motor.upperLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift3Motor.upperLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift3Motor.upperLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armPivot3Motor.InitArmPivotPIDController();
        armPivot3Motor.InitArmPivotPController();
        lift3Motor.initLiftPController();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
       // armPivot = new ArmPivot(hardwareMap);


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
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        //super.start();
    }


    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
    //super.mainLoop();
        //armPivotLeftPosition = armPivot.armPivotLeft.getCurrentPosition();  I don't think we need this if using the "update arm pivot method in th ArmPivot Class
        //armPivotRightPosition = armPivot.armPivotRight.getCurrentPosition();
        //manual pivot controller
        if (gamepad1.dpad_down) {   ///move lift up and sets controller position

            armPivot3Motor.setArmPivotPower(.98);

            armPivot3Motor.pControllerArmPivot.setSetPoint(armPivot3Motor.armPivot.getCurrentPosition());

        }
        else if (gamepad1.dpad_up) {  //move lift down and sets controller position

            armPivot3Motor.setArmPivotPower(-.98);

            armPivot3Motor.pControllerArmPivot.setSetPoint(armPivot3Motor.armPivot.getCurrentPosition());

        }
        else {    //uses proportional controller to hold lift in correct spot
//                 armPivot3Motor.setArmPivotPower(0);
            //armPivot3Motor.updatePControlArmPivotPosition();
        }

        if (gamepad1.dpad_right) {   ///move lift up and sets controller position

            lift3Motor.setLiftPower(.98);


            lift3Motor.pControllerLiftRight.setSetPoint(lift3Motor.liftRight.getCurrentPosition());
            lift3Motor.pControllerLiftLeft.setSetPoint(lift3Motor.liftLeft.getCurrentPosition());
            lift3Motor.pControllerUpperLift.setSetPoint(lift3Motor.upperLift.getCurrentPosition());

        }
        else if (gamepad1.dpad_left) {  //move lift down and sets controller position

            lift3Motor.setLiftPower(-.98);


            lift3Motor.pControllerLiftRight.setSetPoint(lift3Motor.liftRight.getCurrentPosition());
            lift3Motor.pControllerLiftLeft.setSetPoint(lift3Motor.liftLeft.getCurrentPosition());
            lift3Motor.pControllerUpperLift.setSetPoint(lift3Motor.upperLift.getCurrentPosition());

        }
        else {
            //lift3Motor.updateLiftPosition();
//            lift3Motor.liftLeft.setPower(0);
//            lift3Motor.liftRight.setPower(0);
//            lift3Motor.upperLift.setPower(0);
// uses proportional controller to hold lift in correct spot
         // I think this is the same thing as the conditions below

        }


//
//        armPivot.updateLiftPosition();
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("pivot SetPoint", armPivot3Motor.pControllerArmPivot.setPoint);
        telemetry.addData("pivot position", armPivot3Motor.armPivot.getCurrentPosition());
        telemetry.addData("pivot angle",armPivot3Motor.getArmAngle());
        telemetry.addData("pivot pwr", armPivot3Motor.armPivot.getPower());

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("lift left SetPoint", lift3Motor.pControllerLiftLeft.setPoint);
        telemetry.addData("lift right SetPoint", lift3Motor.pControllerLiftRight.setPoint);
        telemetry.addData("lift upper SetPoint", lift3Motor.pControllerUpperLift.setPoint);
        telemetry.addData("lift left position", lift3Motor.liftLeft.getCurrentPosition());
        telemetry.addData("lift right position", lift3Motor.liftRight.getCurrentPosition());
        telemetry.addData("lift upper position", lift3Motor.upperLift.getCurrentPosition());
        telemetry.addData("Average lift ticks", (lift3Motor.upperLift.getCurrentPosition()+lift3Motor.liftRight.getCurrentPosition()+lift3Motor.liftLeft.getCurrentPosition())/3);
        telemetry.addData("Average lift extension", lift3Motor.getLiftExtension());
        telemetry.addData("lift left pwr", lift3Motor.liftLeft.getPower());
        telemetry.addData("lift right pwr", lift3Motor.liftRight.getPower());
        telemetry.addData("lift upper pwr", lift3Motor.upperLift.getPower());
        telemetry.addData("stage", stage);

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

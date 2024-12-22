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
import com.qualcomm.robotcore.util.ElapsedTime;

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
    int x;



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
        lift.initLiftPIDController();
        armPivot.InitArmPivotPIDController();


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
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            armPivot.update(90, 1, 20,0.2, telemetry);
        } else if (gamepad1.left_bumper) {
            armPivot.update(-7, 1, 5, 0.5, telemetry);
        }

        lift.setLiftPower(gamepad1.right_stick_y);


        /*if (gamepad1.dpad_down && armPivot.armPivotRightPosition <0 && armPivot.armPivotLeftPosition <0) {   ///move arm down up and sets controller position

                velocity = 1000;
                armPivot.setArmPivotVelocity(velocity);
                armPivot.setArmPivotPosition();
                lift.setLiftVelocityFromPivotVelocity(velocity);
                lift.setLiftPosition();
                armPivot.pidControllerArmPivotLeft.setSetPointPID(armPivot.armPivotLeftPosition);
                armPivot.pidControllerArmPivotRight.setSetPointPID(armPivot.armPivotRightPosition);
                lift.pidControllerLiftLeft.setSetPointPID(lift.liftLeftPosition);
                lift.pidControllerLiftRight.setSetPointPID(lift.liftRightPosition);

        }
        else if (gamepad1.dpad_up && armPivot.armPivotRightPosition <950 && armPivot.armPivotLeftPosition <950) {  //move arm up and sets controller position


            if(armPivot.armPivotRightPosition <= -748 || armPivot.armPivotLeftPosition <= -748)
            {
                //linear interpolation variables
                int y1 = -1000;
                int y2 = 0;
                int x1 = -748;
                int x2 = -935;
                //current arm position as one variable
                x = (armPivot.armPivotLeftPosition + armPivot.armPivotRightPosition) / 2;

                //linear interpolation formula - will linearly ramp power down as we approach max pivot angle
                velocity = y1 + (x -x1) * ((y2 - y1) / (x2 - x1));
            }
            else {
                velocity = -1000;
            }

            armPivot.setArmPivotVelocity(velocity);
            armPivot.setArmPivotPosition();
            lift.setLiftVelocityFromPivotVelocity(velocity);
            lift.setLiftPosition();
            armPivot.pidControllerArmPivotLeft.setSetPointPID(armPivot.armPivotLeftPosition);
            armPivot.pidControllerArmPivotRight.setSetPointPID(armPivot.armPivotRightPosition);
            lift.pidControllerLiftLeft.setSetPointPID(lift.liftLeftPosition);
            lift.pidControllerLiftRight.setSetPointPID(lift.liftRightPosition);

        }
        else {                                       //uses proportional controller to hold lift in correct spot

            armPivot.updateArmPivotPositionPID();
            lift.updateLiftPositionPID();

        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("arm SetPoint Left", armPivot.pidControllerArmPivotLeft.setPoint);
        telemetry.addData("arm SetPoint Right", armPivot.pidControllerArmPivotRight.setPoint);
        telemetry.addData("Lift SetPoint Left", lift.pidControllerLiftLeft.setPoint);
        telemetry.addData("Lift SetPoint Right", lift.pidControllerLiftRight.setPoint);
        telemetry.addData("arm position Left", armPivot.armPivotLeftPosition);
        telemetry.addData("arm position Right", armPivot.armPivotRightPosition);
        telemetry.addData("arm Left Motor", armPivot.armPivotLeft.getCurrentPosition());
        telemetry.addData("arm Right Motor", armPivot.armPivotRight.getCurrentPosition());
        telemetry.addData("arm Right Motor pwr", armPivot.armPivotRight.getPower());
        telemetry.addData("arm Left Motor pwr", armPivot.armPivotLeft.getPower());
        telemetry.addData("arm Right Motor velocity", armPivot.armPivotRight.getVelocity());
        telemetry.addData("arm Left Motor velocity", armPivot.armPivotLeft.getVelocity());

        telemetry.addData("stage", stage);
        telemetry.addData("arm position averaged ", x);
        telemetry.addData("arm velocity ", velocity);*/

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

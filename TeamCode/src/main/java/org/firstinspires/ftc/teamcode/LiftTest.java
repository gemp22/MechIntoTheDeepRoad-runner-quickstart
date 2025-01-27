package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

import java.util.ArrayList;
import java.util.HashMap;

@TeleOp(name = "LiftTest", group = "Mechbot")
public class LiftTest extends Robot {

    @Override
    public void init() {
        isAuto = false;
        resetEncoders = false;
        AutoSpecimens.pickupOffWall = false;
        super.init();      //Ask Miles what this is?

        drive = new MecanumDrive(hardwareMap, new Pose2d(17.75 / 2, 23.75, Math.PI));  // this initilizes the Odo?
    }

    @Override
    public void start() {
        super.start(); //what is super.start?

        //pawright.setPosition(0.59);

        //droneAndRobotLiftRotator.setDroneAndRobotLiftRotatorPosition(0.8);


        //fourBars.setFourBarPosition(0.4);
        //pixelLift.pControllerPixelLift.setSetPoint(600);
        //startTime = System.currentTimeMillis();
    }

    @Override
    public void stop() {
        super.stop(); //what is super.stop?
        //droneAndRobotLiftRotator.droneAndRobotLiftRotator.setPwmDisable();

        //for rumble added 4/6
        //runtime.reset();
    }

    @Override
    public void mainLoop() {
        super.mainLoop();
        telemetry.addData("arm pivot current angle", armPivot.getArmAngle());
        ButtonPress.giveMeInputs(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.dpad_up,
                gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.right_bumper,
                gamepad1.left_bumper, gamepad1.left_stick_button, gamepad1.right_stick_button,
                gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y, gamepad2.dpad_up,
                gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.right_bumper,
                gamepad2.left_bumper, gamepad2.left_stick_button, gamepad2.right_stick_button);

        lift.setLiftPower(-gamepad1.right_stick_y);
    }
}

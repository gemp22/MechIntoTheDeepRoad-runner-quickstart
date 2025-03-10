package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

@TeleOp
public class ClawTest extends Robot {

    private int servoIndex = 0;
    private double tilt = 0;
    private double twist = 0;
    private double currentServoPosition = 0;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();

        List<Map.Entry<String, Pair<Servo, Double>>> entryServoList = servoMap.entrySet().stream().collect(Collectors.toList());
        Map.Entry<String, Pair<Servo, Double>> entry = entryServoList.get(servoIndex);

        currentServoPosition = entry.getValue().second;
    }

    @Override
    public void mainLoop() {
        super.mainLoop();

        ButtonPress.giveMeInputs(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.dpad_up,
                gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.right_bumper,
                gamepad1.left_bumper, gamepad1.left_stick_button, gamepad1.right_stick_button,
                gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y, gamepad2.dpad_up,
                gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.right_bumper,
                gamepad2.left_bumper, gamepad2.left_stick_button, gamepad2.right_stick_button);

//        if (ButtonPress.isGamepad1_right_bumper_pressed()) {
//            servoIndex += 1;
//
//            List<Map.Entry<String, Pair<Servo, Double>>> entryServoList = servoMap.entrySet().stream().collect(Collectors.toList());
//            Map.Entry<String, Pair<Servo, Double>> entry = entryServoList.get(servoIndex);
//
//            currentServoPosition = entry.getValue().second;
//        } else if (ButtonPress.isGamepad1_left_bumper_pressed()) {
//            servoIndex -= 1;
//
//            List<Map.Entry<String, Pair<Servo, Double>>> entryServoList = servoMap.entrySet().stream().collect(Collectors.toList());
//            Map.Entry<String, Pair<Servo, Double>> entry = entryServoList.get(servoIndex);
//
//            currentServoPosition = entry.getValue().second;
//        }

//        List<Map.Entry<String, Pair<Servo, Double>>> entryServoList = servoMap.entrySet().stream().collect(Collectors.toList());
//        Map.Entry<String, Pair<Servo, Double>> entry = entryServoList.get(servoIndex);

        if (ButtonPress.isGamepad1_a_pressed()) {
            //currentServoPosition += 0.005;
            //Open
            armPivot.intakeJawServo.setPosition(.34);
        } else if (ButtonPress.isGamepad1_b_pressed()) {
            //currentServoPosition -= 0.005;
            //Closed
            armPivot.intakeJawServo.setPosition(.6);
        }


        if (ButtonPress.isGamepad1_dpad_left_pressed()) {
            //currentServoPosition += 0.005;
            //Open
            twist += 0.05;
        } else if (ButtonPress.isGamepad1_dpad_right_pressed()) {
            //currentServoPosition -= 0.005;
            //Closed
            twist -= 0.05;
        }

        armPivot.twist.setPosition(twist);


        if (ButtonPress.isGamepad1_dpad_up_pressed()) {
            //currentServoPosition += 0.005;
            //Open
            tilt += 0.05;
        } else if (ButtonPress.isGamepad2_dpad_down_pressed()) {
            //currentServoPosition -= 0.005;
            //Closed
            tilt -= 0.05;
        }

        armPivot.intakeTilt.setPosition(tilt);

//        entry.getValue().first.setPosition(currentServoPosition);

        //telemetry.addData("Current Servo Tuning", entry.getKey());
        telemetry.addData("jaw: ", armPivot.intakeJawServo.getPosition());
        telemetry.addData("tilt: ", armPivot.intakeTilt.getPosition());
        telemetry.addData("twist: ", armPivot.twist.getPosition());

    }
}

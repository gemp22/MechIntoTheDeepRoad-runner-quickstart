package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

@Autonomous
public class ServoTuning extends Robot {
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
    }

    private int servoIndex = 0;
    private double currentServoPosition = 0;

    @Override
    public void mainLoop() {
        ButtonPress.giveMeInputs(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.dpad_up,
                gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.right_bumper,
                gamepad1.left_bumper, gamepad1.left_stick_button, gamepad1.right_stick_button,
                gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y, gamepad2.dpad_up,
                gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.right_bumper,
                gamepad2.left_bumper, gamepad2.left_stick_button, gamepad2.right_stick_button);

        if (ButtonPress.isGamepad1_right_bumper_pressed()) {
            servoIndex += 1;

            List<Map.Entry<String, Pair<Servo, Double>>> entryServoList = servoMap.entrySet().stream().collect(Collectors.toList());
            Map.Entry<String, Pair<Servo, Double>> entry = entryServoList.get(servoIndex);

            currentServoPosition = entry.getValue().second;
        } else if (ButtonPress.isGamepad1_left_bumper_pressed()) {
            servoIndex -= 1;

            List<Map.Entry<String, Pair<Servo, Double>>> entryServoList = servoMap.entrySet().stream().collect(Collectors.toList());
            Map.Entry<String, Pair<Servo, Double>> entry = entryServoList.get(servoIndex);

            currentServoPosition = entry.getValue().second;
        }

        List<Map.Entry<String, Pair<Servo, Double>>> entryServoList = servoMap.entrySet().stream().collect(Collectors.toList());
        Map.Entry<String, Pair<Servo, Double>> entry = entryServoList.get(servoIndex);

        if (ButtonPress.isGamepad1_a_pressed()) {
            currentServoPosition += 0.005;
        } else if (ButtonPress.isGamepad1_b_pressed()) {
            currentServoPosition -= 0.005;
        }

        entry.getValue().first.setPosition(currentServoPosition);

        telemetry.addData("Current Servo Tuning", entry.getKey());
        telemetry.addData("Current Servo Position", currentServoPosition);
    }
}


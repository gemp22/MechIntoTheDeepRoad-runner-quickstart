package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

@TeleOp
public class ServoTuningSimple extends Robot3Motor {

    private int servoIndex = 0;
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

        if (ButtonPress.isGamepad1_a_pressed()) {
            armPivot3Motor.intakeTilt.setPosition(currentServoPosition+= 0.005);
            armPivot3Motor.intakeTiltTwo.setPosition(currentServoPosition+= 0.005); ;
        } else if (ButtonPress.isGamepad1_b_pressed()) {
            armPivot3Motor.intakeTilt.setPosition(currentServoPosition-= 0.005);
            armPivot3Motor.intakeTiltTwo.setPosition(currentServoPosition-= 0.005);
        }

        if (ButtonPress.isGamepad1_x_pressed()) {
            armPivot3Motor.intakeJawServo.setPosition(currentServoPosition+= 0.005);

        } else if (ButtonPress.isGamepad1_y_pressed()) {
            armPivot3Motor.intakeJawServo.setPosition(currentServoPosition-= 0.005);

        }

        telemetry.addData("tilt servo 2 position", armPivot3Motor.intakeTiltTwo.getPosition());
        telemetry.addData("tilt servo 1 position", armPivot3Motor.intakeTilt.getPosition());
        telemetry.addData("Jaw position", armPivot3Motor.intakeJawServo.getPosition());

    }
}

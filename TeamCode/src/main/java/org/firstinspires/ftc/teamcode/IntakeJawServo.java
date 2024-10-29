package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class IntakeJawServo {
    public ServoImplEx intakeJawServo;

    public IntakeJawServo(HardwareMap hardwareMap) {
        intakeJawServo = hardwareMap.get(ServoImplEx.class, "intakeJaw");
        intakeJawServo.setDirection(Servo.Direction.REVERSE);
        intakeJawServo.setPosition(0.01);
    }
}

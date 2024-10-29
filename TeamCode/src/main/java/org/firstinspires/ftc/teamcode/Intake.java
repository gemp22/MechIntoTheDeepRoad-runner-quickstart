package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    //public DcMotorEx intake; this was from center stage
    public CRServo vexIntake = null;

    public Intake(HardwareMap hardwareMap) {
        //intake = hardwareMap.get(DcMotorEx.class, "intake");
        //intake.setDirection(DcMotor.Direction.REVERSE);
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        vexIntake = (CRServo) hardwareMap.get(CRServo.class, "vexIntake");

    }

    public class IntakeOn implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                //intake.setPower(0.8); this was centerstage
                vexIntake.setPower(-.91);

                initialized = true;
            }

            double pow = vexIntake.getPower();
            packet.put("intake Power", pow);
            return pow <10_000.0;
        }
    }

    public Action intakeOn() {
        return new IntakeOn();
    }

    public class IntakeOff implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                //intake.setPower(0.0);
                vexIntake.setPower(0);
                initialized = true;
            }

            double pow = vexIntake.getPower();
            packet.put("intake Power", pow);
            return pow <10_000.0;
        }
    }
    public Action intakeOff() {
        return new IntakeOff();
    }
}

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    public Servo intakeTwist;
    public Servo intakeTilt;
    private double intakeTwistActionPosition; // variable for RR1.0 actions
    private double intakeTwistPosition; // variable for regular use like on init.. humm may not need since its defined in the method?

    private double intakeTiltActionPosition; // variable for RR1.0 actions
    private double intakeTiltPosition; // variable for regular use like on init.. humm may not need since its defined in the method?

    public Wrist(HardwareMap hardwareMap) {
        intakeTwist = hardwareMap.get(Servo.class, "intakeTwist");
        intakeTilt = hardwareMap.get(Servo.class, "intakeTilt");
    }

    /////////////////////////   IntakeTwist   /////////////////////////////////////
    public class IntakeTwistActionSet implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

                intakeTwist.setPosition(intakeTwistActionPosition);
                return false;
        }
    }
    public class IntakeTwistActionOpen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            intakeTwist.setPosition(.66);
            return false;
        }
    }
    public class IntakeTwistActionClose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            intakeTilt.setPosition(.16);
            return false;
        }
    }

    public Action intakeTwistActionSet(double intakeTwistActionSetPosition) {  // this method is for use in RR trajectories
        intakeTwistActionPosition = intakeTwistActionSetPosition;
        return new IntakeTwistActionSet();
    }
    public Action intakeTwistActionOpen() {  // this method is for use in RR trajectories

        return new IntakeTwistActionOpen();
    }
    public Action intakeTwistActionClose() {  // this method is for use in RR trajectories

        return new IntakeTwistActionClose();
    }

    public void setIntakeTwistPosition(double intakeTwistPosition){  // this method is for basic use like init...
        intakeTwist.setPosition(intakeTwistPosition);
    }
/////////////////////////   Back Gripper   /////////////////////////////////////
public class IntakeTiltActionSet implements Action {

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {

            intakeTilt.setPosition(intakeTiltActionPosition);
            return false;

    }
}

    public class IntakeTiltActionOpen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            intakeTilt.setPosition(.71);
            return false;

        }
    }
    public class BackGripperActionClosed implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            intakeTilt.setPosition(.21);
            return false;

        }
    }

    public Action intakeTiltActionSet(double intakeTiltActionSetPosition) {  // this method is for use in RR trajectories
        intakeTiltActionPosition = intakeTiltActionSetPosition;
        return new IntakeTiltActionSet();
    }

    public Action intakeTiltActionOpen() {  // this method is for use in RR trajectories

        return new IntakeTiltActionOpen();
    }
    public Action intakeTiltActionClose() {  // this method is for use in RR trajectories

        return new IntakeTwistActionClose();
    }

    public void setIntakeTiltPosition(double intakeTiltPosition){  // this method is for basic use like init...
       intakeTilt.setPosition(intakeTiltPosition);
    }

}

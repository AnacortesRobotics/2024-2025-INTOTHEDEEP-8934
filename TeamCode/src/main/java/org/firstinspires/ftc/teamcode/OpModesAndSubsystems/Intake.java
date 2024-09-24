package org.firstinspires.ftc.teamcode.OpModesAndSubsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.*;

// intake class
public class Intake {
    private DcMotorEx intakeMotor;
    //TODO: Make these for the rev encoders on the end of the intake
    //double intakePosLeft = intakeExtensionleft
    //double intakePosRight = intakeExtensionRight

    Servo intakeFlip, grabber, grabberWrist, intakeExtensionleft, intakeExtensionRight;

    public Intake(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeFlip = hardwareMap.get(Servo.class, "intakeFlip");
        grabber = hardwareMap.get(Servo.class, "grabber");
        grabberWrist = hardwareMap.get(Servo.class, "grabberWrist");
        intakeExtensionleft = hardwareMap.get(Servo.class, "intakeExtensionLeft");
        intakeExtensionRight = hardwareMap.get(Servo.class, "intakeExtensionRight");
    }

    /*
    public class IntakeOut implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                liftLeft.setPower(0.8);
                liftRight.setPower(0.8);
                initialized = true;
            }

            //packet.put("intakePosLeft", intakePosLeft);
            //packet.put("intakePosRight", intakePosRight);
            //TODO: Change these encoded (I think) values to be real for lift extension (probably really small)
            if (posLeft < 3000.0 && posRight < 3000.0) {
                return true;
            } else {
                liftLeft.setPower(0);
                liftRight.setPower(0);
                return false;
            }
        }
    }

    public class intakeIn implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                liftLeft.setPower(-0.8);
                liftRight.setPower(-0.8);
                initialized = true;
            }

            packet.put("liftPosLeft", posLeft);
            packet.put("liftPosRight", posRight);
            //TODO: Change these encoded (I think) values to be real for lift extension (probably really small)
            if (posLeft > 100.0 && posRight > 100.0) {
                return true;
            } else {
                liftLeft.setPower(0);
                liftRight.setPower(0);
                return false;
            }
        }
    }

     */
}

package org.firstinspires.ftc.teamcode.OpModesAndSubsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

// lift class
public class Lift {
    private DcMotorEx liftLeft, liftRight;
    double posLeft = liftLeft.getCurrentPosition();
    double posRight = liftRight.getCurrentPosition();
    public Lift(HardwareMap hardwareMap) {
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public class LiftUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                liftLeft.setPower(0.8);
                liftRight.setPower(0.8);
                initialized = true;
            }

            packet.put("liftPosLeft", posLeft);
            packet.put("liftPosRight", posRight);
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

    public class LiftDown implements Action {
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
}
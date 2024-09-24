package org.firstinspires.ftc.teamcode.OpModesAndSubsystems;


import androidx.annotation.NonNull;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import org.firstinspires.ftc.teamcode.Constants.RobotConstants8934;
import org.firstinspires.ftc.teamcode.OpModesAndSubsystems.robot;
import org.firstinspires.ftc.teamcode.OpModesAndSubsystems.robot.COLOR;

// intake class
public class Intake {
    private robot robot;
    private DcMotorEx intakeMotor;
    Servo intakeFlip, grabber, grabberWrist, intakeExtensionleft, intakeExtensionRight;


    //TODO: Make these for the rev encoders on the end of the intake
    double intakePosLeft;
    double intakePosRight;

    public Intake(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //TODO: Figure out which way this runs
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeFlip = hardwareMap.get(Servo.class, "intakeFlip");
        intakeExtensionleft = hardwareMap.get(Servo.class, "intakeExtensionLeft");
        intakeExtensionRight = hardwareMap.get(Servo.class, "intakeExtensionRight");
        //TODO: Check/tune these so that positive position will be extend forward
        intakeExtensionleft.setDirection(Servo.Direction.FORWARD);
        intakeExtensionRight.setDirection(Servo.Direction.REVERSE);
    }


    private boolean IntakeMove(double targetServoPos, double targetPos) {
        //intakePosLeft = rev encoder left pos
        //intakePosRight = rev encoder right pos
        double intakePosAvg = ((intakePosLeft + intakePosRight) / 2);

        intakeExtensionleft.setPosition(targetServoPos);
        intakeExtensionRight.setPosition(targetServoPos);
        //TODO: Figure out if I want to make one of these for each color (red/blue), rely on vision to target samples, or do any color (RBY)
        while (intakePosAvg < targetPos) {
            // not there yet, keep trying
            return true;
        }
        return false;
    }
    private boolean IntakeGrab(Enum targetSampleColor, double failLoopsAllowed) {
        double failCount = 0;
        //TODO: tune this value
        intakeFlip.setPosition(0.5);
        //TODO: tune this value
        intakeMotor.setPower(0.9);
        failCount ++;

        //TODO: Figure out if I want to make one of these for each color (red/blue), rely on vision to target samples, or do any color (RBY)
        //TODO: Figure out if this failcount works and tune failcount # required to move on without sample
        if (robot.CURRENT_COLOR == targetSampleColor || failCount >= failLoopsAllowed) {
            //TODO: TUNE THIS VALUE
            intakeMotor.setPower(0);
            //TODO: TUNE THIS VALUE
            intakeFlip.setPosition(0);
            intakeExtensionleft.setPosition(0); //fully in, to tune
            intakeExtensionRight.setPosition(0); // fully in, to tune
            return false;
        } else {
            return true;
        }
    }

    public boolean ExtendOverBarrierIntake() {
        //TODO: Tune this value for barrier distance
        double targetServoPos = 0.0;
        //TODO: change this target pose to align with whatever just over barrier is with REV encoder
        double targetPose = 0.0;
        boolean intakeIsThere = false;
        while (!intakeIsThere) {
            intakeIsThere = IntakeMove(targetServoPos, targetPose);
        }
        //TODO: only try n times (timeout)
        return intakeIsThere;
    }

    public Action AutoExtendOverBarrierIntake() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("intakePosLeft", intakePosLeft);
                packet.put("intakePosRight", intakePosRight);
                return !ExtendOverBarrierIntake();
            }
        };
    }

    public boolean OverBarrierRedSampleIntake() {
        //TODO: Tune this value for over barrier distance
        double targetServoPos = 0.0;
        //TODO: change this target pose to align with whatever over barrier distance is with REV encoder
        double targetPose = 0.0;
        Enum targetSampleColor = COLOR.RED;
        double failLoopsAllowed = 300;
        boolean intakeIsThere = false;
        boolean sampleGrabbed = false;
        while (!intakeIsThere) {
            intakeIsThere = IntakeMove(targetServoPos, targetPose);
        }
        //TODO: check if this will complete before is returns true (I think it will)
        if (intakeIsThere) {
            sampleGrabbed = IntakeGrab(targetSampleColor, failLoopsAllowed);
        }
        return sampleGrabbed;
    }

    public Action AutoOverBarrierRedSampleIntake() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("intakePosLeft", intakePosLeft);
                packet.put("intakePosRight", intakePosRight);
                return !OverBarrierRedSampleIntake();
            }
        };
    }

    public boolean ExtendMaxIntake() {
        //TODO: Tune this value for max distance
        double targetServoPos = 0.0;
        //TODO: change this target pose to align with whatever max distance is with REV encoder
        double targetPose = 0.0;
        boolean intakeIsThere = false;
        while (!intakeIsThere) {
            intakeIsThere = IntakeMove(targetServoPos, targetPose);
        }
        //TODO: only try n times (timeout)
        return intakeIsThere;
    }

    public Action AutoExtendMaxIntake() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("intakePosLeft", intakePosLeft);
                packet.put("intakePosRight", intakePosRight);
                return !ExtendMaxIntake();
            }
        };
    }
}


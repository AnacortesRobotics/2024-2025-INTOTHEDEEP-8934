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


// lift class
public class Lift {
    private DcMotorEx liftLeft, liftRight;
    double posLeft;
    double posRight;
    Servo grabber, grabberWrist;

    public Lift(HardwareMap hardwareMap) {
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //TODO: set one to reverse - figure out which one
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        grabber = hardwareMap.get(Servo.class, "grabber");
        grabberWrist = hardwareMap.get(Servo.class, "grabberWrist");
    }

    private boolean LiftMove(double targetReference, double targetPose) {
        posLeft = liftLeft.getCurrentPosition();
        posRight = liftRight.getCurrentPosition();
        // Elapsed timer class from SDK
        ElapsedTime timer = new ElapsedTime();

        // Proportional Integral Derivative Controller w/ Low pass filter and anti-windup
        //TODO: Tune these (applies to all lift functions)
        double Kp = 0.3;
        double Ki = 0;
        double Kd = 0;

        double integralSum = 0;

        double lastError = 0;
        //TODO: TUNE THIS VALUE - COMPLETELY EXPERIMENTAL
        double maxIntegralSum = 2.5;

        double a = 0.8; // a can be anything from 0 < a < 1
        double previousFilterEstimate = 0;
        double currentFilterEstimate = 0;


        //TODO: change reference to needed encoder pos (I think)
        double reference = targetReference;
        double lastReference = reference;
        //TODO: Figure out if these need to be the same or if this PID even works :P
        //TODO: Change these encoded (I think) values to be real for lift extension (probably really small)


        // obtain the encoder position
        double encoderPosition = ((posLeft + posRight) / 2);
        // calculate the error
        double error = reference - encoderPosition;

        double errorChange = (error - lastError);

        // filter out hight frequency noise to increase derivative performance
        currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        // rate of change of the error
        double derivative = currentFilterEstimate / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());


        // max out integral sum
        if (integralSum > maxIntegralSum) {
            integralSum = maxIntegralSum;
        }

        if (integralSum < -maxIntegralSum) {
            integralSum = -maxIntegralSum;
        }

        // reset integral sum upon setpoint changes
        if (reference != lastReference) {
            integralSum = 0;
        }

        double liftPowerOut = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        liftLeft.setPower(liftPowerOut);
        liftRight.setPower(liftPowerOut);

        lastError = error;

        lastReference = reference;
        // reset the timer for next time
        timer.reset();

        if (posLeft < targetPose && posRight < targetPose){
            return true;
        }
        else {
            liftLeft.setPower(0);
            liftRight.setPower(0);
            return false;
        }
    }


    public boolean GoToHighBasket(){
        double targetReference = 0.0;
        double targetPose = 0.0;
        boolean liftIsThere = false;
        while (!liftIsThere) {
            liftIsThere = LiftMove(targetReference, targetPose);
        }
        //TODO: only try n times (timeout)
        return liftIsThere;
    }

    public Action AutoGoToHighBasket(){
       return new Action() {
           @Override
           public boolean run(@NonNull TelemetryPacket packet) {
               packet.put("liftPosLeft", posLeft);
               packet.put("liftPosRight", posRight);
               return ! GoToHighBasket();
           }
       };
    }
}
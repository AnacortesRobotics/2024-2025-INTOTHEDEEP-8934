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
public class Grabber {

    //TODO: TUNE THESE BELOW VALUES
    double fullGrab = 0.8; //closed pos for grabber
    double fullRelease = 0.2; //open pos for grabber
    double swingRestingPos = 0.65; //0 pos for swing
    double wristRestingPos = 0.35; //0 pos for wrist
    double grabTimeOffset = 300; //time (ms) it take to grab a sample/specimen safely
    double releaseTimeOffset = 300; //time (ms) it takes to release a sample/specimen safely
    private enum LOCATION {
        INTAKE,
        WALL,
        FLOOR
    }
    LOCATION TARGET_LOCATION;

    private enum OUTPUT_LOCATION {
        HIGH_CHAMBER,
        LOW_CHAMBER,
        HIGH_BASKET,
        LOW_BASKET,
        OBS_ZONE
    }
    OUTPUT_LOCATION TARGET_LOCATION_DEPOSIT;
    private robot robot;
    double time;
    Servo outputSwing, grabber, grabberWrist;
    public Grabber(HardwareMap hardwareMap) {
        outputSwing = hardwareMap.get(Servo.class, "outputSwing");
        grabber = hardwareMap.get(Servo.class, "grabber");
        grabberWrist = hardwareMap.get(Servo.class, "grabberWrist");
    }


    private boolean Grab(Enum TARGET_LOCATION, double customWristPos, double customSwingPos, double customTimeBeforeGrab) {
        //TODO: test is returning when we start grabbing works, like if it gets knocked out or smth

        if (TARGET_LOCATION == LOCATION.INTAKE) {
            ElapsedTime timer = new ElapsedTime();
            time = timer.milliseconds();
            //TODO: tune these servo positions
            outputSwing.setPosition(0.2);
            grabberWrist.setPosition(0.4);
            //TODO: tune this time value to how long it takes other servos to get to position, then grab element
            if (time >= 500 ) {
                grabber.setPosition(fullGrab);
                if(time >= 500 + grabTimeOffset) {
                    outputSwing.setPosition(swingRestingPos);
                    grabberWrist.setPosition(wristRestingPos);
                    return false;
                }
            }
            else {
                return true;
            }
        }
        else if (TARGET_LOCATION == LOCATION.WALL) {
            ElapsedTime timer = new ElapsedTime();
            time = timer.milliseconds();
            //TODO: tune these servo positions
            outputSwing.setPosition(0.8);
            grabberWrist.setPosition(0.1);
            //TODO: tune this time value to how long it takes other servos to get to position, then grab element
            if (time >= 750 ) {
                grabber.setPosition(fullGrab);
                if(time >= 750 + grabTimeOffset) {
                    outputSwing.setPosition(swingRestingPos);
                    grabberWrist.setPosition(wristRestingPos);
                    return false;
                }
            }
            else {
                return true;
            }
        }
        else if (TARGET_LOCATION == LOCATION.FLOOR) {
            ElapsedTime timer = new ElapsedTime();
            time = timer.milliseconds();
            //TODO: tune these servo positions
            outputSwing.setPosition(1);
            grabberWrist.setPosition(0.5);
            //TODO: tune this time value to how long it takes other servos to get to position, then grab element
            if (time >= 1000 ) {
                grabber.setPosition(fullGrab);
                if(time >= 1000 + grabTimeOffset) {
                    outputSwing.setPosition(swingRestingPos);
                    grabberWrist.setPosition(wristRestingPos);
                    return false;
                }
            }
            else {
                return true;
            }
        }
        else {
            ElapsedTime timer = new ElapsedTime();
            time = timer.milliseconds();
            //TODO: tune these servo positions
            outputSwing.setPosition(customSwingPos);
            grabberWrist.setPosition(customWristPos);
            //TODO: tune this time value to how long it takes other servos to get to position, then grab element
            if (time >= customTimeBeforeGrab ) {
                grabber.setPosition(fullGrab);
                if(time >= customTimeBeforeGrab + grabTimeOffset) {
                    outputSwing.setPosition(swingRestingPos);
                    grabberWrist.setPosition(wristRestingPos);
                    return false;
                }
            }
            else {
                return true;
            }
        }
        //TODO: should I do this? seems unnecessary because it must return true if all else fails
        return false;
    }

    private boolean Deposit(Enum TARGET_LOCATION_DEPOSIT, double customWristPos, double customSwingPos, double customTimeBeforeRelease) {
        //TODO: test is returning when we start grabbing works, like if it gets knocked out or smth

        if (TARGET_LOCATION_DEPOSIT == OUTPUT_LOCATION.HIGH_BASKET) {
            ElapsedTime timer = new ElapsedTime();
            time = timer.milliseconds();
            //TODO: tune these servo positions
            outputSwing.setPosition(0.3);
            grabberWrist.setPosition(0.5);
            //TODO: tune this time value to how long it takes other servos to get to position
            if (time >= 500 ) {
                grabber.setPosition(fullRelease);
                if(time >= 500 + releaseTimeOffset) {
                    outputSwing.setPosition(swingRestingPos);
                    grabberWrist.setPosition(wristRestingPos);
                    return false;
                }
            }
            else {
                return true;
            }
        }
        else if (TARGET_LOCATION_DEPOSIT == OUTPUT_LOCATION.LOW_BASKET) {
            ElapsedTime timer = new ElapsedTime();
            time = timer.milliseconds();
            //TODO: tune these servo positions
            outputSwing.setPosition(0.35);
            grabberWrist.setPosition(0.55);
            //TODO: tune this time value to how long it takes other servos to get to position
            if (time >= 400 ) {
                grabber.setPosition(fullRelease);
                if(time >= 400 + releaseTimeOffset) {
                    outputSwing.setPosition(swingRestingPos);
                    grabberWrist.setPosition(wristRestingPos);
                    return false;
                }
            }
            else {
                return true;
            }
        }
        else if (TARGET_LOCATION_DEPOSIT == OUTPUT_LOCATION.HIGH_CHAMBER) {
            ElapsedTime timer = new ElapsedTime();
            time = timer.milliseconds();
            //TODO: tune these servo positions
            outputSwing.setPosition(0.22);
            grabberWrist.setPosition(0.35);
            //TODO: tune this time value to how long it takes other servos to get to position
            if (time >= 600 ) {
                grabber.setPosition(fullRelease);
                if(time >= 600 + releaseTimeOffset) {
                    outputSwing.setPosition(swingRestingPos);
                    grabberWrist.setPosition(wristRestingPos);
                    return false;
                }
            }
            else {
                return true;
            }
        }
        else if (TARGET_LOCATION_DEPOSIT == OUTPUT_LOCATION.LOW_CHAMBER) {
            ElapsedTime timer = new ElapsedTime();
            time = timer.milliseconds();
            //TODO: tune these servo positions
            outputSwing.setPosition(0.15);
            grabberWrist.setPosition(0.8);
            //TODO: tune this time value to how long it takes other servos to get to position
            if (time >= 370 ) {
                grabber.setPosition(fullRelease);
                if(time >= 370 + releaseTimeOffset) {
                    outputSwing.setPosition(swingRestingPos);
                    grabberWrist.setPosition(wristRestingPos);
                    return false;
                }
            }
            else {
                return true;
            }
        }
        else if (TARGET_LOCATION_DEPOSIT == OUTPUT_LOCATION.OBS_ZONE) {
            ElapsedTime timer = new ElapsedTime();
            time = timer.milliseconds();
            //TODO: tune these servo positions
            outputSwing.setPosition(0.1);
            grabberWrist.setPosition(0.1);
            //TODO: tune this time value to how long it takes other servos to get to position
            if (time >= 850 ) {
                grabber.setPosition(fullRelease);
                if(time >= 850 + releaseTimeOffset) {
                    outputSwing.setPosition(swingRestingPos);
                    grabberWrist.setPosition(wristRestingPos);
                    return false;
                }
            }
            else {
                return true;
            }
        }
        else {
            ElapsedTime timer = new ElapsedTime();
            time = timer.milliseconds();
            //TODO: tune these servo positions
            outputSwing.setPosition(customSwingPos);
            grabberWrist.setPosition(customWristPos);
            //TODO: tune this time value to how long it takes other servos to get to position, then grab element
            if (time >= customTimeBeforeRelease) {
                grabber.setPosition(fullGrab);
                if(time >= customTimeBeforeRelease + releaseTimeOffset) {
                    outputSwing.setPosition(swingRestingPos);
                    grabberWrist.setPosition(wristRestingPos);
                    return false;
                }
            }
            else {
                return true;
            }
        }
        //TODO: should I do this? seems unnecessary because it must return true if all else fails
        return false;
    }

    public boolean GrabFromIntake(){
        boolean sampleGrabbed = false;
        while (!sampleGrabbed) {
            sampleGrabbed = Grab(LOCATION.INTAKE, 0, 0, 0);
        }
        //TODO: only try n times (timeout)
        return sampleGrabbed;
    }

    public Action AutoGrabFromIntake(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("time", time);
                return ! GrabFromIntake();
            }
        };
    }

    public boolean GrabFromWall(){
        boolean sampleGrabbed = false;
        while (!sampleGrabbed) {
            sampleGrabbed = Grab(LOCATION.WALL, 0, 0, 0);
        }
        //TODO: only try n times (timeout)
        return sampleGrabbed;
    }

    public Action AutoGrabFromWall(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("time", time);
                return ! GrabFromIntake();
            }
        };
    }

    public boolean GrabFromFloor(){
        boolean sampleGrabbed = false;
        while (!sampleGrabbed) {
            sampleGrabbed = Grab(LOCATION.FLOOR, 0, 0, 0);
        }
        //TODO: only try n times (timeout)
        return sampleGrabbed;
    }

    public Action AutoGrabFromFloor(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("time", time);
                return ! GrabFromIntake();
            }
        };
    }
    public boolean DepositToHighBasket(){
        boolean sampleDeposited = false;
        while (!sampleDeposited) {
            sampleDeposited = Deposit(OUTPUT_LOCATION.HIGH_BASKET, 0, 0, 0);
        }
        //TODO: only try n times (timeout)
        return sampleDeposited;
    }

    public Action AutoDepositToHighBasket(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("time", time);
                return ! DepositToHighBasket();
            }
        };
    }

    public boolean DepositToLowBasket(){
        boolean sampleDeposited = false;
        while (!sampleDeposited) {
            sampleDeposited = Deposit(OUTPUT_LOCATION.LOW_BASKET, 0, 0, 0);
        }
        //TODO: only try n times (timeout)
        return sampleDeposited;
    }

    public Action AutoDepositToLowBasket(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("time", time);
                return ! DepositToLowBasket();
            }
        };
    }

    public boolean DepositToHighChamber(){
        boolean sampleDeposited = false;
        while (!sampleDeposited) {
            sampleDeposited = Deposit(OUTPUT_LOCATION.HIGH_CHAMBER, 0, 0, 0);
        }
        //TODO: only try n times (timeout)
        return sampleDeposited;
    }

    public Action AutoDepositToHighChamber(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("time", time);
                return ! DepositToHighChamber();
            }
        };
    }

    public boolean DepositToLowChamber(){
        boolean sampleDeposited = false;
        while (!sampleDeposited) {
            sampleDeposited = Deposit(OUTPUT_LOCATION.LOW_CHAMBER, 0, 0, 0);
        }
        //TODO: only try n times (timeout)
        return sampleDeposited;
    }

    public Action AutoDepositToLowChamber(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("time", time);
                return ! DepositToLowChamber();
            }
        };
    }

    public boolean DepositToObsZone(){
        boolean sampleDeposited = false;
        while (!sampleDeposited) {
            sampleDeposited = Deposit(OUTPUT_LOCATION.OBS_ZONE, 0, 0, 0);
        }
        //TODO: only try n times (timeout)
        return sampleDeposited;
    }

    public Action AutoDepositToObsZone(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("time", time);
                return ! DepositToObsZone();
            }
        };
    }


}

package org.firstinspires.ftc.teamcode.OpModesAndSubsystems;

import android.annotation.SuppressLint;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Vision.visionManager;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.OpModesAndSubsystems.robot.COLOR;

import static org.firstinspires.ftc.teamcode.Constants.FieldConstants8934.*;

import com.acmerobotics.roadrunner.ftc.Actions;

import java.lang.Math;

// this is a generic auto mode for Into the Deep.
// the red and blue alliance game configurations are set by the child classes which inherit from this.
public class autoBase extends LinearOpMode {
    final double DESIRED_DISTANCE = 8; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.75 / 20;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.5 / 30;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.5 / 35;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)

    public enum ALLIANCE {
        RED,
        BLUE
    }

    public enum SIDE {
        BASKET,
        OBS
    }

    protected SIDE CURRENT_SIDE;

    protected ALLIANCE THIS_ALLIANCE;

    private int TeamPropLocation = -1; // this means it is not detected
    private AprilTagDetection targetAprilTag; // Used to hold the data for a detected AprilTag
    private robot robot;
    private visionManager visionManager;
    private boolean arrivedAtAprilTag = false;
    double lastKnownRangeToAprilTag = 0;

    // TODO: These fields might make more sense in your constants classes
    private static Vector2d addTest = new Vector2d(4, 1);
    //public Pose2d STACK3_POSITION = new Pose2d(pixelStack3.plus(addTest), Math.toRadians(180));
    //public Vector2d STACK1_POSITION_VECTOR2D = (pixelStack1.plus(addTest));
    private static Vector2d boardAddCenter = new Vector2d(-5.5, 0);
    private static Vector2d boardAddLeft = new Vector2d(-6, 6.5);
    private static Vector2d blueBoardScoringLeft = new Vector2d(54.5, 43.5);
    private static Vector2d blueBoardScoringMiddle = new Vector2d(54.5, 41);
    private static Vector2d blueBoardScoringRight = new Vector2d(55, 30.75);
    //private static Pose2d blueBoardScoring = new Pose2d(blueBoard.plus(boardAddCenter), Math.toRadians(180));
    private static Pose2d centerField = new Pose2d(0, 0, Math.toRadians(0));
    private static Vector2d centerFieldVector = new Vector2d(0, 6);
    private static Pose2d centerFieldOffset = new Pose2d(-36, -2, Math.toRadians(180));
    private static Pose2d centerFieldOffset2 = new Pose2d(-4, -2, Math.toRadians(180));

    // TODO: Don't suppress the indentation warnings lol (CTRL-ALT-L will auto-format)
    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {

        //TODO: Is this if-statement necessary? (Seems like this initialization should always occur)
        if (opModeInInit()) {
            initializeGameConfig();
            initializeSystem();
            robot.imu.resetYaw();
        }

        MecanumDrive drive;
        //TODO: FIX STARTING LOCATIONS FOR OBS
        if (THIS_ALLIANCE == ALLIANCE.BLUE) {
            if (CURRENT_SIDE == SIDE.BASKET) {
                drive = new MecanumDrive(hardwareMap, basketBlueStarting);
            } else {
                drive = new MecanumDrive(hardwareMap, frontstageBlueStarting);
            }

        } else {
            if (CURRENT_SIDE == SIDE.BASKET) {
                drive = new MecanumDrive(hardwareMap, basketRedStarting);
            } else {
                drive = new MecanumDrive(hardwareMap, frontstageRedStarting);
            }

        }

        if (THIS_ALLIANCE == ALLIANCE.BLUE) {
            if (CURRENT_SIDE == SIDE.BASKET) {
                telemetry.addLine("sigma");
                telemetry.update();
            } else if (CURRENT_SIDE == SIDE.OBS) {
                telemetry.addLine("we cooked");
                telemetry.update();
            } else {
                telemetry.addLine("No Side Selected... Uh Oh");
                telemetry.update();
            }
        } else if (THIS_ALLIANCE == ALLIANCE.RED) {
            if (CURRENT_SIDE == SIDE.BASKET) {
                telemetry.addLine("why we doing red?");
                telemetry.update();
            } else if (CURRENT_SIDE == SIDE.OBS) {
                telemetry.addLine("we cooked");
                telemetry.update();
            } else {
                telemetry.addLine("No Side Selected... Uh Oh");
                telemetry.update();
            }
        }

        Action toClip;
        Action toBasket;
        Action toYSample1;
        Action toYSample2;
        Action toYSample3;
        Action toPark;
        Action trajectoryActionChosen = null;
        toClip = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(autoClipBlue, Math.toRadians(270))
                .build();

        toBasket = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(blueBasket, Math.toRadians(48))
                .build();

        toYSample1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(blueYSample1, Math.toRadians(253))
                .build();

        toYSample2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(blueYSample2, Math.toRadians(285))
                .build();

        toYSample3 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(blueYSample3, Math.toRadians(318))
                .build();

        toPark = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(48, 12))
                .build();

        waitForStart(); // ready to rock

        /*
        if (this.TeamPropLocation == -1) {
            telemetry.addLine("Seeking prop, please wait...");
            telemetry.update();
        }
        this.TeamPropLocation = visionManager.getDetectedSpikeMark();
        telemetry.clearAll();
        telemetry.addData("detected prop = %d", this.TeamPropLocation);
        telemetry.update();
        */

        //TODO: ACTUALLY CREATE 4 AUTOS, IT JUST RUNS BELOW SCRIPT (I THINK)
        /*
        Actions.runBlocking(
                new SequentialAction(
                        toClip,
                        new ParallelAction(
                                toBasket,
                                robot.lift.AutoGoToHighBasket()
                        ),
                        toYSample1,
                        new ParallelAction(
                                toBasket,
                                robot.lift.AutoGoToHighBasket()
                        ),
                        toYSample2,
                        new ParallelAction(
                                toBasket,
                                robot.lift.AutoGoToHighBasket()
                        ),
                        toYSample3,
                        new ParallelAction(
                                toBasket,
                                robot.lift.AutoGoToHighBasket()
                        ),
                        toPark
                        //TODO: Should this go here? (I don't think so)
                        //if (isStopRequested()) return;
                )
        );
        */
        /*
        if (robot.CURRENT_COLOR == COLOR.BLUE) {
            telemetry.clearAll();
            telemetry.addLine("BLUE DETECTED :)");
            telemetry.update();
            robot.roboNap(5000);
            //drive.followTrajectory(goUnderTrussFromStack);
        } else {
            telemetry.clearAll();
            telemetry.addLine("NO BLUE DETECTION... Uh Oh");
            telemetry.update();
        }
        if (!arrivedAtAprilTag)
        {
            if (THIS_ALLIANCE == CenterStageAutoBackstage.ALLIANCE.RED) {
                driveToAprilTag(5);
            } else if (THIS_ALLIANCE == CenterStageAutoBackstage.ALLIANCE.BLUE) {
                driveToAprilTag(2);
            }
        }
        */


        while (opModeIsActive()) {
        }
    }

    private void initializeSystem() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //visionManager = new visionManager(THIS_ALLIANCE, hardwareMap.get(WebcamName.class, "Webcam 1"));
        robot = new robot(this, hardwareMap);
        telemetry.addData("auto", "initializeSystem() complete");
    }

    // empty because the subclass is supposed to implement it
    public void initializeGameConfig() {
    }


    private void driveToAprilTag(int DetectedPropZone) {
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        targetAprilTag = visionManager.getDetectedAprilTag(DetectedPropZone);

        if (targetAprilTag != null) {
            // "error" here refers to the remaining distance to target.
            double rangeError = (targetAprilTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = targetAprilTag.ftcPose.bearing;
            double yawError = targetAprilTag.ftcPose.yaw;

            if (rangeError <= DESIRED_DISTANCE && rangeError <= lastKnownRangeToAprilTag) {
                arrivedAtAprilTag = true;
            } else if (rangeError > lastKnownRangeToAprilTag) {
                lastKnownRangeToAprilTag = rangeError;
            }

            // TODO: Consider moving this movement code into it's own function in robot
            //       (Function accepts target distances and angles, uses a loop, finds
            //        approximate errors using encoders/imu values and uses the
            //        proportional speed control below).
            //       Even fancier (perhaps overkill), the function could accept an object
            //       with getError functions, reducing drift in the loop.
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            // Apply desired axes of motion to the drivetrain.
            robot.moveRobot(drive, strafe, turn);
            robot.roboNap(20);
            telemetry.addData("auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
    }
}
package org.firstinspires.ftc.teamcode.OpModesAndRobots;

import android.annotation.SuppressLint;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.visionManager;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.Vision.visionManager;
import org.firstinspires.ftc.teamcode.OpModesAndRobots.robot;
import org.firstinspires.ftc.teamcode.OpModesAndRobots.robot.COLOR;
import static org.firstinspires.ftc.teamcode.Constants.FieldConstants8934.*;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// this is a generic auto mode for CenterStage Backstage.
// the red and blue alliance game configurations are set by the child classes which inherit from this.
public class autoBase extends LinearOpMode {
    final double DESIRED_DISTANCE = 8; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.75/20;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.5/30;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.5/35;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)

    public enum ALLIANCE {
        RED,
        BLUE
    }

    public enum STAGE {
        FRONT,
        BACK
    }

    protected STAGE STAGE_LOCATION;

    protected ALLIANCE THIS_ALLIANCE;

    private int TeamPropLocation = -1; // this means it is not detected
    private AprilTagDetection targetAprilTag; // Used to hold the data for a detected AprilTag
    private robot robot;
    private visionManager visionManager;
    private boolean arrivedAtAprilTag = false;
    double lastKnownRangeToAprilTag = 0;
    private static Vector2d addTest = new Vector2d(4,1);
    public Pose2d STACK3_POSITION = new Pose2d(pixelStack3.plus(addTest), Math.toRadians(180));
    public Vector2d STACK1_POSITION_VECTOR2D = (pixelStack1.plus(addTest));
    private static Vector2d boardAddCenter = new Vector2d(-5.5,0);
    private static Vector2d boardAddLeft = new Vector2d(-6,6.5);
    private static Vector2d blueBoardScoringLeft = new Vector2d(54.5,43.5);
    private static Vector2d blueBoardScoringMiddle = new Vector2d(54.5,41);
    private static Vector2d blueBoardScoringRight = new Vector2d(55,30.75);
    private static Pose2d blueBoardScoring = new Pose2d(blueBoard.plus(boardAddCenter), Math.toRadians(180));
    private static Pose2d centerField = new Pose2d(0, 0, Math.toRadians(0));
    private static Vector2d centerFieldVector = new Vector2d(0,6);
    private static Pose2d centerFieldOffset = new Pose2d(-36,-2,Math.toRadians(180));
    private static Pose2d centerFieldOffset2 = new Pose2d(-4,-2,Math.toRadians(180));


    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {
        if (opModeInInit()) {
            initializeGameConfig();
            initializeSystem();
            robot.imu.resetYaw();
        }

        MecanumDrive drive;

        if (THIS_ALLIANCE == ALLIANCE.BLUE)
        {
            if (STAGE_LOCATION == STAGE.BACK)
            {
                drive = new MecanumDrive(hardwareMap,backstageBlueStarting);
            }
            else
            {
                drive = new MecanumDrive(hardwareMap,frontstageBlueStarting);
            }

        }
        else
        {
            if (STAGE_LOCATION == STAGE.BACK)
            {
                drive = new MecanumDrive(hardwareMap,backstageRedStarting);
            }
            else
            {
                drive = new MecanumDrive(hardwareMap,frontstageRedStarting);
            }

        }




        waitForStart(); // ready to rock

        Action goToLeftTape;
        Action goToMiddleTape;
        Action goToRightTape;
        Action trajectoryActionChosen = null;
        Action trajectoryActionCloseOut;

        goToLeftTape = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(29,33.5), Math.toRadians(180))
                .build();

        goToMiddleTape = drive.actionBuilder(drive.pose)
                .lineToYConstantHeading(31.75)
                .build();

        goToRightTape = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(7, 33.5), Math.toRadians(180))
                .build();

        trajectoryActionCloseOut = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(48, 12))
                .build();




        //if (isStopRequested()) return;


        if (this.TeamPropLocation == -1) {
            telemetry.addLine("Seeking prop, please wait...");
            telemetry.update();
        }
        this.TeamPropLocation = visionManager.getDetectedSpikeMark();
        telemetry.clearAll();
        telemetry.addData("detected prop = %d", this.TeamPropLocation);
        telemetry.update();

        if (STAGE_LOCATION == STAGE.BACK) {
            if (this.TeamPropLocation == 1) {
                trajectoryActionChosen = goToLeftTape;
            } else if (this.TeamPropLocation == 2) {
                trajectoryActionChosen = goToMiddleTape;
            } else if (this.TeamPropLocation == 3) {
                trajectoryActionChosen = goToRightTape;
            } else {
                trajectoryActionChosen = goToLeftTape;
            }
        } else if (STAGE_LOCATION == STAGE.FRONT) {
            if (this.TeamPropLocation == 1) {
                trajectoryActionChosen = goToLeftTape;
            } else if (this.TeamPropLocation == 2) {
                trajectoryActionChosen = goToMiddleTape;
            } else if (this.TeamPropLocation == 3) {
                trajectoryActionChosen = goToRightTape;
            } else {
                trajectoryActionChosen = goToLeftTape;
            }
        } else {
            telemetry.addLine("No Stage Location.. Uh Oh");
            telemetry.update();
        }

        if (trajectoryActionChosen != null)
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        trajectoryActionCloseOut
                )
        );


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


        /*
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

        private void initializeSystem () {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            //visionManager = new visionManager(THIS_ALLIANCE, hardwareMap.get(WebcamName.class, "Webcam 1"));
            robot = new robot(this, hardwareMap);
            telemetry.addData("auto", "initializeSystem() complete");
        }

        // empty because the subclass is supposed to implement it
        public void initializeGameConfig(){}

        private void GoThroughTruss () {
            if (STAGE_LOCATION == STAGE_LOCATION.FRONT) {
                if (this.TeamPropLocation == 1) {

                } else if (this.TeamPropLocation == 2) {

                } else if (this.TeamPropLocation == 3) {

                }
            }
        }

        private void driveToAprilTag ( int DetectedPropZone){
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
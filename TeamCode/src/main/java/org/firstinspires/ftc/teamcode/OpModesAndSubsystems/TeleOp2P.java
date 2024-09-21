package org.firstinspires.ftc.teamcode.OpModesAndSubsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.*;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="TeleOp2P", group="TeleOp")
public class TeleOp2P extends OpMode
{
    private robot robot;
    private MecanumDrive fDrive;
    Motor frontLeftX, frontRightX, backLeftX, backRightX;
    GamepadEx gamePad2, gamePad1;
    ButtonReader aReader, lbReader, rbReader, xReader, yReader, bReader, dpad_downReader, dpadUpReader, dpad_leftReader;
    TriggerReader ltReader, rtReader;
    NormalizedRGBA colors = new NormalizedRGBA();
    private boolean fDriveMode = false;
    @Override
    public void init()
    {
        robot = new robot(null, hardwareMap);

        // experimental fDrive
        gamePad1 = new GamepadEx(gamepad1);
        dpadUpReader = new ButtonReader(gamePad1, GamepadKeys.Button.DPAD_UP);
        dpad_downReader = new ButtonReader(gamePad1, GamepadKeys.Button.DPAD_DOWN);
        frontLeftX = new Motor(hardwareMap, "frontLeft");
        frontRightX = new Motor(hardwareMap, "frontRight");
        backLeftX = new Motor(hardwareMap, "backLeft");
        backRightX = new Motor(hardwareMap, "backRight");
        fDrive = new MecanumDrive(frontLeftX, frontRightX, backLeftX, backRightX);

        // subsystem controls
        gamePad2 = new GamepadEx(gamepad2);
        ltReader = new TriggerReader(gamePad2, GamepadKeys.Trigger.LEFT_TRIGGER);
        rtReader = new TriggerReader(gamePad2, GamepadKeys.Trigger.RIGHT_TRIGGER);
        lbReader = new ButtonReader(gamePad2, GamepadKeys.Button.LEFT_BUMPER);
        rbReader = new ButtonReader(gamePad2, GamepadKeys.Button.RIGHT_BUMPER);
        aReader = new ButtonReader(gamePad2, GamepadKeys.Button.A);
        bReader = new ButtonReader(gamePad2, GamepadKeys.Button.B);
        xReader = new ButtonReader(gamePad2, GamepadKeys.Button.X);
        yReader = new ButtonReader(gamePad2, GamepadKeys.Button.Y);
        dpad_downReader = new ButtonReader(gamePad2, GamepadKeys.Button.DPAD_DOWN);
        dpad_leftReader = new ButtonReader(gamePad2, GamepadKeys.Button.DPAD_LEFT);


        IMU.Parameters aIMUparams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );

        robot.imu.initialize(aIMUparams);
        robot.imu.resetYaw();
        telemetry.clearAll();
        telemetry.addLine("IMU RESET");
        // Create an object to receive the IMU angles
        YawPitchRollAngles robotOrientation;
        robotOrientation = robot.imu.getRobotYawPitchRollAngles();

        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double Poll  = robotOrientation.getRoll(AngleUnit.DEGREES);

        if (robot.colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)robot.colorSensor).enableLight(true);
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() { }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() { }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {

        // color sensor telemetry

        telemetry.addData("Gain", robot.gain);
        if (robot.colorSensor instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) robot.colorSensor).getDistance(DistanceUnit.CM));
        }
        telemetry.addLine()
                .addData("Red", "%.3f", robot.colors.red)
                .addData("Green", "%.3f", robot.colors.green)
                .addData("Blue", "%.3f", robot.colors.blue);
        telemetry.addData("color of floor", robot.CURRENT_COLOR);
        telemetry.addData("Degrees Rotation:", robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.update();
        // receive gamepad input
        aReader.readValue();
        ltReader.readValue();
        rtReader.readValue();
        lbReader.readValue();
        rbReader.readValue();
        bReader.readValue();
        xReader.readValue();
        yReader.readValue();
        dpad_downReader.readValue();
        dpadUpReader.readValue();
        dpad_leftReader.readValue();

        // TODO: Lots of empty/unused code here, consider cleaning up
        if (aReader.isDown()) {

        } else if (aReader.wasJustReleased()) {

        }

        if (xReader.wasJustReleased()) {

        } else if (bReader.wasJustReleased()) {

        } else if (yReader.wasJustReleased()) {

        }

        // drive screw out
        if (ltReader.isDown()) {
            //robot.hookArm.setPower(gamePad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        } else if (ltReader.wasJustReleased()) {
            //robot.hookArm.setPower(0);
        }

        if(dpad_leftReader.wasJustReleased()) {}

        // drive screw in
        if (rtReader.isDown()) {
            //robot.hookArm.setPower(- gamePad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        } else if (rtReader.wasJustReleased()) {
            //robot.hookArm.setPower(0);
        }

        if (lbReader.wasJustReleased()) {} // hanger up
        if (rbReader.wasJustReleased()) {} // hanger down
        if (dpad_downReader.wasJustReleased()) {} // reset pixel picker-uppers

        double viperPower = gamepad2.left_stick_y;
        robot.safeViperSlide(viperPower);
        /*
       if(theRobot.bucketStop.getState() == false){
           theRobot.viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       }
        if(theRobot.viperSlide.getCurrentPosition() >= -2000) {
            theRobot.safeViperSlide(viperPower);
        }
        else if(theRobot.viperSlide.getCurrentPosition() <= -1800){
            theRobot.viperSlide.setPower(0);
        }
        */
        //telemetry.addData("Encoder pos: ",robot.viperSlide.getCurrentPosition());
        //telemetry.addData("bucket stop state: ", robot.bucketStop.getState());
        telemetry.update();

        // enable/disable experimental fDrive
        if (dpadUpReader.wasJustReleased()) {
            if (fDriveMode) {
                // return to normal mode
                robot.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                fDriveMode = false;
                gamepad1.rumbleBlips(1);
                telemetry.addLine("drive mode: normal");
            } else {
                // enable fDrive
                robot.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                fDriveMode = true;
                gamepad1.rumbleBlips(5);
                telemetry.addLine("drive mode: fDrive");
            }
        }

        if (fDriveMode) {
            fDrive.driveFieldCentric(
                    -gamePad1.getLeftX(), gamePad1.getLeftY(), -gamePad1.getRightX(),
                    robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle,
                    false);
        } else {
            robot.mecanumX(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        if (dpad_downReader.wasJustReleased()) {
            if(fDriveMode){
                robot.imu.resetYaw();
            }
        }

    }
    @Override
    public void stop() { }

}

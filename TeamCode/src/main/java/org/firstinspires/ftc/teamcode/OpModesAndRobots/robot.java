package org.firstinspires.ftc.teamcode.OpModesAndRobots;

import android.view.View;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ThreadPool;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import java.util.concurrent.ExecutorService;

public class robot {
    DigitalChannel limitSwitch1;
    Servo testServo1;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    private ExecutorService threadExecuter;
    private LinearOpMode linearOpMode;

    private Runnable backgroundThread;

    public NormalizedColorSensor colorSensor;
    NormalizedRGBA colors = new NormalizedRGBA();
    IMU imu;
    View relativeLayout;
    float gain = 10;

    public enum COLOR {
        RED,
        BLUE,
        WHITE,
        TILE
    }
    COLOR CURRENT_COLOR;

    double Kp = 1;
    double Ki = 0.1;
    double Kd = 0.01;
    double robotHeading = 0;

    PIDCoefficients coefficients = new PIDCoefficients(Kp, Ki, Kd);
    BasicPID controller = new BasicPID(coefficients);

    private boolean viperThreadLock = false;
    public robot(LinearOpMode linearOpMode, HardwareMap hardwareMap)
    {
        this.linearOpMode = linearOpMode;
/*
        frontLeft = hardwareMap.get(DcMotor.class, "perp");
        frontRight = hardwareMap.get(DcMotor.class, "par0");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "par1");
*/
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        imu = hardwareMap.get(IMU.class, "imu");
        //testServo1 = hardwareMap.get(Servo.class, "testServo1");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");


        colorSensor.setGain(gain);
        colors = colorSensor.getNormalizedColors();


        // odometry wheels



        //reverses direction of wheels (left)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        threadExecuter = ThreadPool.newSingleThreadExecutor("loadPixel");
        backgroundThread = () -> {
            try {
                viperThreadLock = true;
                this.depositPixelsInBucketDragOff();
                viperThreadLock = false;
            } catch (Throwable t) {
                // hm, should probably throw an error
            }
        };
    }

    public void colorChanger()
    {
        if(colors.blue <= 0.1)
        {
            CURRENT_COLOR = COLOR.RED;
        }
        if(colors.blue >= 0.55)
        {
            CURRENT_COLOR = COLOR.BLUE;
        }
        if(colors.blue >= 0.9 & colors.green >= 0.9)
        {
            CURRENT_COLOR = COLOR.WHITE;
        }
        if(CURRENT_COLOR != COLOR.RED & CURRENT_COLOR != COLOR.BLUE & CURRENT_COLOR != COLOR.WHITE)
        {
            CURRENT_COLOR = COLOR.TILE;
        }

    }






    // We transfer from intake to deposit in a background thread because it takes a while.
    // It needs to be accurate, but we should be able to move the drivetrain while its happening.
    public void loadPixelsInBucket()
    {
        threadExecuter.submit(backgroundThread);
    }

    public void safeViperSlide(double power)
    {
        if (! viperThreadLock) {
            //viperSlide.setPower(power);
        }
    }
    public void depositPixelsInBucketDragOff() // this is the code that actually loads the bucket
    {

    }

    public void mecanumX(double forwards,double sideways, double rotate) {
        double denominator = Math.max(Math.abs(forwards) + Math.abs(sideways) + Math.abs(rotate), 1);

        // does math for mecanum chassis
        frontLeft.setPower((forwards + sideways + rotate) / denominator);
        backLeft.setPower((forwards - sideways - rotate) / denominator);
        frontRight.setPower((forwards - sideways + rotate) / denominator);
        backRight.setPower((forwards + sideways - rotate) / denominator);
    }



    // TODO: consolidate this with mecanumX
    // TODO: This is only used by the AprilTag drive-up code (came with the sample code).
    // TODO: We should only have one way to drive the robot.
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }



    public void roboNap(int millis)
    {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

}

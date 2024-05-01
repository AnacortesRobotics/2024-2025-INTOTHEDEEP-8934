package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class propProcessor implements VisionProcessor, CameraStreamSource
{
    private CameraCalibration cCal;
    private Rect[] boundRect;
    private int detectedPropIndex;
    private Paint rectPaint;

    private int data = 0;
    private boolean isRed;
    private Scalar colorLowerBound;
    private Scalar colorUpperBound;
    private Object origin;

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public propProcessor(boolean isRed, Object origin)
    {
        this.isRed = isRed;
        this.origin = origin;
    }

    public propProcessor(Scalar colorLowerBound, Scalar colorUpperBound)
    {
        this.colorLowerBound = colorLowerBound;
        this.colorUpperBound = colorUpperBound;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        cCal = calibration;

        rectPaint = new Paint();
        rectPaint.setAntiAlias(true);
        rectPaint.setColor(Color.rgb(12, 255, 12)); // AE: this should be config for competition tuning
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(5);

        detectedPropIndex = -1;
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    public int getDetectedSpikeMark()
    {
        return data;
    }

    public boolean setDetectedSpikeMark(int newSpikeMarkZone)
    {
        if (newSpikeMarkZone > 0 && newSpikeMarkZone < 4) {
            data = newSpikeMarkZone;
            return true;
        }
        else
            return false;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Mat ourFrame = new Mat(frame.rows(),frame.cols(), frame.type());
        Mat mask = new Mat(frame.rows(),frame.cols(), CvType.CV_8U);
        Mat maskedFrame = new Mat(frame.rows(),frame.cols(),frame.type());
        Mat bgrMask = new Mat(frame.rows(),frame.cols(),frame.type());
        Imgproc.cvtColor(frame, ourFrame, Imgproc.COLOR_RGB2HSV);

        // apply a color mask thingie to the color range we want (red or blue)
        Core.inRange(ourFrame, colorLowerBound, colorUpperBound, mask);

        Imgproc.cvtColor(mask, bgrMask, Imgproc.COLOR_GRAY2RGBA);

        Core.addWeighted(frame, 0.7, bgrMask, 0.3, 0, maskedFrame);

        Bitmap b = Bitmap.createBitmap(maskedFrame.width(), maskedFrame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(maskedFrame, b);
        lastFrame.set(b);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        boundRect = new Rect[contours.size()];
        Point[] centers = new Point[contours.size()];
        float[][] radius = new float[contours.size()][1];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            centers[i] = new Point();
            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
        }

        int largestSize = 800; //Minimum Threshold
        int thisDetectedPropIndex = -1;
        for(int i = 0; i < boundRect.length; i++)
        {
            int currentSize = boundRect[i].width * boundRect[i].height;
            if(currentSize > largestSize)
            {
                largestSize = currentSize;
                thisDetectedPropIndex = i;
            }
        }

        if (boundRect.length > 0 && thisDetectedPropIndex != -1) {

            int x = (boundRect[thisDetectedPropIndex].x + boundRect[thisDetectedPropIndex].width / 3)
                    / (frame.width() / 3);
            x = Math.min(x, 2) + 1;
            setDetectedSpikeMark(x);
        }

        return null;
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        // AE seeing if the crashing occurs when painting the fancy rectangles
        if(boundRect.length == 0)
            return;

        int largestSize = 0;
        int index = 0;
        for(int i = 0; i < boundRect.length; i++)
        {
            int currentSize = boundRect[i].width * boundRect[i].height;
            if(currentSize > largestSize)
            {
                largestSize = currentSize;
                index = i;
            }
        }

        canvas.drawRect(
                (float) boundRect[index].tl().x * scaleBmpPxToCanvasPx,
                (float) boundRect[index].tl().y * scaleBmpPxToCanvasPx,
                (float) boundRect[index].br().x * scaleBmpPxToCanvasPx,
                (float) boundRect[index].br().y * scaleBmpPxToCanvasPx,
                rectPaint);
    }


    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}

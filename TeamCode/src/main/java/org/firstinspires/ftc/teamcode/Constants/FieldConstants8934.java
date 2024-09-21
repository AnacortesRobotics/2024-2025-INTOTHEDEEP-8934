package org.firstinspires.ftc.teamcode.Constants;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;


/*
    Constants that describe the robot's environment and the current field

 Blue Side         +Y
 *Obs Blue          |                *Blue Baskets
                    |
  *   *   *  _______|______  *   *   *Preplaced Yellow Samples
             |      |     |
             |      |     |
             |      |     |
-X -------------- (0,0) -------------- +X
             |      |     |
             |      |     |
             |      |     |
  *   *   *  _______|______  *   *   *Preplaced Red Samples
                    |
 *Red Baskets       |                *Obs Red
 Red Side          -Y



 */

public class FieldConstants8934 {
    //units of the field
    private static final double field = 141.0; // = 6*tileTabless + 5*tab = 6*22.875 + 5*0.750 = 141.000
    private static final double wall = field/2;
    private static final double tileTabbed = 24.375; // See [am-2499 Soft Tile.PDF] (https://www.andymark.com/pages/resources-files?prefix=PDF%20Drawings/)
    private static final double tileTabless = 22.875;
    private static final double tab = 0.750;
    private static final double tabHalf = tab/2;
    private static final double tapeThickness = 2.0;
    private static final double hubRadius = 18.0/2;

    //TODO: VALDIFY CONSTANTS FOR THIS SEASON (TENTATIVE CURRENTLY)

    //Basket Locations
    private static final Pose2d blueBasket = new Pose2d(57.5,57.5,Math.toRadians(48));
    private static final Pose2d redBasket = new Pose2d(-57.5,-57.5,Math.toRadians(228));
    //Center of obs zone ish
    public static final Pose2d blueObsZone = new Pose2d(-57.5,57.5, Math.toRadians(90));
    public static final Pose2d redObsZone = new Pose2d(57.5,-57.5, Math.toRadians(270));

    //Robot starting locations
    public static final Pose2d basketBlueStarting = new Pose2d(6,60.5,Math.toRadians(270));
    public static final Pose2d basketRedStarting = new Pose2d(-6,-60.5,Math.toRadians(270));
    public static final Pose2d autoClipBlue = new Pose2d(6,32,Math.toRadians(270));
    public static final Pose2d autoClipRed = new Pose2d(-6,-32,Math.toRadians(90));

    public static final Pose2d backstageBlueStarting = new Pose2d(12,60.5,Math.toRadians(270));
    public static final Pose2d frontstageBlueStarting = new Pose2d(-36,60.5,Math.toRadians(270));
    public static final Pose2d backstageRedStarting = new Pose2d(12,-60.5,Math.toRadians(90));
    public static final Pose2d frontstageRedStarting = new Pose2d(-36,-60.5,Math.toRadians(90));







}

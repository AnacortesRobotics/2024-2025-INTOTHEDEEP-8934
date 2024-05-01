package org.firstinspires.ftc.teamcode.Constants;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

//TODO: REDO THIS FOR NEXT SEASON'S FIELD

/*
    Constants that describe the robot's environment and the current field

 Substation Red    +Y
 *    Front Blue *  |  * Backstage Blue
 * Stack            |
                    |                * Blue
 * Stack            |                * Board
                    |
 * Stack            |
-X -------------- (0,0) -------------- +X
 * Stack            |
                    |
 * Stack            |                * Red
                    |                * Board
 * Stack            |
 *     Front Red *  |  * Backstage Red
 Substation Blue   -Y



 */

//TODO: SETUP CONSTANTS FOR NEXT SEASON'S FIELD

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


    //Pixel stacks (numbered from +Y to -Y in ascending order)
    public static final Vector2d pixelStack1 = new Vector2d(-70.5,35.75);
    public static final Vector2d pixelStack2 = new Vector2d(-70.5,23.75);
    public static final Vector2d pixelStack3 = new Vector2d(-70.5,11.75);
    public static final Vector2d pixelStack4 = new Vector2d(-70.5,-11.75);
    public static final Vector2d pixelStack5 = new Vector2d(-70.5,-23.75);
    public static final Vector2d pixelStack6 = new Vector2d(-70.5,-35.75);

    //Boards center of front of lip touching ground
    public static final Vector2d blueBoard = new Vector2d(59.75,35.5);
    public static final Vector2d redBoard = new Vector2d(59.75,-35.5);

    //Center of tape line in substation
    public static final Vector2d redSubstation = new Vector2d(-59,58);
    public static final Vector2d blueSubstation = new Vector2d(-59,-58);

    //Robot starting locations
    public static final Pose2d backstageBlueStarting = new Pose2d(12,60.5,Math.toRadians(270));
    public static final Pose2d frontstageBlueStarting = new Pose2d(-36,60.5,Math.toRadians(270));
    public static final Pose2d backstageRedStarting = new Pose2d(12,-60.5,Math.toRadians(90));
    public static final Pose2d frontstageRedStarting = new Pose2d(-36,-60.5,Math.toRadians(90));







}

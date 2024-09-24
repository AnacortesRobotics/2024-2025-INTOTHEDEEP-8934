package org.firstinspires.ftc.teamcode.OpModesAndSubsystems;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BLUEBasketCycle", group="ITD")
public class BLUEBasketCycle extends autoBase {
    @Override
    public void initializeGameConfig() {
        CURRENT_SIDE = SIDE.BASKET;
        THIS_ALLIANCE = autoBase.ALLIANCE.BLUE;
    }
}
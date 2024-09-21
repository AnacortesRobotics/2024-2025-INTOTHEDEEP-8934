package org.firstinspires.ftc.teamcode.OpModesAndSubsystems;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto1Blue", group="CenterStage")
public class Auto1Blue extends autoBase {
    @Override
    public void initializeGameConfig() {
        STAGE_LOCATION = STAGE_LOCATION.BACK;
        THIS_ALLIANCE = autoBase.ALLIANCE.BLUE;
    }
}
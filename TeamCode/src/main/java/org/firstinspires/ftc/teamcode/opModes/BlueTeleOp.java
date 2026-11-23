package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Alliance;

@TeleOp(name = "Blue TeleOp", group = "Match")
public class BlueTeleOp extends BaseTeleOp {
    public BlueTeleOp() {
        super(Alliance.BLUE);
    }
}

package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Alliance;

@TeleOp(name = "Red TeleOp", group = "Match")
public class RedTeleOp extends BaseTeleOp {
    public RedTeleOp() {
        super(Alliance.RED);
    }
}

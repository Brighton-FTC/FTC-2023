package org.firstinspires.ftc.teamcode.opModes.team1.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TeamColour;

@TeleOp(name="Team 1 Red", group="Team 1")
public class Team1RedTeleOp extends TeleOpModeBase {
    Team1GenericTeleOp teleop;

    @Override
    public void setup() {
        // Runs once at INIT
        teleop = new Team1GenericTeleOp();
        teleop.setup(TeamColour.RED);
    }

    @Override
    public void every_tick() {
        teleop.every_tick();
    }
}

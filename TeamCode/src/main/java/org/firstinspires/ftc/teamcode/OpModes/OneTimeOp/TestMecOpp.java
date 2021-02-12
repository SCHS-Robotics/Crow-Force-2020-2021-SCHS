package org.firstinspires.ftc.teamcode.OpModes.OneTimeOp;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.annotations.StandAlone;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.TestMec;


@StandAlone
@TeleOp(name = "TestMecOpp")
public class TestMecOpp extends BaseTeleop {
    TestMec robot;
    @Override
    protected Robot buildRobot() {
        robot = new TestMec(this);
        return robot;
    }

}

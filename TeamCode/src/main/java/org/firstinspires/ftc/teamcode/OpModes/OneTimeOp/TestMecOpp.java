package org.firstinspires.ftc.teamcode.OpModes.OneTimeOp;


import com.SCHSRobotics.HAL9001.system.config.StandAlone;
import com.SCHSRobotics.HAL9001.system.robot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
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

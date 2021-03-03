package org.firstinspires.ftc.teamcode.OpModes.OneTimeOp;


import com.SCHSRobotics.HAL9001.system.config.StandAlone;
import com.SCHSRobotics.HAL9001.system.robot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.EncoderTestRobot;

@StandAlone
@TeleOp(name="EncoderTest")
public class EncoderTest extends BaseTeleop {
    private EncoderTestRobot robot;
    @Override
    protected Robot buildRobot() {
        robot = new EncoderTestRobot(this);
        return robot;
    }

}
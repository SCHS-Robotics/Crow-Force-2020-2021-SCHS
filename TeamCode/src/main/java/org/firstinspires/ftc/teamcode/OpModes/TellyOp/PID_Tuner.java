package org.firstinspires.ftc.teamcode.OpModes.TellyOp;


import com.SCHSRobotics.HAL9001.system.config.StandAlone;
import com.SCHSRobotics.HAL9001.system.robot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.PID_TunerRobot;

@StandAlone
@TeleOp(name = "PID Tuner")
public class PID_Tuner extends BaseTeleop {

    PID_TunerRobot asd;
    @Override
    protected Robot buildRobot() {
        asd = new PID_TunerRobot(this);
        return asd;
    }
}

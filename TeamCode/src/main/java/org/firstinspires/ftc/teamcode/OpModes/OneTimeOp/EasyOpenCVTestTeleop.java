package org.firstinspires.ftc.teamcode.OpModes.OneTimeOp;

import com.SCHSRobotics.HAL9001.system.robot.HALPipeline;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.VisionSubSystem;

import org.opencv.core.Mat;

class EasyOpenCVTestTeleop extends VisionSubSystem {

    public EasyOpenCVTestTeleop(Robot robot) {
        super(robot);
    }

    @Override
    protected HALPipeline[] getPipelines() {
        return new HALPipeline[0];
    }


    public Mat onCameraFrame(Mat input) {
        return null;
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {

    }
}

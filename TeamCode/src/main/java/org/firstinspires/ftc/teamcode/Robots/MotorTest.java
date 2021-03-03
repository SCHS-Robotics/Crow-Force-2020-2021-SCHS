package org.firstinspires.ftc.teamcode.Robots;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.localizer.HolonomicDriveEncoderIMULocalizer;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.RoadrunnerConfig;
import com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain.MecanumDrive;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class MotorTest extends Robot {
    public MecanumDrive mDrive;
    /**
     * Constructor for robot.
     *
     * @pacom.SCHSRobotics.HAL9001.system.robot.RobotpMode - The opmode the robot is currently running.
     */
    public MotorTest(OpMode opMode) {
        super(opMode);
        mDrive = new MecanumDrive(
                this,
                new RoadrunnerConfig(1.88976, 1, 13.359, 383.6, 435),
                "topLeft",
                "topRight",
                "bottomLeft",
                "bottomRight", false);
        //We also need to set a localizer for mDrive
        // Idk which side to reverse
        mDrive.setDriveStick(new Button(1, Button.VectorInputs.left_stick));
        mDrive.setTurnStick(new Button(1, Button.DoubleInputs.right_stick_x));
        mDrive.setAllMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mDrive.setReverseType(MecanumDrive.ReverseType.LEFT);

        //Localizer (need imu config)
        mDrive.setLocalizer(new HolonomicDriveEncoderIMULocalizer(
                this,
                mDrive,
                "imu",
                "topLeft",
                "topRight",
                "bottomLeft",
                "bottomRight"
        ));
    }
}

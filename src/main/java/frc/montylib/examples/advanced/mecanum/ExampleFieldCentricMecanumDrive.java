package frc.montylib.examples.advanced.mecanum;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.montylib.MontyMath;
import frc.montylib.hardware.Falcon500;
import frc.montylib.hardware.NavX2;

public class ExampleFieldCentricMecanumDrive extends SubsystemBase{

    private final double maxMetersPerSecond = 8;
    private final double maxRadiansPerSecond = 2 * Math.PI;

    private Falcon500 leftFrontDrive = null;
    private Falcon500 rightFrontDrive = null;
    private Falcon500 leftBackDrive = null;
    private Falcon500 rightBackDrive = null;

    private NavX2 gyroscope = null;
    
    public ExampleFieldCentricMecanumDrive() {}

    public ExampleFieldCentricMecanumDrive(int[] can_id_array, boolean left_side_reversed, boolean right_side_reversed) {
        leftFrontDrive = new Falcon500(can_id_array[0]);
        rightFrontDrive = new Falcon500(can_id_array[1]);
        leftBackDrive = new Falcon500(can_id_array[2]);
        rightBackDrive = new Falcon500(can_id_array[3]);

        leftFrontDrive.setInverted(left_side_reversed);
        leftBackDrive.setInverted(left_side_reversed);

        rightFrontDrive.setInverted(right_side_reversed);
        rightBackDrive.setInverted(right_side_reversed);

        gyroscope = new NavX2(Port.kMXP);
    }

    public double getHeading() {
        return Units.degreesToRadians(gyroscope.getYaw());
    }

    public void setDesiredSpeeds(ChassisSpeeds speeds) {
        double y = speeds.vxMetersPerSecond / maxMetersPerSecond; //ChassisSpeeds x is the chassis axial movement (forward | backward)
        double x = speeds.vyMetersPerSecond / maxMetersPerSecond; //ChassisSpeeds y it the chassis lateral movement (left | right)
        double r = speeds.omegaRadiansPerSecond / maxRadiansPerSecond; //ChassisSpeeds r is the chassis rotation (clockwise | counterclockwise)

        double fcY = x * Math.sin(getHeading()) + y * Math.cos(getHeading());
        double fcX = x * Math.cos(getHeading()) - y * Math.sin(getHeading());

        leftFrontDrive.set(
            MontyMath.clip(fcY + fcX + r, -1, 1));
        rightFrontDrive.set(
            MontyMath.clip(fcY - fcX - r, -1, 1));
        leftBackDrive.set(
            MontyMath.clip(fcY - fcX + r, -1, 1));
        rightBackDrive.set(
            MontyMath.clip(fcY + fcX - r, -1, 1));
    }

}
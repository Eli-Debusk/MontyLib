package frc.montylib.examples;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.montylib.hardware.Falcon500;

public class ExampleMecanumDrive extends SubsystemBase{
    
    private final double maxMetersPerSecond = 8;
    private final double maxRadiansPerSecond = 2 * Math.PI;

    private Falcon500 leftFrontDrive = null;
    private Falcon500 rightFrontDrive = null;
    private Falcon500 leftBackDrive = null;
    private Falcon500 rightBackDrive = null;
    
    public ExampleMecanumDrive() {}

    public ExampleMecanumDrive(int[] can_id_array, boolean left_side_reversed, boolean right_side_reversed) {
        leftFrontDrive = new Falcon500(can_id_array[0]);
        rightFrontDrive = new Falcon500(can_id_array[1]);
        leftBackDrive = new Falcon500(can_id_array[2]);
        rightBackDrive = new Falcon500(can_id_array[3]);

        leftFrontDrive.setInverted(left_side_reversed);
        leftBackDrive.setInverted(left_side_reversed);

        rightFrontDrive.setInverted(right_side_reversed);
        rightBackDrive.setInverted(right_side_reversed);
    }

    public void setDesiredSpeeds(ChassisSpeeds speeds) {
        double y = speeds.vxMetersPerSecond / maxMetersPerSecond; //ChassisSpeeds x is the chassis axial movement (forward | backward)
        double x = speeds.vyMetersPerSecond / maxMetersPerSecond; //ChassisSpeeds y it the chassis lateral movement (left | right)
        double r = speeds.omegaRadiansPerSecond / maxRadiansPerSecond; //ChassisSpeeds r is the chassis rotation (clockwise | counterclockwise)

        leftFrontDrive.set(y + x + r);
        rightFrontDrive.set(y - x - r);
        leftBackDrive.set(y - x + r);
        rightBackDrive.set(y + x - r);
    }
}   

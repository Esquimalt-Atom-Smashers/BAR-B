package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vision.LimelightEntry;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable table;

    private boolean validTarget;
    private double aprilTagXOffset;
    private double aprilTagYOffset;
    private double[] aprilTagId;

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Command for updating a specifc limelight entry. Note, implicitly assumes the
     * number is an integer.
     */
    public CommandBase updateConfig(Number value, LimelightEntry... entries) {
        return runOnce(() -> {
            for (LimelightEntry entry : entries) {
                table.getEntry(entry.attrName).setNumber(MathUtil.clamp((int) value, entry.min, entry.max));
            }
        });
    }

    public double getAprilTagXOffset() {
        return aprilTagXOffset;
    }

    public double getAprilTagYOffset() {
        return aprilTagYOffset;
    }

    public double getPrimaryAprilTagId() {
        return aprilTagId[0];
    }

    public boolean hasTarget() {
        return validTarget;
    }

    @Override
    public void periodic() {
        validTarget = (table.getEntry("tv").getDouble(0) == 1) ? true : false;
        aprilTagXOffset = table.getEntry("tx").getDouble(0);
        aprilTagYOffset = table.getEntry("ty").getDouble(0);
        aprilTagId = table.getEntry("tid").getDoubleArray(new double[6]);
    }
}

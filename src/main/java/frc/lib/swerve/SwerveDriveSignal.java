package frc.lib.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** 
 * Class which represents the signal that will be applied to the robot.
 * We use a class like this to hold all the important features when the robot is driving,
 * these values include the driving orientation of the robot, whether it is open loop, and if the wheels need to be locked.
 * There should only be one instance of this class 'attatched' to the swerve subsystem and it should 
 * be changed if the conditions of the signal are different.
 */
public class SwerveDriveSignal extends ChassisSpeeds {
    private boolean isFieldOriented;
    private boolean isOpenLoop;

    private boolean isLocked = false;

    /**
     * Constructs a SwerveDriveSignal. 
     * 
     * @param velocity The velocity in ChassisSpeeds
     * @param isFieldOriented true if field oriented, false otherwise.
     * @param isOpenLoop true if open loop, false otherwise.
     */
    public SwerveDriveSignal(ChassisSpeeds velocity, boolean isFieldOriented, boolean isOpenLoop) {
        super(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond);

        this.isFieldOriented = isFieldOriented;
        this.isOpenLoop = isOpenLoop;
    }

    /**
     * Constructs a SwerveDriveSignal object. Assumes isOpenLoop is true;
     * 
     * @param velocity The velocity in ChassisSpeeds
     * @param isFieldOriented true if field orientedm 
     */
    public SwerveDriveSignal(ChassisSpeeds velocity, boolean isFieldOriented) {
        this(velocity, isFieldOriented, true);
    }

    /**
     * Constructs a SwerveDriveSignal object that is locked. 
     * Meaning the wheels are in a X formation to prevent moving.
     * 
     * @param locked used for constructor overloading.
     */
    public SwerveDriveSignal(boolean locked) {
        this(new ChassisSpeeds(), false, false);

        this.isLocked = true;
    }

    /**
     * Constructs a SwerveDriveSignal object. 
     * Assuming field orientation is false and open loop is true;
     */
    public SwerveDriveSignal() {
        super();

        this.isFieldOriented = false;
        this.isOpenLoop = true;
    }

    /** @return true if the signal is field oriented, otherwise false. */
    public boolean isFieldOriented() {
        return this.isFieldOriented;
    }

     /** @return true if the signal is open loop, otherwise false. */
    public boolean isOpenLoop() {
        return this.isOpenLoop;
    }

    /** @return true if the signal is locked, otherwise false. */
    public boolean isLocked() {
        return isLocked;
    }
}

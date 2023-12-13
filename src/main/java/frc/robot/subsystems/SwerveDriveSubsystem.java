package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.gyro.NavXGyro;
import frc.lib.swerve.SwerveDriveSignal;
import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import frc.lib.swerve.SwerveModuleConstants;

public class SwerveDriveSubsystem extends SubsystemBase {
    /** The current pose of the robot. */
    private Pose2d pose = new Pose2d();

    /** The current velocity and previous velocity of the robot. */
    private ChassisSpeeds velocity = new ChassisSpeeds();
    private ChassisSpeeds previousVelocity = new ChassisSpeeds();

    private SwerveDriveSignal driveSignal = new SwerveDriveSignal();

    /** Array of the swerve modules on the robot */
    private SwerveModule[] modules;

    private NavXGyro gyro;

    /** Max speed supplier. */
    private DoubleSupplier maxSpeedSupplier = () -> Constants.SwerveConstants.maxSpeed;

    /**
     * Constructs a SwerveDriveSubsystem object.
     * Initializes the gyro, modules, and any autonmous variables.
     */
    public SwerveDriveSubsystem() {
        gyro = new NavXGyro(); 

        modules = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };
    }

    /** 
     * Command used for driving during tele-operated mode.
     * 
     * @param forward The forwards velocity in meters/second.
     * @param strafe The strafe velocity in meters/second.
     * @param rotation The rotation velocity in radians/second.
     * @param isFieldOriented The driving orientation of the robot.
     * 
     * @return A command object.
     */
    public Command driveCommand(
            DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation, boolean isFieldOriented) {
        return run(() -> {
            setVelocity(
                    new ChassisSpeeds(forward.getAsDouble(), strafe.getAsDouble(), rotation.getAsDouble()),
                    isFieldOriented);
        });
    }

    /**
     * Sets a custom max speed multipler.
     * Very useful if we need to go slower or faster in specific scenarios.
     * 
     * @param maxSpeedSupplier The speed supplier in meters/second. 
     */
    public void setCustomMaxSpeedSupplier(DoubleSupplier maxSpeedSupplier) {
        this.maxSpeedSupplier = maxSpeedSupplier;
    }

    /** @return The pose of the robot. */
    public Pose2d getPose() {
        return pose;
    }

    /** @return The robot relative velocity of the drivetrain. */
    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    /** @return The relative acceleration of the drivetrain. */
    public ChassisSpeeds getAcceleration() {
        return new ChassisSpeeds(
                velocity.vxMetersPerSecond - previousVelocity.vxMetersPerSecond,
                velocity.vyMetersPerSecond - previousVelocity.vyMetersPerSecond,
                velocity.omegaRadiansPerSecond - previousVelocity.omegaRadiansPerSecond);
    }

    /** @return The potentially field relative desired velocity of the drivetrain */
    public ChassisSpeeds getDesiredVelocity() {
        return (ChassisSpeeds) driveSignal;
    }

    /** @return The magnitude of the velocity on the robot. */
    public double getVelocityMagnitude() {
        return Math.sqrt(Math.pow(velocity.vxMetersPerSecond, 2) + Math.pow(velocity.vyMetersPerSecond, 2));
    }

    public Rotation2d getVelocityRotation() {
        return (new Translation2d(velocity.vxMetersPerSecond, velocity.vxMetersPerSecond)).getAngle();
    }

    /** @return The Rotation2d of the gyro. */
    public Rotation2d getGyroRotation() {
        return gyro.getRotation2d();
    }

    /** @return the Rotation2d of the robots pose. */
    public Rotation2d getRotation() {
        return pose.getRotation();
    }

    /** @return The Rotation3d of the gyro. */
    public Rotation3d getGyroRotation3d() {
        return gyro.getRotation3d();
    }

    public Translation3d getNormalVector3d() {
        return new Translation3d(0, 0, 1).rotateBy(getGyroRotation3d());
    }

    /**
     * Instantializes the driveSignal based on the given parameters.
     *
     * @param velocity The velocity of the robot in ChassisSpeeds. 
     * @param isFieldOriented true if it is field oriented, otherwise false.
     * @param isOpenLoop true if it is field oriented, otherwise false. 
    */
    public void setVelocity(ChassisSpeeds velocity, boolean isFieldOriented, boolean isOpenLoop) {
        driveSignal = new SwerveDriveSignal(velocity, isFieldOriented, isOpenLoop);
    }

    /**
     * Instantiatesz the driveSignal based on the given parameters.
     * 
     * @param velocity The velocity of the robot in ChassisSpeeds. 
     * @param isFieldOriented true if it is field oriented, otherwise false.
    */
    public void setVelocity(ChassisSpeeds velocity, boolean isFieldOriented) {
        setVelocity(velocity, isFieldOriented, true);
    }

    /** */
    public void setVelocity(ChassisSpeeds velocity) {
        setVelocity(velocity, false);
    }

    public void stop() {
        driveSignal = new SwerveDriveSignal();
    }

    public void update() {
        updateOdometry();

        updateModules(driveSignal);
    }

    private void updateOdometry() {
        SwerveModuleState[] moduleStates = getModuleStates();

        previousVelocity = velocity;
        velocity = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(moduleStates);
    }

    private void updateModules(SwerveDriveSignal driveSignal) {
        ChassisSpeeds chassisVelocity;

        if (driveSignal.isFieldOriented()) {
            chassisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveSignal.vxMetersPerSecond,
                    driveSignal.vyMetersPerSecond,
                    driveSignal.omegaRadiansPerSecond,
                    getRotation());
        } else {
            chassisVelocity = (ChassisSpeeds) driveSignal;
        }

        SwerveModuleState[] moduleStates =
                Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisVelocity);

        setModuleStates(moduleStates, isDriveSignalStopped(driveSignal) ? true : driveSignal.isOpenLoop());
    }

    private void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeedSupplier.getAsDouble());

        for (SwerveModule module : modules) {
            module.setDesiredState(desiredStates[module.moduleNumber], isOpenLoop, SwerveModuleConstants.isSecondOrder);
        }
    }

    private boolean isDriveSignalStopped(SwerveDriveSignal driveSignal) {
        return driveSignal.vxMetersPerSecond == 0
                && driveSignal.vyMetersPerSecond == 0
                && driveSignal.omegaRadiansPerSecond == 0;
    }

    @Override
    public void periodic() {
        update();
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule module : modules) {
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }
}


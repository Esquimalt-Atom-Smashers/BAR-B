package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.MathUtils;
import frc.robot.Constants.IntakeConstants;

public final class IntakeSubsystem extends SubsystemBase {
    /** Motor for rotating the intake into intake and index position. */
    private final CANSparkMax rotationMotor;
    private final CANSparkMax intakeMotor;

    private final SparkMaxPIDController intakeController;

    private final DigitalInput intakePositionLimit;
    private final DigitalInput indexPositionLimit;

    public IntakeSubsystem() {
        rotationMotor = new CANSparkMax(-1, null);
        configureRotationMotor();

        intakeMotor = new CANSparkMax(-1, null);
        intakeController = intakeMotor.getPIDController();
        configureIntakeMotor();

        intakePositionLimit = new DigitalInput(-1);
        indexPositionLimit = new DigitalInput(-1);
    }

    private void configureIntakeMotor() {
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.enableVoltageCompensation(12.2);
        intakeMotor.setInverted(false);

        intakeController.setP(IntakeConstants.INTAKE_CONTROLLER_KP);
        intakeController.setI(IntakeConstants.INTAKE_CONTROLLER_KI);
        intakeController.setD(IntakeConstants.INTAKE_CONTROLLER_KD);
        intakeController.setIZone(IntakeConstants.INTAKE_CONTROLLER_IZ);
        intakeController.setFF(IntakeConstants.INTAKE_CONTROLLER_FF);
        intakeController.setOutputRange(-1, 1);
    }

    private void configureRotationMotor() {

    }

    public CommandBase intakeNote() {
        return run(() -> intake(IntakeConstants.INTAKE_RPM));
    }

    public CommandBase outtakeNote() {
        return run(() -> outtake(IntakeConstants.OUTTAKE_RPM));
    }


    /** Rotates the intake until it is in index position. */
    public CommandBase intakeToIndexPosition() {
        return run(() -> rotateIntake(-1)).until(this::atIndexPosition).andThen(this::stopRotation);
    }

    /** Rotates the intake until it is in intake position */
    public CommandBase intakeToIntakePosition() {
        return run(() -> rotateIntake(1)).until(this::atIntakePosition).andThen(this::stopRotation);
    }

    private void rotateIntake(double rpm) {

    }

    private void intake(double rpm) {
        intakeController.setReference(MathUtil.clamp(rpm, rpm, IntakeConstants.INTAKE_MAX_RPM)
        , ControlType.kVelocity);
    }

    private void outtake(double rpm) {
        intakeController.setReference(-MathUtil.clamp(rpm, rpm, IntakeConstants.OUTTAKE_MAX_RPM)
        , ControlType.kVelocity);
    }

    private void stopRotation() { rotationMotor.set(0); }
    private void stopIntaking() {
        intakeMotor.set(0);
    }
    

    private boolean atIntakePosition() { return intakePositionLimit.get(); }
    private boolean atIndexPosition() { return indexPositionLimit.get(); }
}

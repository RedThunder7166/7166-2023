package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX motor = new TalonFX(Constants.Intake.MOTOR_ID);

    public IntakeSubsystem () {
        motor.configFactoryDefault();
        motor.setInverted(TalonFXInvertType.CounterClockwise);
    }

    public void setPower (double power) {
        motor.set(TalonFXControlMode.PercentOutput, power);
    }

    public void intake() { setPower(-0.45); }
    public void outtake() { setPower(0.18); }
}

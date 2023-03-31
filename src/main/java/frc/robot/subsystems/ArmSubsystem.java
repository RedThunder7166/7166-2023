package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private TalonFX leftMotor = new TalonFX(Constants.Arm.LEFT_MOTOR_ID);
    private TalonFX rightMotor = new TalonFX(Constants.Arm.RIGHT_MOTOR_ID);
    public enum ArmState { INSIDE, LOW, MEDIUM, HIGH };
    private ArmState state = ArmState.LOW;

    public ArmSubsystem () {
        leftMotor.configFactoryDefault();
        leftMotor.config_kP(0, 0.5);
        leftMotor.config_kI(0, 0);
        leftMotor.config_kD(0, 0);
        leftMotor.configClosedLoopPeakOutput(0, 0.4);
        leftMotor.setInverted(TalonFXInvertType.Clockwise);
        leftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        leftMotor.setNeutralMode(NeutralMode.Brake);
        leftMotor.configMotionCruiseVelocity(7000);
        leftMotor.configMotionAcceleration(14000);
        leftMotor.setSelectedSensorPosition(0);

        rightMotor.configFactoryDefault();
        rightMotor.follow(leftMotor);
        rightMotor.setInverted(TalonFXInvertType.OpposeMaster);
        rightMotor.setNeutralMode(NeutralMode.Brake);

        dashboard();
    }

    public void setAngle (double angle) {
        leftMotor.set(
            TalonFXControlMode.MotionMagic,
            Utils.angleToTicks(angle),
            DemandType.ArbitraryFeedForward,
            Utils.calculateFeedForward(leftMotor.getSelectedSensorPosition())
            // Utils.calculateFeedForward(getAngle() * (Math.PI / 180.0), leftMotor.getSelectedSensorVelocity())
        );
    }

    public double getAngle () {
        return Utils.ticksToAngle(leftMotor.getSelectedSensorPosition());
    }

    public void goToState (ArmState state) {
        switch (state) {
            case INSIDE:
                setAngle(Constants.Arm.INSIDE_SETPOINT);
                break;

            case LOW:
                setAngle(Constants.Arm.LOW_SETPOINT);
                break;

            case MEDIUM:
                setAngle(Constants.Arm.MEDIUM_SETPOINT);
                break;

            case HIGH:
                setAngle(Constants.Arm.HIGH_SETPOINT);
        
            default:
                break;
        }
    }

    public void manualControl (double power) {
        leftMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    public void dashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
        tab.add(this);
        tab.addDouble("Arm Ticks", () -> leftMotor.getSelectedSensorPosition());
        tab.addDouble("Arm Angle", () -> Utils.ticksToAngle( leftMotor.getSelectedSensorPosition() ));
        tab.addString("Arm State", () -> state.name());
    }

    @Override
    public void periodic() {
        // determine arm state
        double angle = Utils.ticksToAngle(leftMotor.getSelectedSensorPosition());
        if (angle >= Constants.Arm.HIGH_SETPOINT) {
            state = ArmState.HIGH;
        } else if (angle < Constants.Arm.HIGH_SETPOINT && angle >= Constants.Arm.MEDIUM_SETPOINT) {
            state = ArmState.MEDIUM;
        } else if (angle < Constants.Arm.MEDIUM_SETPOINT && angle >= Constants.Arm.LOW_SETPOINT) {
            state = ArmState.LOW;
        } else {
            state = ArmState.INSIDE;
        }
    }

    public static class Utils {
        static ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0);//TODO needs to be tuned

        public static double angleToTicks (double angle) { return (angle * ((Constants.Arm.GEAR_RATIO * 2048) / 360)); }

        public static double ticksToAngle (double ticks) { return (ticks * 360) / (Constants.Arm.GEAR_RATIO * 2048); }

        public static double calculateFeedForward (double ticks) {
            return Constants.Arm.FEED_FORWARD * Math.cos(
                Math.toRadians(ticksToAngle(ticks))
            );
            // return armFeedforward.calculate(position, velocity);
        }
    }
    
}

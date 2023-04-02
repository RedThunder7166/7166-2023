package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    private TalonFX motor = new TalonFX(Constants.Wrist.MOTOR_ID);
    public enum WristState { TRANSPORT, LOW, PLACE_MID, PLACE_HIGH, GROUND_CONE };
    private CANCoder wristCanDo = new CANCoder(Constants.Wrist.CANCODER_ID);
    private DigitalInput topSwitch = new DigitalInput(1);
    private DigitalInput bottomSwitch = new DigitalInput(2);
    public WristSubsystem () {
        motor.configFactoryDefault();
        motor.config_kP(0, 0.7);
        motor.config_kI(0, 0);
        motor.config_kD(0, 0);
        motor.configClosedLoopPeakOutput(0, 0.5);
        motor.setInverted(true);
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configMotionCruiseVelocity(7000);
        motor.configMotionAcceleration(10000);
        motor.setSelectedSensorPosition(0);
        wristCanDo.configFactoryDefault();
        wristCanDo.configSensorDirection(false);
        wristCanDo.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        wristCanDo.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        wristCanDo.configMagnetOffset(Constants.Wrist.MAGNET_OFFSET);

        dashboard();
    }


    public void setAngle (double angle) {
        System.out.println(Utils.angleToTicks(angle)
        );
        motor.set(
            TalonFXControlMode.MotionMagic,
            Utils.angleToTicks(angle),
            DemandType.ArbitraryFeedForward,
          0);
    }

    public double getAngle () {
        return Utils.ticksToAngle(motor.getSelectedSensorPosition());
    }

    public void manualControl (double power) {
        if (power > 0 && bottomSwitch.get()) {
            motor.set(TalonFXControlMode.PercentOutput, power);
        } else if (power < 0 && topSwitch.get()) {
            motor.set(TalonFXControlMode.PercentOutput, power);
        } else {
            motor.set(TalonFXControlMode.PercentOutput, 0);
        }
    }

    public void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Wrist");
        tab.add(this);
        tab.addBoolean("Top", () -> !topSwitch.get());
        tab.addBoolean("Bottom", () -> !bottomSwitch.get());
        tab.addDouble("Wrist Pos", () -> motor.getSelectedSensorPosition());
        tab.addDouble("Wrist Angle", () -> Utils.ticksToAngle(motor.getSelectedSensorPosition()));
        tab.addDouble("Wrist Cancoder",() -> wristCanDo.getAbsolutePosition());
    }

    @Override
    public void periodic() {
        motor.setSelectedSensorPosition(Utils.angleToTicks(wristCanDo.getAbsolutePosition()));

    }

    public static class Utils {
        public static double angleToTicks (double angle) { return (angle * ((Constants.Wrist.GEAR_RATIO * 2048) / 360)); }

        public static double ticksToAngle (double ticks) { return (ticks * 360) / (Constants.Wrist.GEAR_RATIO * 2048); }

        public static double calculateFeedForward (double ticks) {
            return Constants.Wrist.FEED_FORWARD * Math.cos(
                Math.toRadians(ticksToAngle(ticks))
            );
        }
    }
}

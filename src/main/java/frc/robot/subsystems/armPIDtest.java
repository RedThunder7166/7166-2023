package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class armPIDtest extends SubsystemBase {

    private TalonFX leftMotor = new TalonFX(Constants.Arm.LEFT_MOTOR_ID);
    private TalonFX rightMotor = new TalonFX(Constants.Arm.RIGHT_MOTOR_ID);
    private DigitalInput ArmZero = new DigitalInput(0);
  
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kF = 0;
    double kRef = 0;


    public armPIDtest () {
        leftMotor.configFactoryDefault();
        leftMotor.config_kP(0, 0);
        leftMotor.config_kI(0, 0);
        leftMotor.config_kD(0, 0);
        leftMotor.config_kF(0, 0);
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

        
    }
    // public void manualControl (double power) {
    //       leftMotor.set(
    //           TalonFXControlMode.PercentOutput, power);
    //   }

      public void editable() {
        
        // get edited PID values from smart dashboard
        double P = SmartDashboard.getNumber("P Input", kP);
        double I = SmartDashboard.getNumber("I Input", kI);
        double D = SmartDashboard.getNumber("D Input", kD);
        double F = SmartDashboard.getNumber("F Input", kF);
        double Ref = SmartDashboard.getNumber("Setpoint Input", kRef);

       

        if(kP != P){

          kP = P;
          leftMotor.config_kP(0, kP);
        }

        if(kI != I){

          kI = I;
          leftMotor.config_kI(0, kI);

      

        }

        if(kD != D){

          kD = D;
          leftMotor.config_kD(0, kD);

        }

        if(kF != F){

          kF = F;
          leftMotor.config_kF(0,kF);

        }

        if(kRef != Ref){

          kRef = Ref; 

        }

        SmartDashboard.putNumber("P Input", kP);
        SmartDashboard.putNumber("I Input", kI);
        SmartDashboard.putNumber("D Input", kD);
        SmartDashboard.putNumber("F Input", kF);
        SmartDashboard.putNumber("Setpoint Input", kRef);
      }
    @Override
    public void periodic() {





  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSubsystem extends SubsystemBase {

private Spark m_leftBlinkIn = new Spark(Constants.LED.LEFT_BLINKIN_ID);
private Spark m_rightBlinkIn = new Spark(Constants.LED.RIGHT_BLINKIN_ID);
  /** Creates a new LEDs. */
  public LedSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLeftBlinkIn(double value){
    m_leftBlinkIn.set(value);
  }
  public void setRightBlinkIn(double value){
    m_rightBlinkIn.set(value);
  }

  public void setBoth(double value){
    setLeftBlinkIn(value);
    setRightBlinkIn(value);
  }
}

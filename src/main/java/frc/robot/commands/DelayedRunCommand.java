// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DelayedRunCommand extends CommandBase {
  private final Timer m_timer = new Timer();
  private final double m_delay_seconds;
  private final Runnable m_execute_func;
  private final boolean m_do_exponential_speed;

  public DelayedRunCommand(double delay, Runnable execute_func) {
    m_delay_seconds = delay;
    m_execute_func = execute_func;
    m_do_exponential_speed = false;
  }
  public DelayedRunCommand(double delay, Runnable execute_func, boolean do_exponential_speed) {
    m_delay_seconds = delay;
    m_execute_func = execute_func;
    m_do_exponential_speed = do_exponential_speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() >= m_delay_seconds) {
      m_execute_func.run();
      if (m_do_exponential_speed) {
        double times = m_timer.get() / 0.7;
        if (times > 1) {
          for (double i = 1; i < times; i++) {
            m_execute_func.run();
          }
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

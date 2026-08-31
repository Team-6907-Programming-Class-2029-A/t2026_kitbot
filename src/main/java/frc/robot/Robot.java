// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Command-based 版本的 Robot 主类。
 * CommandRobot 风格:robotPeriodic 里运行 CommandScheduler,
 * 所有周期逻辑由 subsystem 的默认命令和按键绑定的命令自动执行。
 */
public class Robot extends TimedRobot {
  private final RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // 每个周期运行 command 调度器,这是 command-based 程序的心跳。
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    // 自动阶段开始时取出并调度自动命令。
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void disabledInit() {
    // 禁用阶段停掉所有命令,防止残留输出。
    CommandScheduler.getInstance().cancelAll();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * 使用 AdvantageKit 的 LoggedRobot 替代 TimedRobot,
 * 自动记录所有 subsystem 的 input/output 和 command 状态。
 */
public class Robot extends LoggedRobot {
  private final RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    // 配置 AdvantageKit 日志输出:同时写入 USB 存储(.wpilog)和 NetworkTables(用于 AdvantageScope 实时查看)
    Logger.addDataReceiver(new WPILOGWriter());
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    // 每个周期运行 command 调度器,这是 command-based 程序的心跳。
    CommandScheduler.getInstance().run();
    // 记录手柄输入和顶层 command 状态。
    m_robotContainer.periodic();
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
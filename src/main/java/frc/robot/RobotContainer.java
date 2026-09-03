// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;

/**
 * RobotContainer 负责存放所有 subsystem、手柄绑定和 autonomous 命令。
 * 真正的周期性逻辑都由 command-based 调度器自动运行。
 */
public class RobotContainer {
  // subsystem 对象。
  private final Drive m_drive = new Drive();
  private final Shooter m_shooter = new Shooter();

  // 手柄。CommandXboxController 直接提供按键 Trigger,方便绑定命令。
  private final CommandXboxController m_controller =
      new CommandXboxController(Constants.kControllerPort);

  public RobotContainer() {
    configureButtonBindings();
  }

  /** 配置手柄按键和命令的绑定。 */
  private void configureButtonBindings() {
    // 默认命令:teleop 时一直用摇杆开底盘。
    m_drive.setDefaultCommand(
        m_drive.arcadeDriveCommand(m_controller::getLeftY, m_controller::getRightX));

    // 按住左 bumper 吸球。
    m_controller.leftBumper().whileTrue(m_shooter.intakeCommand());

    // 按住 X 反向排球,把球从吸球口退出去。
    m_controller.x().whileTrue(m_shooter.ejectCommand());

    // 按住右 bumper 高速射球(先升速到目标转速,再 feed 射出)。
    m_controller.rightBumper().whileTrue(m_shooter.launchFastCommand());

    // 按住 Y 低速射球(先升速到目标转速,再 feed 射出)。
    m_controller.rightTrigger().whileTrue(m_shooter.launchSlowCommand());

    // 按住 B 只转 feeder,把球送到发射位置。
    m_controller.b().whileTrue(m_shooter.feedCommand());
  }

  /**
   * 每周期由 Robot.robotPeriodic() 调用,记录手柄输入和 command 调度状态。
   * 这些是 AdvantageKit 日志中的顶层 input 变量。
   */
  public void periodic() {
    // 手柄摇杆输入(底盘命令的 input 来源)。
    Logger.recordOutput("Controller/leftY", m_controller.getLeftY());
    Logger.recordOutput("Controller/rightX", m_controller.getRightX());

    // 手柄按键状态。
    Logger.recordOutput("Controller/leftBumper", m_controller.leftBumper().getAsBoolean());
    Logger.recordOutput("Controller/rightBumper", m_controller.rightBumper().getAsBoolean());
    Logger.recordOutput("Controller/rightTrigger", m_controller.rightTrigger().getAsBoolean());
    Logger.recordOutput("Controller/aButton", m_controller.a().getAsBoolean());
    Logger.recordOutput("Controller/bButton", m_controller.b().getAsBoolean());
    Logger.recordOutput("Controller/xButton", m_controller.x().getAsBoolean());
    Logger.recordOutput("Controller/yButton", m_controller.y().getAsBoolean());

    // 当前正在运行的 command 名称(LoggedRobot 会自动记录 CommandScheduler 状态,这里额外补一个汇总)。
    Command currentCommand = CommandScheduler.getInstance().requiring(m_drive);
    Logger.recordOutput("ActiveCommand/drive",
        currentCommand != null ? currentCommand.getName() : "none");
    currentCommand = CommandScheduler.getInstance().requiring(m_shooter);
    Logger.recordOutput("ActiveCommand/shooter",
        currentCommand != null ? currentCommand.getName() : "none");
  }

  /** 自动阶段执行的命令:直接低速射球(先升速再 feed)。 */
  public Command getAutonomousCommand() {
    return (m_drive.arcadeDriveCommand(()->0.3, ()->0.0)).withTimeout(3).andThen(m_shooter.launchSlowCommand());
    
  }
}

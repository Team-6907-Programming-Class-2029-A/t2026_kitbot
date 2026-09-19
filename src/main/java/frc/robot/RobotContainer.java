// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  // Logitech Extreme 3D Pro 飞行摇杆。
  private final CommandJoystick m_joystick =
      new CommandJoystick(Constants.kJoystickPort);

  // Xbox 手柄保留为射击机构的第二组控制输入。
  private final CommandXboxController m_xboxController =
      new CommandXboxController(Constants.kXboxControllerPort);

  public RobotContainer() {
    configureButtonBindings();
  }

  /** 配置手柄按键和命令的绑定。 */
  private void configureButtonBindings() {
    // 默认命令: Y 轴控制前后,手柄扭转轴控制转向。
    m_drive.setDefaultCommand(
        m_drive.arcadeDriveCommand(m_joystick::getY, m_joystick::getTwist));

    // Extreme 3D Pro 按钮 1 (扳机) 或 Xbox 右 bumper: 高速射球。
    Trigger launchFast = m_joystick.button(1).or(m_xboxController.rightBumper());
    launchFast.whileTrue(m_shooter.launchFastCommand());

    // Extreme 3D Pro 按钮 2 或 Xbox 右 trigger: 低速射球。
    Trigger launchSlow = m_joystick.button(2).or(m_xboxController.rightTrigger());
    launchSlow.whileTrue(m_shooter.launchSlowCommand());

    // Extreme 3D Pro 按钮 3 或 Xbox 左 bumper: 吸球。
    Trigger intake = m_joystick.button(3).or(m_xboxController.leftBumper());
    intake.whileTrue(m_shooter.intakeCommand());

    // Extreme 3D Pro 按钮 4 或 Xbox X: 反向排球。
    Trigger eject = m_joystick.button(4).or(m_xboxController.x());
    eject.whileTrue(m_shooter.ejectCommand());

    // Extreme 3D Pro 按钮 5 或 Xbox B: 只转 feeder。
    Trigger feed = m_joystick.button(5).or(m_xboxController.b());
    feed.whileTrue(m_shooter.feedCommand());
  }

  /**
   * 每周期由 Robot.robotPeriodic() 调用,记录手柄输入和 command 调度状态。
   * 这些是 AdvantageKit 日志中的顶层 input 变量。
   */
  public void periodic() {
    // 手柄摇杆输入(底盘命令的 input 来源)。
    Logger.recordOutput("Controller/y", m_joystick.getY());
    Logger.recordOutput("Controller/twist", m_joystick.getTwist());

    // 手柄按键状态。
    Logger.recordOutput("Controller/button1LaunchFast", m_joystick.getHID().getRawButton(1));
    Logger.recordOutput("Controller/button2LaunchSlow", m_joystick.getHID().getRawButton(2));
    Logger.recordOutput("Controller/button3Intake", m_joystick.getHID().getRawButton(3));
    Logger.recordOutput("Controller/button4Eject", m_joystick.getHID().getRawButton(4));
    Logger.recordOutput("Controller/button5Feed", m_joystick.getHID().getRawButton(5));
    Logger.recordOutput("Xbox/leftBumper", m_xboxController.getHID().getLeftBumperButton());
    Logger.recordOutput("Xbox/rightBumper", m_xboxController.getHID().getRightBumperButton());
    Logger.recordOutput("Xbox/rightTrigger", m_xboxController.getRightTriggerAxis());
    Logger.recordOutput("Xbox/bButton", m_xboxController.getHID().getBButton());
    Logger.recordOutput("Xbox/xButton", m_xboxController.getHID().getXButton());

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

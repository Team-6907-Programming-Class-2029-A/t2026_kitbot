// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** 底盘子系统:负责差速底盘的电机初始化和开/转向控制。 */
public class Drive extends SubsystemBase {
  // master 是每侧底盘的主电机控制器,follower 会跟随同侧 master 输出。
  private final WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(Constants.kLeftMasterCanId);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(Constants.kLeftFollowerCanId);
  private final WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(Constants.kRightMasterCanId);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(Constants.kRightFollowerCanId);

  // DifferentialDrive 负责把 forward/rotation 转换成左右两侧底盘输出。
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  public Drive() {
    // 恢复默认配置,减少旧配置影响当前代码。
    m_leftMaster.configFactoryDefault();
    m_leftFollower.configFactoryDefault();
    m_rightMaster.configFactoryDefault();
    m_rightFollower.configFactoryDefault();

    // 设置 follower,让每侧副电机自动跟随同侧主电机。
    m_leftFollower.follow(m_leftMaster);
    m_rightFollower.follow(m_rightMaster);

    // 反转右侧底盘,因为差速底盘左右电机通常镜像安装。
    m_rightMaster.setInverted(true);
    m_rightFollower.setInverted(true);

    // 底盘设置为 Brake,松开摇杆时更快停下,也更不容易被推动。
    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setNeutralMode(NeutralMode.Brake);
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * 用一个前后量和一个旋转量控制差速底盘。
   *
   * @param forward 前后速度,正方向为前进
   * @param rotation 旋转速度,正方向为顺时针(向右)
   */
  public void arcadeDrive(double forward, double rotation) {
    m_drive.arcadeDrive(forward, rotation);
  }

  /** 持续用摇杆值驱动底盘的命令,一般作为 teleop 默认命令。 */
  public Command arcadeDriveCommand(XboxController controller) {
    return run(
        () ->
            arcadeDrive(
                // 加负号是为了让“摇杆向上”对应机器人前进。
                -controller.getLeftY(), -controller.getRightX()));
  }
}

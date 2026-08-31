// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * 射击子系统:包含 feeder(供球)和 shooter(发射)两组电机。
 * feeder 和 shooter 总是配合动作,所以放在同一个 subsystem 里管理。
 */
public class Shooter extends SubsystemBase {
  private final TalonFX m_feeder = new TalonFX(Constants.kFeederCanId);
  private final TalonFX m_shooter = new TalonFX(Constants.kShooterCanId);

  // Phoenix 6 控制请求对象:VelocityVoltage 表示速度闭环,NeutralOut 表示停止输出。
  private final VelocityVoltage m_feederVelocityRequest = new VelocityVoltage(0.0);
  private final NeutralOut m_stopRequest = new NeutralOut();

  public Shooter() {
    // 创建 feeder 的配置,包括方向、刹车模式和速度闭环参数。
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.MotorOutput.Inverted = Constants.kFeederInverted;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederConfig.Slot0.kS = Constants.kFeederKs;
    feederConfig.Slot0.kV = Constants.kFeederKv;
    feederConfig.Slot0.kP = Constants.kFeederKp;
    feederConfig.Slot0.kI = Constants.kFeederKi;
    feederConfig.Slot0.kD = Constants.kFeederKd;

    // shooter 使用 Coast,停止输出后可以自然滑行。
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted = Constants.kShooterInverted;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.Slot0.kS = Constants.kShooterKs;
    shooterConfig.Slot0.kV = Constants.kShooterKv;
    shooterConfig.Slot0.kP = Constants.kShooterKp;
    shooterConfig.Slot0.kI = Constants.kShooterKi;
    shooterConfig.Slot0.kD = Constants.kShooterKd;

    // 把配置真正写入 TalonFX 控制器。
    m_feeder.getConfigurator().apply(feederConfig);
    m_shooter.getConfigurator().apply(shooterConfig);

    // 默认命令:没有按键按下时让 feeder 和 shooter 停止输出。
    setDefaultCommand(run(this::stop));
  }

  /** 让 feeder 和 shooter 停止输出。 */
  public void stop() {
    m_shooter.setControl(m_stopRequest);
    m_feeder.setControl(m_stopRequest);
  }

  /** 吸球:shooter 慢速正转,feeder 反转把球吸进来。 */
  public void intake() {
    m_shooter.setControl(
        new VelocityVoltage(Constants.kIntakeShooterVelocityRps));
    m_feeder.setControl(new VelocityVoltage(Constants.kIntakeFeederVelocityRps));
  }

  /** 高速射球:shooter 和 feeder 直接给定电压全速输出。 */
  public void shootFast() {
    m_shooter.setControl(new VoltageOut(Constants.kShootFastShooterVoltage));
    m_feeder.setControl(new VoltageOut(Constants.kShootFastFeederVoltage));
  }

  /** 低速射球:shooter 和 feeder 给定较低电压。 */
  public void shootSlow() {
    m_shooter.setControl(new VoltageOut(Constants.kShootSlowShooterVoltage));
    m_feeder.setControl(new VoltageOut(Constants.kShootSlowFeederVoltage));
  }

  /** 只转 feeder(shooter 停转),用于把球送到发射位置。 */
  public void feed() {
    m_feeder.setControl(m_feederVelocityRequest.withVelocity(Constants.kFeederVelocityRps));
    m_shooter.setControl(m_stopRequest);
  }

  /** 按住左 bumper 时吸球。 */
  public Command intakeCommand() {
    return run(this::intake);
  }

  /** 按住右 bumper 时高速射球。 */
  public Command shootFastCommand() {
    return run(this::shootFast);
  }

  /** 按住 Y 时低速射球。 */
  public Command shootSlowCommand() {
    return run(this::shootSlow);
  }

  /** 按住 B 时只转 feeder。 */
  public Command feedCommand() {
    return run(this::feed);
  }
}

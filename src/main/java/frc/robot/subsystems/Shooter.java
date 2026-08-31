// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * 射击子系统:包含 feeder(供球)和 shooter(发射)两组电机。
 * feeder 和 shooter 总是配合动作,所以放在同一个 subsystem 里管理。
 * 所有动作都用速度闭环(VelocityVoltage)控制。
 */
public class Shooter extends SubsystemBase {
  private final TalonFX m_feeder = new TalonFX(Constants.kFeederCanId);
  private final TalonFX m_shooter = new TalonFX(Constants.kShooterCanId);

  // Phoenix 6 控制请求对象:VelocityVoltage 表示速度闭环,NeutralOut 表示停止输出。
  private final VelocityVoltage m_feederVelocityRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage m_shooterVelocityRequest = new VelocityVoltage(0.0);
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

  /** 判断 shooter 是否已经升到目标转速(允许一定误差)。 */
  private boolean atShooterSpeed(double targetRps) {
    return Math.abs(m_shooter.getVelocity().getValueAsDouble())
        >= Math.abs(targetRps) - Constants.kLaunchShooterVelocityToleranceRps;
  }

  /** 按住左 bumper 时吸球:shooter 慢速正转,feeder 反转把球吸进来。 */
  public Command intakeCommand() {
    return run(
        () -> {
          m_shooter.setControl(
              m_shooterVelocityRequest.withVelocity(Constants.kIntakeShooterVelocityRps));
          m_feeder.setControl(
              m_feederVelocityRequest.withVelocity(Constants.kIntakeFeederVelocityRps));
        });
  }

  /**
   * 反向转动 feeder 和 shooter,把球从吸球口排出(和 AdvantageKit 的 eject 一样,
   * 速度取吸球速度的反向)。
   */
  public Command ejectCommand() {
    return run(
        () -> {
          m_shooter.setControl(
              m_shooterVelocityRequest.withVelocity(-Constants.kIntakeShooterVelocityRps));
          m_feeder.setControl(
              m_feederVelocityRequest.withVelocity(-Constants.kIntakeFeederVelocityRps));
        });
  }

  /**
   * 射球流程(和 AdvantageKit 的 launch 一样先升速再射):
   * 升速阶段 shooter 目标转速闭环加速,feeder 反向慢转把球挡在 shooter 外;
   * shooter 达到目标转速后 feeder 才正向 feed,把球送进已加速的 shooter 射出。
   */
  private Command launchCommand(double shooterVelocityRps, double feederVelocityRps) {
    // 第一阶段:升速,直到 shooter 达到目标转速。
    return run(
            () -> {
              m_shooter.setControl(m_shooterVelocityRequest.withVelocity(shooterVelocityRps));
              m_feeder.setControl(
                  m_feederVelocityRequest.withVelocity(Constants.kSpinUpFeederVelocityRps));
            })
        .until(() -> atShooterSpeed(shooterVelocityRps))
        // 第二阶段:feeder 开始 feed 射球。
        .andThen(
            run(
                () -> {
                  m_shooter.setControl(
                      m_shooterVelocityRequest.withVelocity(shooterVelocityRps));
                  m_feeder.setControl(m_feederVelocityRequest.withVelocity(feederVelocityRps));
                }));
  }

  /** 按住右 bumper 时高速射球(先升速再 feed)。 */
  public Command launchFastCommand() {
    return launchCommand(
        Constants.kLaunchFastShooterVelocityRps, Constants.kLaunchFastFeederVelocityRps);
  }

  /** 按住 Y 时低速射球(先升速再 feed)。 */
  public Command launchSlowCommand() {
    return launchCommand(
        Constants.kLaunchSlowShooterVelocityRps, Constants.kLaunchSlowFeederVelocityRps);
  }

  /** 按住 B 时只转 feeder(shooter 停转),用于把球送到发射位置。 */
  public Command feedCommand() {
    return run(
        () -> {
          m_feeder.setControl(m_feederVelocityRequest.withVelocity(Constants.kFeederVelocityRps));
          m_shooter.setControl(m_stopRequest);
        });
  }
}

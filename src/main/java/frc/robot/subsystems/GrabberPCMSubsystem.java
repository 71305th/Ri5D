// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.OIConstants;;

public class GrabberPCMSubsystem extends SubsystemBase {
  
  private final Compressor comp = new Compressor(GrabberConstants.compressorID ,PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid DoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GrabberConstants.ForwardChannel, GrabberConstants.ReverseChannel);

  private final Joystick joystick = new Joystick(OIConstants.operatorController);

  private boolean state = false;
  private boolean lastButtonState = false;

  /** Creates a new GrabberPCMSubsystem. */
  public GrabberPCMSubsystem() {
    this.state = false;
    this.lastButtonState = false;
    this.comp.disable();
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    boolean buttonInput = this.joystick.getRawButtonPressed(OIConstants.Btn_LB);
    if(this.lastButtonState == true && buttonInput == false){
      this.state = !this.state;
    }
    this.lastButtonState = buttonInput;
    if (this.state){
      this.handOpen();
    } else {
      this.handClose();
    }

  }

  public void enablecompressor(){
    comp.enableDigital();
    DoublePCM.isFwdSolenoidDisabled();
  }

  public void handOpen(){
    DoublePCM.set(DoubleSolenoid.Value.kForward);
  }

  public void handClose(){
    DoublePCM.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void handStop(){
    DoublePCM.set(DoubleSolenoid.Value.kOff);
  }
}
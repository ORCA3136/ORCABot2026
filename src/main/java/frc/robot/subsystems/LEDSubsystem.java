// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Colors;

public class LEDSubsystem extends SubsystemBase {
  //Creates a new subsystem for the LEDs

  Spark blinkin;

  boolean discoOn = false;

  public LEDSubsystem() {

    blinkin = new Spark(0);
  }
  
  public void setLedColor(double color) {
    
    blinkin.set(color);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (discoOn) {

      setRandomColor();
    }

  }

  public void setRandomColor() {
    int min = 1;
    int max = 11;
    int randomNum = (int)(Math.random() * (max - min + 1) + min);
    // randomNum will be between 1 and 11, inclusive

    //This will chose a random color out of the 11 bellow, yes there is more blue than any other color, yes that is intentional
    Double randomColor = null;
    switch (randomNum) {
      case 1: randomColor =  Colors.Hot_Pink;
        break;
      case 2: randomColor =  Colors.Red;
        break;
      case 3: randomColor =  Colors.Orange;
        break;
      case 4: randomColor =  Colors.Yellow;
        break;
      case 5: randomColor =  Colors.Green;
        break;
      case 6: randomColor =  Colors.Aqua;
        break;
      case 7: randomColor =  Colors.Dark_Blue;
        break;
      case 8: randomColor =  Colors.Blue;
        break;
      case 9: randomColor =  Colors.Violet;
        break;
      case 10: randomColor =  Colors.White;
        break;
      case 11: randomColor =  Colors.Black;
        break;
      default:
        break;
    }

    setLedColor(randomColor);
  }

  public void setInitialColor() {
    
    setLedColor(Constants.Colors.White);

    // private String teamColor = input()
    // if (teamColor == Blue) {
    //   setLedColor(Constants.Colors.Blue);
    // }else if (teamColor == Red) {
    //   setLedColor(Constants.Colors.Red);
    // }
  }

  public void ChangeLedColor (int num) {
    discoOn = false;
    switch (num) {
      case 1:  // lined_up
        blinkin.set(Colors.Green);
        break;
      case 2: // LimelightHelpers.getTV("limelight-two");
        blinkin.set(Colors.Red);
        break;
      case 3: // coralSensor (Lidar I think)
        blinkin.set(Colors.Lawn_Green);
        break;
      case 4: // algaeSensor
        blinkin.set(Colors.Blue_Violet);
        break;
      case 5: 
        discoOn = true;
      case 6: //No coral / No coral in safe position
        blinkin.set(Colors.White);
        break;
      default:
        setInitialColor();
        break;
    }

  }

}
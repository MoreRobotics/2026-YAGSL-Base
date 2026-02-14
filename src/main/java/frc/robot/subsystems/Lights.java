// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;



public class Lights extends SubsystemBase {
    
    // Constants
    private static final int candleID = 0;
    private static final int numLEDS = 0;
    private static final double brightness = 0.5;
    private static final int halfpoint = Math.round(numLEDS / 2);
    
    // RGB Colors
    private static final RGBWColor kRed = new RGBWColor(Color.kRed).scaleBrightness(brightness);
    private static final RGBWColor kGreen = new RGBWColor(Color.kGreen).scaleBrightness(brightness);
    private static final RGBWColor kBlue = new RGBWColor(Color.kBlue).scaleBrightness(0.5);
    private static final RGBWColor kYellow = new RGBWColor(Color.kYellow).scaleBrightness(0.5);
    private static final RGBWColor kBlack = new RGBWColor(Color.kBlack).scaleBrightness(0.5);

    // Candle Method
    private final CANdle m_candle = new CANdle(candleID, CANBus.roboRIO());


    private enum AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
    }

    private AnimationType m_anim0State = AnimationType.None;



    public Lights() {

        /* Configure CANdle */
        var cfg = new CANdleConfiguration();

        /* set the LED strip type and brightness */
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = brightness;

        /* disable status LED when being controlled */
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        m_candle.getConfigurator().apply(cfg);

        /* clear all previous animations */
        for (int i = 0; i < 8; ++i) {
            m_candle.setControl(new EmptyAnimation(i));
        }
    };

    public void ClearLights() {
         m_candle.setControl(new SolidColor(0, numLEDS).withColor(kBlack));
    }


    public void Aim() {
        m_candle.setControl(new SolidColor(0, numLEDS).withColor(kGreen));
    };
    
    public void ReadyToFire() {
        m_candle.setControl(new SolidColor(0, numLEDS).withColor(kRed));
    };

    
    public void Defense() {
        m_candle.setControl(new SolidColor(0, halfpoint).withColor(kRed));
        m_candle.setControl(new SolidColor(halfpoint, numLEDS).withColor(kBlue));
    };
    public void DefenseAlternate() {
        m_candle.setControl(new SolidColor(0, halfpoint).withColor(kBlue));
        m_candle.setControl(new SolidColor(halfpoint, numLEDS).withColor(kRed));
    }

}

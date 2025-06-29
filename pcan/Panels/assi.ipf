CLSID = {DFAFFAC0-2F50-428E-824C-B97089CD5F3F}
object _FormData
  FormatVersion = '3.0'
  Locked = True
  AutoFrameRate = False
end
object Scene1: _Scene
  HorzScrollBar.Tracking = True
  VertScrollBar.Tracking = True
  Color = clWhite
  Index = 0
  ExplicitWidth = 559
  ExplicitHeight = 319
  PixelsPerInch = 96
  TextHeight = 13
  object Label1: _Label
    Left = 8
    Top = 8
    Width = 80
    Height = 16
    Caption = 'ASSI'
    Color = clBtnFace
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -16
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    ParentColor = False
    ParentFont = False
    ShowAccelChar = False
    Transparent = True
    WordWrap = True
  end
  object ToggleSwitch1: _ToggleSwitch
    Left = 146
    Top = 59
    Width = 32
    Height = 56
    SignalName = 'eufs_can2.VCU2AI_Status.BrakeLight'
    BevelOuter.BorderSpacing = 1
    BevelOuter.BevelLine = blNone
    BevelOuter.BorderWidth = 1
    BevelOuter.BkColor = clBtnShadow
    BevelOuter.ColorShadowFrom = clBlack
    BevelOuter.ColorShadowTo = clBlack
    BevelOuter.ColorHighLightFrom = clBlack
    BevelOuter.ColorHighLightTo = clBlack
    BevelOuter.BorderStyle = 2
    BevelOuter.SurfaceGrad.Visible = False
    BevelOuter.SurfaceGrad.Style = gsHorizontal1
    BevelOuter.GradientStyle = gsHorizontal1
    BevelOuter.StartColor = 14869218
    BevelOuter.StopColor = 9408399
    BtnColorHighlight = clBtnHighlight
    BtnColorShadow = clBtnShadow
    BtnColorFace = clBtnFace
    BtnBevelWidth = 2
    Orientation = boVertical
    Mode = mSwitch
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    StatusInt = 0
    StatusBit = 0
    GroupIndex = 0
    Checked = False
    BinarySignal = True
    UseSignalCycleTime = False
    CycleTime = 100
    IsPaused = False
    SwapGraphic = False
    TextOff = 'Off'
    TextOn = 'On'
    ColorOn = clLime
    ColorOff = clSilver
  end
  object Label2: _Label
    Left = 114
    Top = 29
    Width = 102
    Height = 19
    Caption = 'Brake Light'
    Color = clDefault
    Enabled = False
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -16
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    ParentColor = False
    ParentFont = False
    ShowAccelChar = False
    Transparent = True
    WordWrap = True
  end
  object Button1: _Button
    Left = 0
    Top = 30
    Width = 75
    Height = 23
    Caption = 'AS Off'
    TabOrder = 0
    SignalName = 'eufs_can2.VCU2AI_Status.AS_STATE'
    PushValue = '1'
    ReleaseValue = '1'
    UseSignalCycleTime = False
    CycleTime = 100
    IsPaused = False
  end
  object Button2: _Button
    Left = 0
    Top = 55
    Width = 75
    Height = 23
    Caption = 'AS Ready'
    TabOrder = 1
    SignalName = 'eufs_can2.VCU2AI_Status.AS_STATE'
    PushValue = '2'
    ReleaseValue = '2'
    UseSignalCycleTime = False
    CycleTime = 100
    IsPaused = False
  end
  object Button3: _Button
    Left = 0
    Top = 80
    Width = 75
    Height = 23
    Caption = 'AS Driving'
    TabOrder = 2
    SignalName = 'eufs_can2.VCU2AI_Status.AS_STATE'
    PushValue = '3'
    ReleaseValue = '3'
    UseSignalCycleTime = False
    CycleTime = 100
    IsPaused = False
  end
  object Button4: _Button
    Left = 0
    Top = 105
    Width = 75
    Height = 23
    Caption = 'AS Emergency'
    TabOrder = 3
    SignalName = 'eufs_can2.VCU2AI_Status.AS_STATE'
    PushValue = '4'
    ReleaseValue = '4'
    UseSignalCycleTime = False
    CycleTime = 100
    IsPaused = False
  end
  object Button5: _Button
    Left = 0
    Top = 130
    Width = 75
    Height = 23
    Caption = 'AS Finished'
    TabOrder = 4
    SignalName = 'eufs_can2.VCU2AI_Status.AS_STATE'
    PushValue = '5'
    ReleaseValue = '5'
    UseSignalCycleTime = False
    CycleTime = 100
    IsPaused = False
  end
  object Button6: _Button
    Left = 0
    Top = 155
    Width = 75
    Height = 23
    Caption = 'Manual Driving'
    TabOrder = 5
    SignalName = 'eufs_can2.VCU2AI_Status.AS_STATE'
    PushValue = '6'
    ReleaseValue = '6'
    UseSignalCycleTime = False
    CycleTime = 100
    IsPaused = False
  end
end

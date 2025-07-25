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
  ExplicitWidth = 1254
  ExplicitHeight = 483
  PixelsPerInch = 96
  TextHeight = 13
  object RockerSwitch1: _RockerSwitch
    Left = 0
    Top = 32
    Width = 48
    Height = 40
    SignalName = 'eufs_can1.NMT_Node_Control.Operational_Mode_Flag'
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
    OnValue = 1.000000000000000000
    BinarySignal = True
    UseSignalCycleTime = False
    CycleTime = 30
    IsPaused = False
    Text = 'On/Off'
    TextX = 0
    TextY = 0
    SwapGraphic = False
    LEDHeight = 8
    BtnColorFaceHi = 14540253
    BtnColorFaceSh = clBtnFace
    BtnBevel2Width = 3
    LEDColorOn = clLime
    LEDColorOff = clSilver
  end
  object LED2: _LED
    Left = 240
    Top = 0
    Width = 104
    Height = 24
    IsTransparent = True
    SignalName = 'eufs_can1.PDO_Status.PDO2000_K2_Status'
    Caption = 'K2 Switch'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    ParentFont = False
    LED.ColorOff = clRed
    LED_Position = lpLeft
    Spacing = 5
    Checked = False
    Flashing = False
    Frequency = ff1Hz
    StatusInt = 0
    StatusBit = 0
    GroupIndex = 0
    Mode = mIndicator
    Threshold = 1.000000000000000000
    BinarySignal = True
  end
  object LED3: _LED
    Left = 240
    Top = 24
    Width = 104
    Height = 24
    IsTransparent = True
    SignalName = 'eufs_can1.PDO_Status.PDO2000_K3_Status'
    Caption = 'K3 Button'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    ParentFont = False
    LED.ColorOff = clRed
    LED_Position = lpLeft
    Spacing = 5
    Checked = False
    Flashing = False
    Frequency = ff1Hz
    StatusInt = 0
    StatusBit = 0
    GroupIndex = 0
    Mode = mIndicator
    Threshold = 1.000000000000000000
    BinarySignal = True
  end
  object HorizontalBar1: _HorizontalBar
    Left = 56
    Top = 0
    Width = 176
    Height = 72
    SignalName = 'eufs_can1.PDO_Status.PDO2006_Radio_Quality'
    Digit = 0
    LimitUpper = 90.000000000000000000
    LimitLower = -1.000000000000000000
    HintOptions = [hoValue, hoMin, hoMax, hoMinMaxDate, hoMinMaxTime]
    SectorSettings.Sector1To = 333
    SectorSettings.Sector2From = 334
    SectorSettings.Sector2To = 666
    SectorSettings.Sector3From = 667
    SectorSettings.Sector3To = 1000
    SectorSettings.Offset = 0
    SectorSettings.WidthOffset = 0
    SignalSettings.DigitalTo = 100000
    SignalSettings.Name1 = 'Radio Quality'
    SignalSettings.Name2 = 'eufs_can1.PDO_Status.PDO2006_Radio_Quality'
    SignalSettings.ValueFormat = '##0.0'
    SignalSettings.ValueTo = 100.000000000000000000
    SignalSettings.ValueUnit = '%'
    MinMax.MaxColor = clRed
    MinMax.MinColor = clLime
    MinMax.UseSectorCol = False
    MinMax.MinVisible = False
    MinMax.MaxVisible = False
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    FontUnit.Charset = DEFAULT_CHARSET
    FontUnit.Color = clWindowText
    FontUnit.Height = -13
    FontUnit.Name = 'Tahoma'
    FontUnit.Style = []
    FontValue.Charset = DEFAULT_CHARSET
    FontValue.Color = clLime
    FontValue.Height = -13
    FontValue.Name = 'Tahoma'
    FontValue.Style = []
    BarSettings.Style = bsBar
    BarSettings.Font.Charset = DEFAULT_CHARSET
    BarSettings.Font.Color = clBtnHighlight
    BarSettings.Font.Height = -11
    BarSettings.Font.Name = 'Tahoma'
    BarSettings.Font.Style = []
    BarSettings.Options = []
    BarSettings.Color = clLime
    BarSettings.BkColor = clSilver
    BarSettings.Bevel.BorderSpacing = 1
    BarSettings.Bevel.BevelLine = blNone
    BarSettings.Bevel.BorderWidth = 1
    BarSettings.Bevel.BkColor = clBlack
    BarSettings.Bevel.BorderStyle = 0
    BarSettings.Bevel.SurfaceGrad.Visible = False
    BarSettings.Bevel.SurfaceGrad.Style = gsHorizontal1
    BarSettings.Bevel.GradientStyle = gsHorizontal1
    BarSettings.Bevel.StartColor = 14869218
    BarSettings.Bevel.StopColor = 9408399
    BarSettings.MinHeight = 10
    BarSettings.MinWidth = 100
    BevelInner.BorderSpacing = 0
    BevelInner.BevelLine = blNone
    BevelInner.BorderStyle = 0
    BevelInner.SurfaceGrad.Visible = False
    BevelInner.SurfaceGrad.Style = gsHorizontal1
    BevelInner.GradientStyle = gsHorizontal1
    BevelInner.StartColor = 14869218
    BevelInner.StopColor = 9408399
    BevelOuter.BevelLine = blNone
    BevelOuter.BorderWidth = 1
    BevelOuter.ColorShadowFrom = clBlack
    BevelOuter.ColorShadowTo = clBlack
    BevelOuter.ColorHighLightFrom = clBlack
    BevelOuter.ColorHighLightTo = clBlack
    BevelOuter.BorderStyle = 2
    BevelOuter.SurfaceGrad.ColorFrom = 15790320
    BevelOuter.SurfaceGrad.ColorTo = 9933713
    BevelOuter.SurfaceGrad.Visible = True
    BevelOuter.SurfaceGrad.Style = gsHorizontal1
    BevelOuter.EnableGradient = True
    BevelOuter.GradientStyle = gsHorizontal1
    BevelOuter.StartColor = 15790320
    BevelOuter.StopColor = 9933713
    BevelValue.BorderSpacing = 0
    BevelValue.BevelLine = blNone
    BevelValue.BorderWidth = 1
    BevelValue.BkColor = clBlack
    BevelValue.BorderStyle = 0
    BevelValue.SurfaceGrad.Visible = False
    BevelValue.SurfaceGrad.Style = gsHorizontal1
    BevelValue.GradientStyle = gsHorizontal1
    BevelValue.StartColor = 14869218
    BevelValue.StopColor = 9408399
    Options = [opBevelOuter, opName1, opUnit, opValue, opOverflow]
  end
  object LED4: _LED
    Left = 240
    Top = 48
    Width = 112
    Height = 24
    IsTransparent = True
    SignalName = 'eufs_can1.PDO_Status.PDO2007_Radio_Comms_Interuption'
    Caption = 'Comms'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    ParentFont = False
    LED.ColorOn = clRed
    LED.ColorOff = clLime
    LED_Position = lpLeft
    Spacing = 5
    Checked = False
    Flashing = False
    Frequency = ff1Hz
    StatusInt = 0
    StatusBit = 0
    GroupIndex = 0
    Mode = mIndicator
    Threshold = 1.000000000000000000
    BinarySignal = True
  end
  object Label1: _Label
    Left = 8
    Top = 8
    Width = 80
    Height = 16
    Caption = 'RES'
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
end

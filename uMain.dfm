object Form2: TForm2
  Left = 0
  Top = 0
  Caption = 'FModel - Sensor Dynamics'
  ClientHeight = 679
  ClientWidth = 1096
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -12
  Font.Name = 'Segoe UI'
  Font.Style = []
  Menu = MainMenu1
  Scaled = False
  OnClose = FormClose
  OnCreate = FormCreate
  OnMouseWheel = FormMouseWheel
  OnResize = FormResize
  TextHeight = 15
  object Image1: TImage
    Left = 0
    Top = 76
    Width = 1096
    Height = 547
    Cursor = crSizeAll
    Hint = 
      #1055#1077#1088#1077#1090#1072#1089#1082#1080#1074#1072#1081#1090#1077' '#1092#1086#1085' '#1076#1083#1103' '#1089#1076#1074#1080#1075#1072' '#1086#1089#1077#1081', '#1082#1088#1072#1089#1085#1091#1102' '#1090#1086#1095#1082#1091' '#8212' '#1076#1083#1103' '#1077#1105' '#1087#1077#1088#1077#1084 +
      #1077#1097#1077#1085#1080#1103
    Align = alClient
    ParentShowHint = False
    ShowHint = True
    OnMouseDown = Image1MouseDown
    OnMouseMove = Image1MouseMove
    OnMouseUp = Image1MouseUp
  end
  object ProcessPanel: TPanel
    Left = 0
    Top = 0
    Width = 1096
    Height = 76
    Align = alTop
    BevelOuter = bvNone
    TabOrder = 0
    object OffsetLabel: TLabel
      Left = 92
      Top = 47
      Width = 12
      Height = 15
      Caption = '20'
    end
    object ResultLabel: TLabel
      Left = 330
      Top = 47
      Width = 3
      Height = 15
    end
    object ProcessLabel: TLabel
      Left = 220
      Top = 14
      Width = 24
      Height = 15
      Caption = 't = 0'
    end
    object AngleCorrectionCheckBox: TCheckBox
      Left = 650
      Top = 45
      Width = 220
      Height = 20
      Caption = #1050#1086#1088#1088#1077#1082#1094#1080#1103' '#1091#1075#1083#1072
      Checked = True
      State = cbChecked
      TabOrder = 4
      OnClick = AngleCorrectionCheckBoxClick
    end
    object OffsetMinusButton: TButton
      Left = 8
      Top = 40
      Width = 34
      Height = 28
      Caption = '-'
      TabOrder = 2
      OnClick = OffsetMinusButtonClick
    end
    object OffsetPlusButton: TButton
      Left = 48
      Top = 40
      Width = 34
      Height = 28
      Caption = '+'
      TabOrder = 3
      OnClick = OffsetPlusButtonClick
    end
    object InitButton: TButton
      Left = 8
      Top = 8
      Width = 88
      Height = 28
      Caption = #1048#1085#1080#1090
      TabOrder = 0
      OnClick = InitButtonClick
    end
    object StartStopButton: TButton
      Left = 104
      Top = 8
      Width = 100
      Height = 28
      Caption = #1057#1090#1072#1088#1090
      TabOrder = 1
      OnClick = StartStopButtonClick
    end
  end
  object HistoryPanel: TPanel
    Left = 0
    Top = 623
    Width = 1096
    Height = 56
    Align = alBottom
    BevelOuter = bvNone
    TabOrder = 1
    object HistoryLabel: TLabel
      Left = 0
      Top = 0
      Width = 1096
      Height = 18
      Align = alTop
      AutoSize = False
    end
    object HistorySlider: TTrackBar
      Left = 0
      Top = 18
      Width = 1096
      Height = 38
      Align = alClient
      Enabled = False
      Max = 1
      PageSize = 10
      Frequency = 10
      TabOrder = 0
      OnChange = HistorySliderChange
    end
  end
  object IntegratorTimer: TTimer
    Enabled = False
    Interval = 10
    OnTimer = IntegratorTimerTimer
    Left = 80
    Top = 96
  end
  object MainMenu1: TMainMenu
    Left = 32
    Top = 48
    object FileMenu: TMenuItem
      Caption = #1060#1072#1081#1083
      object ParametersMenu: TMenuItem
        Caption = #1055#1072#1088#1072#1084#1077#1090#1088#1099
        ShortCut = 16467
        OnClick = ParametersMenuClick
      end
    end
  end
end

unit uMain;

interface

uses
  Winapi.Windows, System.SysUtils, System.Classes, System.Math, Vcl.Graphics,
  Vcl.Controls, Vcl.Forms, Vcl.Dialogs, Vcl.ExtCtrls, Vcl.Menus, System.UITypes,
  System.IniFiles, Vcl.StdCtrls, Vcl.ComCtrls, Winapi.MMSystem, uMotionModel,
  uGuidanceController;

const
  MeasurementHistorySize = 200;
  TrajectoryCapacity = 300;

type
  TVec3 = array[0..2] of Double;
  TTrajectoryPoint = record
    X, Y: Double;
  end;
  TTrajectoryBuffer = array[0..TrajectoryCapacity-1] of TTrajectoryPoint;
  TSimulationFrame = record
    Model: TMotionModel;
    Controller: TGuidanceController;
    Time, Dt, MinimumDistance: Double;
    Step: Int64;
    StoppedOnDistance: Boolean;
    Trajectory: TTrajectoryBuffer;
    TrajectoryCount: Integer;
  end;

  TForm2 = class(TForm)
    Image1: TImage;
    MainMenu1: TMainMenu;
    FileMenu: TMenuItem;
    ParametersMenu: TMenuItem;
    ProcessPanel: TPanel;
    InitButton: TButton;
    StartStopButton: TButton;
    ProcessLabel: TLabel;
    OffsetMinusButton: TButton;
    OffsetPlusButton: TButton;
    OffsetLabel: TLabel;
    ResultLabel: TLabel;
    AngleCorrectionCheckBox: TCheckBox;
    IntegratorTimer: TTimer;
    HistoryPanel: TPanel;
    HistoryLabel: TLabel;
    HistorySlider: TTrackBar;
    procedure HistorySliderChange(Sender: TObject);
    procedure AngleCorrectionCheckBoxClick(Sender: TObject);
    procedure OffsetMinusButtonClick(Sender: TObject);
    procedure OffsetPlusButtonClick(Sender: TObject);
    procedure InitButtonClick(Sender: TObject);
    procedure StartStopButtonClick(Sender: TObject);
    procedure IntegratorTimerTimer(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure FormResize(Sender: TObject);
    procedure FormMouseWheel(Sender: TObject; Shift: TShiftState;
      WheelDelta: Integer; MousePos: TPoint; var Handled: Boolean);
    procedure ParametersMenuClick(Sender: TObject);
    procedure FormClose(Sender: TObject; var Action: TCloseAction);
    procedure Image1MouseDown(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure Image1MouseMove(Sender: TObject; Shift: TShiftState; X, Y: Integer);
    procedure Image1MouseUp(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
  private
    FReady: Boolean;
    FStoppedOnDistance: Boolean;
    FMinimumDistance: Double;
    FModel: TMotionModel;
    FController: TGuidanceController;
    FTrajectory: TTrajectoryBuffer;
    FHistory: array[0..MeasurementHistorySize-1] of TSimulationFrame;
    FHistoryNext, FHistoryCount, FHistoryIndex: Integer;
    FUpdatingHistory: Boolean;
    FTrajectoryCount: Integer;
    FCounterFrequency, FLastCounter: Int64;
    FTimerResolutionActive, FIntegrating: Boolean;
    FSimulationTime, FLastDt: Double;
    FPendingTime: Double;
    FStepCount: Int64;
    FDragging: Boolean;
    FDraggingTarget: Boolean;
    FTargetOffsetX, FTargetOffsetY: Double;
    FOriginX, FOriginY, FDragOffsetX, FDragOffsetY: Integer;
    FMuXYZ, FFiXYZ: TVec3;
    procedure StopProcess;
    procedure AddHistoryFrame;
    procedure UpdateHistoryControls;
    function GetDisplayFrame: TSimulationFrame;
    function GetTargetX: Double;
    function GetTargetY: Double;
    procedure UpdateProcessLabel;
    procedure LoadSettings;
    procedure SaveSettings;
    function NearTarget(X, Y: Integer): Boolean;
    procedure MoveDraggedPoint(X, Y: Integer);
    function d2x(X, Y, Z: Double): Integer;
    function d2y(X, Y, Z: Double): Integer;
    procedure Line3D(X1, Y1, Z1, X2, Y2, Z2: Double;
      Color: TColor; Width: Integer = 1);
    procedure RedrawImage;
    procedure AddTrajectoryPoint;
    procedure DrawTrajectory(const Frame: TSimulationFrame;
      LeftX, RightX, BottomY, TopY: Double);
  public
    destructor Destroy; override;
    procedure Integrate(dt: Double);
    function HistoryFrame(Index: Integer): TSimulationFrame;
    property HistoryCount: Integer read FHistoryCount;
    property DisplayFrame: TSimulationFrame read GetDisplayFrame;
    property SimulationTime: Double read FSimulationTime;
    property LastDt: Double read FLastDt;
    property StepCount: Int64 read FStepCount;
    property Model: TMotionModel read FModel;
    property Controller: TGuidanceController read FController;
    property TargetX: Double read GetTargetX;
    property TargetY: Double read GetTargetY;
    property TrajectoryCount: Integer read FTrajectoryCount;
    property MinimumDistance: Double read FMinimumDistance;
  end;

var
  Form2: TForm2;

implementation

{$R *.dfm}

const
  DefaultPixelsPerMetre = 10.0;
  MaxIntegrationStep = 0.01;
  MaxCatchUpSteps = 100;

// Инициализирует форму, модель и параметры отображения.
procedure TForm2.FormCreate(Sender: TObject);
begin
  if not QueryPerformanceFrequency(FCounterFrequency) or (FCounterFrequency <= 0) then
    raise Exception.Create('Системный счётчик времени недоступен.');
  FHistoryIndex := -1;
  Image1.Stretch := False;
  Image1.Picture.Bitmap.PixelFormat := pf24bit;
  // BINS projection coefficients: X right, Y up, Z along the view normal.
  FMuXYZ[0] := DefaultPixelsPerMetre;
  FMuXYZ[1] := FMuXYZ[0];
  FMuXYZ[2] := 0;
  FFiXYZ[0] := 0;
  FFiXYZ[1] := 90;
  FFiXYZ[2] := 0;
  FOriginX := Image1.Width div 2;
  FOriginY := Image1.Height div 2;
  FModel.Init;
  FController.Init;
  LoadSettings;
  AngleCorrectionCheckBox.Checked := FController.AngleCorrectionEnabled;
  FReady := True;
  InitButtonClick(nil);
end;

// Останавливает расчёт и освобождает форму.
destructor TForm2.Destroy;
begin
  FReady := False;
  StopProcess;
  inherited;
end;

// Останавливает таймер и разрешает просмотр истории.
procedure TForm2.StopProcess;
begin
  FPendingTime := 0;
  if Assigned(IntegratorTimer) then
    IntegratorTimer.Enabled := False;
  if FTimerResolutionActive then
  begin
    timeEndPeriod(1);
    FTimerResolutionActive := False;
  end;
  if Assigned(StartStopButton) then
    StartStopButton.Caption := 'Старт';
  if FReady then UpdateHistoryControls;
end;

// Возвращает сохранённый шаг по порядковому номеру в истории.
function TForm2.HistoryFrame(Index: Integer): TSimulationFrame;
begin
  if (Index < 0) or (Index >= FHistoryCount) then
    raise EArgumentOutOfRangeException.Create('History index');
  Result := FHistory[(FHistoryNext - FHistoryCount + Index +
    MeasurementHistorySize) mod MeasurementHistorySize];
end;

// Возвращает текущее состояние или выбранный кадр истории.
function TForm2.GetDisplayFrame: TSimulationFrame;
begin
  if FHistoryIndex >= 0 then Exit(HistoryFrame(FHistoryIndex));
  Result.Model := FModel;
  Result.Controller := FController;
  Result.Time := FSimulationTime;
  Result.Dt := FLastDt;
  Result.Step := FStepCount;
  Result.MinimumDistance := FMinimumDistance;
  Result.StoppedOnDistance := FStoppedOnDistance;
  Result.Trajectory := FTrajectory;
  Result.TrajectoryCount := FTrajectoryCount;
end;

// Сохраняет очередной шаг в кольцевой массив истории.
procedure TForm2.AddHistoryFrame;
begin
  FHistoryIndex := -1;
  FHistory[FHistoryNext] := GetDisplayFrame;
  FHistoryNext := (FHistoryNext + 1) mod MeasurementHistorySize;
  if FHistoryCount < MeasurementHistorySize then Inc(FHistoryCount);
end;

// Обновляет положение слайдера и подпись выбранного шага.
procedure TForm2.UpdateHistoryControls;
var Frame: TSimulationFrame;
begin
  FUpdatingHistory := True;
  try
    HistorySlider.Max := Max(1, FHistoryCount - 1);
    if FHistoryIndex < 0 then
      HistorySlider.Position := Max(0, FHistoryCount - 1)
    else
      HistorySlider.Position := FHistoryIndex;
    HistorySlider.Enabled := (FHistoryCount > 0) and not IntegratorTimer.Enabled;
    if FHistoryCount = 0 then
      HistoryLabel.Caption := 'История: нет измерений'
    else
    begin
      Frame := GetDisplayFrame;
      HistoryLabel.Caption := Format('История: %d / %d; шаг %d; t = %.4f с',
        [HistorySlider.Position + 1, FHistoryCount, Frame.Step, Frame.Time]);
      if IntegratorTimer.Enabled then
        HistoryLabel.Caption := HistoryLabel.Caption + ' — запись';
    end;
  finally
    FUpdatingHistory := False;
  end;
end;

// Показывает состояние и траекторию выбранного шага.
procedure TForm2.HistorySliderChange(Sender: TObject);
begin
  if not FReady or FUpdatingHistory or IntegratorTimer.Enabled or
    (FHistoryCount = 0) then Exit;
  FHistoryIndex := Min(HistorySlider.Position, FHistoryCount - 1);
  UpdateHistoryControls;
  UpdateProcessLabel;
  RedrawImage;
end;

// Возвращает координату X неподвижной цели.
function TForm2.GetTargetX: Double;
begin
  Result := FModel.Target.X;
end;

// Возвращает координату Y неподвижной цели.
function TForm2.GetTargetY: Double;
begin
  Result := FModel.Target.Y;
end;

// Обновляет время, номер шага и результат расчёта на форме.
procedure TForm2.UpdateProcessLabel;
var Frame: TSimulationFrame;
begin
  Frame := GetDisplayFrame;
  OffsetLabel.Caption := Format('Смещение датчика: %.0f°', [Frame.Model.HeadingOffsetDegrees]);
  ProcessLabel.Caption := Format('t = %.3f с; dt = %.4f с; шагов: %d',
    [Frame.Time, Frame.Dt, Frame.Step]);
  if FHistoryIndex >= 0 then ProcessLabel.Caption := ProcessLabel.Caption + ' — просмотр';
  if Frame.StoppedOnDistance then
  begin
    ProcessLabel.Caption := ProcessLabel.Caption + ' — стоп: дистанция растёт';
    ResultLabel.Caption := Format('Минимальный промах: %.3f м', [Frame.MinimumDistance]);
  end
  else
    ResultLabel.Caption := '';
end;

// Уменьшает смещение датчика на один градус.
procedure TForm2.OffsetMinusButtonClick(Sender: TObject);
begin
  FModel.HeadingOffsetDegrees := FModel.HeadingOffsetDegrees - 1;
  UpdateProcessLabel;
end;

// Переключает адаптивную поправку угла и сохраняет настройку.
procedure TForm2.AngleCorrectionCheckBoxClick(Sender: TObject);
begin
  if not FReady then Exit;
  FController.AngleCorrectionEnabled := AngleCorrectionCheckBox.Checked;
  SaveSettings;
end;

// Увеличивает смещение датчика на один градус.
procedure TForm2.OffsetPlusButtonClick(Sender: TObject);
begin
  FModel.HeadingOffsetDegrees := FModel.HeadingOffsetDegrees + 1;
  UpdateProcessLabel;
end;

// Сбрасывает движение, время расчёта, траекторию и историю.
procedure TForm2.InitButtonClick(Sender: TObject);
begin
  StopProcess;
  FHistoryCount := 0;
  FHistoryNext := 0;
  FHistoryIndex := -1;
  FModel.ResetMotion;
  FController.Init;
  FController.AngleCorrectionEnabled := AngleCorrectionCheckBox.Checked;
  FMinimumDistance := Hypot(FModel.Target.X - FModel.X, FModel.Target.Y - FModel.Y);
  FStoppedOnDistance := False;
  FTrajectoryCount := 0;
  FSimulationTime := 0;
  FLastDt := 0;
  FStepCount := 0;
  FLastCounter := 0;
  UpdateHistoryControls;
  UpdateProcessLabel;
  RedrawImage;
end;

// Запускает или приостанавливает расчёт без учёта времени паузы.
procedure TForm2.StartStopButtonClick(Sender: TObject);
begin
  if IntegratorTimer.Enabled then
  begin
    StopProcess;
    Exit;
  end;
  // Start a fresh interval on every resume: paused time is excluded.
  FHistoryIndex := -1;
  if not QueryPerformanceCounter(FLastCounter) then
    raise Exception.Create('Не удалось прочитать системное время.');
  FTimerResolutionActive := timeBeginPeriod(1) = TIMERR_NOERROR;
  FMinimumDistance := Hypot(FModel.Target.X - FModel.X, FModel.Target.Y - FModel.Y);
  FStoppedOnDistance := False;
  IntegratorTimer.Enabled := True;
  StartStopButton.Caption := 'Стоп';
  UpdateHistoryControls;
  UpdateProcessLabel;
  RedrawImage;
end;

// Догоняет прошедшее время малыми шагами и обновляет изображение.
procedure TForm2.IntegratorTimerTimer(Sender: TObject);
var
  Counter: Int64;
  dt, StepDt: Double;
  TotalSteps: Int64;
  I, StepsToRun: Integer;
begin
  if not IntegratorTimer.Enabled or FIntegrating then
    Exit;
  FIntegrating := True;
  try
    try
      if not QueryPerformanceCounter(Counter) then
        raise Exception.Create('Не удалось прочитать системное время.');
      dt := (Counter - FLastCounter) / FCounterFrequency;
      if dt <= 0 then
        Exit;
      FLastCounter := Counter;
      FPendingTime := FPendingTime + dt;
      // Equal substeps avoid a tiny last remainder and preserve elapsed time.
      TotalSteps := Ceil(FPendingTime / MaxIntegrationStep);
      StepDt := FPendingTime / TotalSteps;
      StepsToRun := Integer(Min(TotalSteps, Int64(MaxCatchUpSteps)));
      for I := 1 to StepsToRun do
      begin
        Integrate(StepDt);
        if not IntegratorTimer.Enabled then
          Break;
      end;
      if IntegratorTimer.Enabled then
      begin
        if StepsToRun = TotalSteps then
          FPendingTime := 0
        else
          FPendingTime := FPendingTime - StepsToRun * StepDt;
      end;
      // A long backlog is retained for the next timer event, keeping UI responsive.
      UpdateHistoryControls;
      UpdateProcessLabel;
      RedrawImage;
    except
      StopProcess;
      raise;
    end;
  finally
    FIntegrating := False;
  end;
end;

// Выполняет шаг модели, сохраняет историю и проверяет условие остановки.
procedure TForm2.Integrate(dt: Double);
var
  DistanceBefore, DistanceAfter: Double;
begin
  DistanceBefore := Hypot(FModel.Target.X - FModel.X, FModel.Target.Y - FModel.Y);
  if IsNan(dt) or IsInfinite(dt) or (dt < 0) then
    raise EArgumentException.Create('dt must be finite and non-negative');
  if dt = 0 then Exit;
  // Связь модулей только здесь: измерение -> регулятор -> команда -> движение.
  FModel.W := FController.Update(dt, FModel.MeasureAngle, FModel.W);
  FModel.Integrate(dt);
  FHistoryIndex := -1;
  FLastDt := dt;
  Inc(FStepCount);
  AddTrajectoryPoint;
  FSimulationTime := FSimulationTime + dt;
  DistanceAfter := Hypot(FModel.Target.X - FModel.X, FModel.Target.Y - FModel.Y);
  FMinimumDistance := Min(FMinimumDistance, Min(DistanceBefore, DistanceAfter));
  if IntegratorTimer.Enabled and (DistanceAfter > DistanceBefore + 1E-9) then
  begin
    StopProcess;
    FStoppedOnDistance := True;
  end;
  AddHistoryFrame;
  if not FIntegrating then
  begin
    UpdateHistoryControls;
    UpdateProcessLabel;
    RedrawImage;
  end;
end;

// Добавляет положение модели в массив точек траектории.
procedure TForm2.AddTrajectoryPoint;
var
  I: Integer;
begin
  if FTrajectoryCount = Length(FTrajectory) then
  begin
    for I := 1 to High(FTrajectory) do
      FTrajectory[I - 1] := FTrajectory[I];
    Dec(FTrajectoryCount);
  end;
  FTrajectory[FTrajectoryCount].X := FModel.X;
  FTrajectory[FTrajectoryCount].Y := FModel.Y;
  Inc(FTrajectoryCount);
end;

// Рисует видимую часть траектории выбранного состояния.
procedure TForm2.DrawTrajectory(const Frame: TSimulationFrame;
  LeftX, RightX, BottomY, TopY: Double);
var
  I, PX, PY: Integer;
  X, Y, DX, DY, T0, T1: Double;

  // Ограничивает отрезок одной границей видимой области.
  function Clip(P, Q: Double): Boolean;
  var
    R: Double;
  begin
    if P = 0 then
      Exit(Q >= 0);
    R := Q / P;
    if P < 0 then
      T0 := Max(T0, R)
    else
      T1 := Min(T1, R);
    Result := T0 <= T1;
  end;

begin
  for I := 1 to Frame.TrajectoryCount - 1 do
  begin
    X := Frame.Trajectory[I - 1].X;
    Y := Frame.Trajectory[I - 1].Y;
    DX := Frame.Trajectory[I].X - X;
    DY := Frame.Trajectory[I].Y - Y;
    T0 := 0;
    T1 := 1;
    if Clip(-DX, X - LeftX) and Clip(DX, RightX - X) and
       Clip(-DY, Y - BottomY) and Clip(DY, TopY - Y) then
      Line3D(X + T0 * DX, Y + T0 * DY, 0,
        X + T1 * DX, Y + T1 * DY, 0, clBlue);
  end;
  with Image1.Picture.Bitmap.Canvas do
  begin
    Brush.Style := bsSolid;
    Brush.Color := clBlue;
    for I := 0 to Frame.TrajectoryCount - 1 do
      if (Frame.Trajectory[I].X >= LeftX) and (Frame.Trajectory[I].X <= RightX) and
         (Frame.Trajectory[I].Y >= BottomY) and (Frame.Trajectory[I].Y <= TopY) then
      begin
        PX := d2x(Frame.Trajectory[I].X, Frame.Trajectory[I].Y, 0);
        PY := d2y(Frame.Trajectory[I].X, Frame.Trajectory[I].Y, 0);
        FillRect(Rect(PX - 1, PY - 1, PX + 2, PY + 2));
      end;
    Brush.Style := bsClear;
  end;
end;

// Перерисовывает сцену при изменении размера формы.
procedure TForm2.FormResize(Sender: TObject);
begin
  if FReady then
  begin
    RedrawImage;
  end;
end;

// Меняет масштаб колёсиком мыши относительно указателя.
procedure TForm2.FormMouseWheel(Sender: TObject; Shift: TShiftState;
  WheelDelta: Integer; MousePos: TPoint; var Handled: Boolean);
var
  P: TPoint;
  WorldX, WorldY, NewScale: Double;
begin
  if not FReady then
    Exit;
  P := Image1.ScreenToClient(MousePos);
  if (P.X < 0) or (P.Y < 0) or (P.X >= Image1.Width) or
     (P.Y >= Image1.Height) then
    Exit;
  Handled := True;
  NewScale := EnsureRange(FMuXYZ[0] * Power(1.1, WheelDelta / 120.0), 0.1, 10000.0);
  if NewScale = FMuXYZ[0] then
    Exit;
  // Keep the world point under the cursor fixed while zooming both axes equally.
  WorldX := (P.X - FOriginX) / FMuXYZ[0];
  WorldY := (FOriginY - P.Y) / FMuXYZ[1];
  FOriginX := Round(P.X - WorldX * NewScale);
  FOriginY := Round(P.Y + WorldY * NewScale);
  FMuXYZ[0] := NewScale;
  FMuXYZ[1] := NewScale;
  if FDragging then
  begin
    FDragOffsetX := P.X - FOriginX;
    FDragOffsetY := P.Y - FOriginY;
    FTargetOffsetX := (P.X - FOriginX) / NewScale - FModel.Target.X;
    FTargetOffsetY := (FOriginY - P.Y) / NewScale - FModel.Target.Y;
  end;
  RedrawImage;
  SaveSettings;
end;

// Загружает параметры модели и отображения из INI-файла.
procedure TForm2.LoadSettings;
var
  Ini: TMemIniFile;
  Scale: Double;
begin
  Ini := TMemIniFile.Create(ChangeFileExt(ParamStr(0), '.ini'));
  try
    FController.AngleCorrectionEnabled := Ini.ReadBool('Controller', 'AngleCorrection', True);
    if TryStrToFloat(Ini.ReadString('View', 'PixelsPerMetre', '10'),
      Scale, TFormatSettings.Invariant) then
      if not IsNan(Scale) and not IsInfinite(Scale) and
        (Scale >= 0.1) and (Scale <= 10000) then
      begin
        FMuXYZ[0] := Scale;
        FMuXYZ[1] := Scale;
      end;
    FOriginX := Ini.ReadInteger('View', 'OriginX', FOriginX);
    if TryStrToFloat(Ini.ReadString('Model', 'Diameter', '1'), Scale,
      TFormatSettings.Invariant) then
      if not IsNan(Scale) and not IsInfinite(Scale) and (Scale > 0) and (Scale <= 10000) then
        FModel.Diameter := Scale;
    if TryStrToFloat(Ini.ReadString('Target', 'Diameter', '1'), Scale,
      TFormatSettings.Invariant) then
      if not IsNan(Scale) and not IsInfinite(Scale) and (Scale > 0) and (Scale <= 10000) then
        FModel.Target.Diameter := Scale;
    FOriginY := Ini.ReadInteger('View', 'OriginY', FOriginY);
    if TryStrToFloat(Ini.ReadString('Target', 'X', '100'), Scale,
      TFormatSettings.Invariant) then
      if not IsNan(Scale) and not IsInfinite(Scale) and (Abs(Scale) <= 1E9) then
        FModel.Target.X := Scale;
    if TryStrToFloat(Ini.ReadString('Target', 'Y', '100'), Scale,
      TFormatSettings.Invariant) then
      if not IsNan(Scale) and not IsInfinite(Scale) and (Abs(Scale) <= 1E9) then
        FModel.Target.Y := Scale;
  finally
    Ini.Free;
  end;
end;

// Сохраняет параметры модели и отображения в INI-файле.
procedure TForm2.SaveSettings;
var
  Ini: TMemIniFile;
begin
  Ini := TMemIniFile.Create(ChangeFileExt(ParamStr(0), '.ini'));
  try
    Ini.WriteString('View', 'PixelsPerMetre',
      FloatToStr(FMuXYZ[0], TFormatSettings.Invariant));
    Ini.DeleteKey('Controller', 'DelayCorrection');
    Ini.DeleteKey('Sensor', 'HeadingOffsetEnabled');
    Ini.WriteBool('Controller', 'AngleCorrection', FController.AngleCorrectionEnabled);
    Ini.WriteInteger('View', 'OriginX', FOriginX);
    Ini.WriteString('Model', 'Diameter', FloatToStr(FModel.Diameter, TFormatSettings.Invariant));
    Ini.WriteString('Target', 'Diameter', FloatToStr(FModel.Target.Diameter, TFormatSettings.Invariant));
    Ini.WriteInteger('View', 'OriginY', FOriginY);
    Ini.WriteString('Target', 'X', FloatToStr(FModel.Target.X, TFormatSettings.Invariant));
    Ini.WriteString('Target', 'Y', FloatToStr(FModel.Target.Y, TFormatSettings.Invariant));
    Ini.UpdateFile;
  finally
    Ini.Free;
  end;
end;

// Останавливает расчёт и сохраняет настройки при закрытии формы.
procedure TForm2.FormClose(Sender: TObject; var Action: TCloseAction);
begin
  StopProcess;
  SaveSettings;
end;

// Начинает перетаскивание цели или системы координат.
procedure TForm2.Image1MouseDown(Sender: TObject; Button: TMouseButton;
  Shift: TShiftState; X, Y: Integer);
begin
  if Button <> mbLeft then
    Exit;
  FDraggingTarget := (FHistoryIndex < 0) and NearTarget(X, Y);
  FDragging := True;
  FTargetOffsetX := (X - FOriginX) / FMuXYZ[0] - FModel.Target.X;
  FTargetOffsetY := (FOriginY - Y) / FMuXYZ[1] - FModel.Target.Y;
  FDragOffsetX := X - FOriginX;
  FDragOffsetY := Y - FOriginY;
  SetCaptureControl(Image1);
  Image1.Cursor := crSizeAll;
end;

// Проверяет попадание указателя мыши в область цели.
function TForm2.NearTarget(X, Y: Integer): Boolean;
var
  DX, DY, HitRadius: Double;
begin
  DX := X - (FOriginX + FModel.Target.X * FMuXYZ[0]);
  DY := Y - (FOriginY - FModel.Target.Y * FMuXYZ[1]);
  HitRadius := Max(8.0, FModel.Target.Diameter * FMuXYZ[0] / 2);
  Result := Sqr(DX) + Sqr(DY) <= Sqr(HitRadius);
end;

// Смещает перетаскиваемую цель или начало координат.
procedure TForm2.MoveDraggedPoint(X, Y: Integer);
begin
  if FDraggingTarget then
  begin
    FModel.Target.X := (EnsureRange(X, 0, Image1.Width - 1) - FOriginX) /
      FMuXYZ[0] - FTargetOffsetX;
    FModel.Target.Y := (FOriginY - EnsureRange(Y, 0, Image1.Height - 1)) /
      FMuXYZ[1] - FTargetOffsetY;
  end
  else
  begin
    FOriginX := X - FDragOffsetX;
    FOriginY := Y - FDragOffsetY;
  end;
end;

// Обрабатывает перетаскивание и потерю захвата мыши.
procedure TForm2.Image1MouseMove(Sender: TObject; Shift: TShiftState; X, Y: Integer);
begin
  // A lost capture (for example, switching windows) must not leave dragging active.
  if FDragging and ((GetCaptureControl <> Image1) or not (ssLeft in Shift)) then
  begin
    FDragging := False;
    if GetCaptureControl = Image1 then
      SetCaptureControl(nil);
    SaveSettings;
  end;
  if FDragging then
  begin
    MoveDraggedPoint(X, Y);
    RedrawImage;
  end;
  Image1.Cursor := crSizeAll;
end;

// Завершает перетаскивание и сохраняет новое положение.
procedure TForm2.Image1MouseUp(Sender: TObject; Button: TMouseButton;
  Shift: TShiftState; X, Y: Integer);
begin
  if (Button <> mbLeft) or not FDragging then
    Exit;
  MoveDraggedPoint(X, Y);
  FDragging := False;
  SetCaptureControl(nil);
  Image1.Cursor := crSizeAll;
  RedrawImage;
  SaveSettings;
end;

// Открывает диалог параметров, проверяет и применяет введённые значения.
procedure TForm2.ParametersMenuClick(Sender: TObject);
var
  Values: TArray<string>;
  Scale, TX, TY, ModelDiameter, TargetDiameter: Double;
begin
  Values := TArray<string>.Create(FloatToStr(FMuXYZ[0]),
    FloatToStr(FModel.Target.X), FloatToStr(FModel.Target.Y),
    FloatToStr(FModel.Diameter), FloatToStr(FModel.Target.Diameter));
  while InputQuery('Параметры', ['Масштаб, пикселей на метр (0,1–10000):',
    'Красная точка X, м:', 'Красная точка Y, м:',
    'Диаметр движущейся модели, м:', 'Диаметр статической модели, м:'], Values) do
  begin
    if TryStrToFloat(Values[0], Scale) and TryStrToFloat(Values[1], TX) and
      TryStrToFloat(Values[2], TY) and TryStrToFloat(Values[3], ModelDiameter) and
      TryStrToFloat(Values[4], TargetDiameter) then
      if not IsNan(Scale) and not IsInfinite(Scale) and
         (Scale >= 0.1) and (Scale <= 10000) and
         not IsNan(TX) and not IsInfinite(TX) and (Abs(TX) <= 1E9) and
         not IsNan(TY) and not IsInfinite(TY) and (Abs(TY) <= 1E9) and
         not IsNan(ModelDiameter) and not IsInfinite(ModelDiameter) and
         (ModelDiameter > 0) and (ModelDiameter <= 10000) and
         not IsNan(TargetDiameter) and not IsInfinite(TargetDiameter) and
         (TargetDiameter > 0) and (TargetDiameter <= 10000) then
      begin
        FModel.Diameter := ModelDiameter;
        FModel.Target.Diameter := TargetDiameter;
        FModel.Target.X := TX;
        FModel.Target.Y := TY;
        FMuXYZ[0] := Scale;
        FMuXYZ[1] := Scale;
        RedrawImage;
        SaveSettings;
        Exit;
      end;
    MessageDlg('Масштаб: от 0,1 до 10000. Координаты: в пределах ±1 млрд м. Диаметры: больше 0, не более 10000 м.',
      mtError, [mbOK], 0);
  end;
end;

// Преобразует пространственные координаты в горизонтальную координату экрана.
function TForm2.d2x(X, Y, Z: Double): Integer;
begin
  // From BINS/uMainForm.pas, without the old +/-5 coordinate restriction.
  Result := Round(FMuXYZ[0] * X * Cos(DegToRad(FFiXYZ[0])) +
    FMuXYZ[1] * Y * Cos(DegToRad(FFiXYZ[1])) +
    FMuXYZ[2] * Z * Cos(DegToRad(FFiXYZ[2]))) + FOriginX;
end;

// Преобразует пространственные координаты в вертикальную координату экрана.
function TForm2.d2y(X, Y, Z: Double): Integer;
begin
  Result := -Round(FMuXYZ[0] * X * Sin(DegToRad(FFiXYZ[0])) +
    FMuXYZ[1] * Y * Sin(DegToRad(FFiXYZ[1])) +
    FMuXYZ[2] * Z * Sin(DegToRad(FFiXYZ[2]))) + FOriginY;
end;

// Проецирует и рисует пространственный отрезок заданным цветом.
procedure TForm2.Line3D(X1, Y1, Z1, X2, Y2, Z2: Double;
  Color: TColor; Width: Integer);
begin
  if IsNan(X1) or IsInfinite(X1) or IsNan(Y1) or IsInfinite(Y1) or
     IsNan(Z1) or IsInfinite(Z1) or IsNan(X2) or IsInfinite(X2) or
     IsNan(Y2) or IsInfinite(Y2) or IsNan(Z2) or IsInfinite(Z2) then
    Exit;
  with Image1.Picture.Bitmap.Canvas do
  begin
    Pen.Color := Color;
    Pen.Width := Width;
    MoveTo(d2x(X1, Y1, Z1), d2y(X1, Y1, Z1));
    LineTo(d2x(X2, Y2, Z2), d2y(X2, Y2, Z2));
  end;
end;

// Рисует оси, модели, траекторию и значения выбранного шага.
procedure TForm2.RedrawImage;
var
  Frame: TSimulationFrame;
  I, W, H, CX, CY: Integer;
  A, B, Radius, LeftX, RightX, BottomY, TopY, GridStep, Arrow, Angle: Double;
begin
  Frame := GetDisplayFrame;
  W := Image1.Width;
  H := Image1.Height;
  if (W <= 0) or (H <= 0) then
    Exit;
  CX := FOriginX;
  CY := FOriginY;
  LeftX := -CX / FMuXYZ[0];
  RightX := (W - 1 - CX) / FMuXYZ[0];
  BottomY := (CY - H + 1) / FMuXYZ[1];
  TopY := CY / FMuXYZ[1];
  Image1.Picture.Bitmap.SetSize(W, H);
  with Image1.Picture.Bitmap.Canvas do
  begin
    Brush.Style := bsSolid;
    Brush.Color := clWhite;
    FillRect(Rect(0, 0, W, H));
    Font.Name := 'Segoe UI';
    Font.Size := 9;
    Brush.Style := bsClear;
  end;

  // Keep the world scale fixed; resizing changes only the visible extent.
  GridStep := Power(10, Ceil(Log10(50 / FMuXYZ[0])));
  for I := Ceil(LeftX / GridStep) to Floor(RightX / GridStep) do
    Line3D(I * GridStep, BottomY, 0, I * GridStep, TopY, 0, $00EEEEEE);
  for I := Ceil(BottomY / GridStep) to Floor(TopY / GridStep) do
    Line3D(LeftX, I * GridStep, 0, RightX, I * GridStep, 0, $00EEEEEE);
  Arrow := 8 / FMuXYZ[0];
  Line3D(LeftX, 0, 0, RightX, 0, 0, clRed);
  Line3D(RightX, 0, 0, RightX - Arrow, Arrow / 2, 0, clRed);
  Line3D(RightX, 0, 0, RightX - Arrow, -Arrow / 2, 0, clRed);
  Line3D(0, BottomY, 0, 0, TopY, 0, clGreen);
  Line3D(0, TopY, 0, -Arrow / 2, TopY - Arrow, 0, clGreen);
  Line3D(0, TopY, 0, Arrow / 2, TopY - Arrow, 0, clGreen);

  // Circle follows the model position, using its configured diameter.
  DrawTrajectory(Frame, LeftX, RightX, BottomY, TopY);
  Radius := Frame.Model.Diameter / 2;
  // Skip off-screen geometry before converting growing world coordinates to pixels.
  if (Frame.Model.X + Radius >= LeftX) and (Frame.Model.X - Radius <= RightX) and
     (Frame.Model.Y + Radius >= BottomY) and (Frame.Model.Y - Radius <= TopY) then
    for I := 0 to 71 do
    begin
      A := 2 * Pi * I / 72;
      B := 2 * Pi * (I + 1) / 72;
      Line3D(Frame.Model.X + Radius * Cos(A), Frame.Model.Y + Radius * Sin(A), 0,
        Frame.Model.X + Radius * Cos(B), Frame.Model.Y + Radius * Sin(B), 0, clBlue);
    end;

  Radius := Frame.Model.Target.Diameter / 2;
  if (Frame.Model.Target.X + Radius >= LeftX) and (Frame.Model.Target.X - Radius <= RightX) and
     (Frame.Model.Target.Y + Radius >= BottomY) and (Frame.Model.Target.Y - Radius <= TopY) then
    with Image1.Picture.Bitmap.Canvas do
    begin
      Pen.Color := clRed;
      Pen.Width := 1;
      Brush.Style := bsSolid;
      Brush.Color := clRed;
      Ellipse(d2x(Frame.Model.Target.X - Radius, Frame.Model.Target.Y, 0),
        d2y(Frame.Model.Target.X, Frame.Model.Target.Y + Radius, 0),
        d2x(Frame.Model.Target.X + Radius, Frame.Model.Target.Y, 0),
        d2y(Frame.Model.Target.X, Frame.Model.Target.Y - Radius, 0));
      Brush.Style := bsClear;
    end;

  with Image1.Picture.Bitmap.Canvas do
  begin
    Font.Color := clBlack;
    TextOut(8, 8, Format('Масштаб: %g пкс/м; диаметры: %g / %g м',
      [FMuXYZ[0], Frame.Model.Diameter, Frame.Model.Target.Diameter]));
    TextOut(8, 28, Format('Область: %g x %g м; сетка: %g м',
      [W / FMuXYZ[0], H / FMuXYZ[1], GridStep]));
    TextOut(8, 48, Format('X = %.2f м; Y = %.2f м; Fi = %.3f рад; V = %.1f км/ч',
      [Frame.Model.X, Frame.Model.Y, Frame.Model.Fi, Frame.Model.V / KmhToMetresPerSecond]));
    TextOut(8, 68, Format('Красная точка: X = %.2f м; Y = %.2f м', [Frame.Model.Target.X, Frame.Model.Target.Y]));
    if Frame.Model.TryAngleToPoint(Frame.Model.Target.X, Frame.Model.Target.Y, Angle) then
      TextOut(8, 88, Format('Угол на красную точку: %.2f°', [Angle]))
    else
      TextOut(8, 88, 'Угол на красную точку: не определён');
    TextOut(8, 108, 'Задержка: не оценивается');
    if Frame.Controller.AngleCorrectionEnabled then
    begin
      TextOut(8, 108, Format('Адаптивная поправка BiasEstimate: %.2f°',
        [RadToDeg(Frame.Controller.BiasEstimate)]));
    end
    else
    begin
      TextOut(8, 108, 'Адаптивная поправка: отключена');
    end;
    TextOut(8, 128, Format('Угол на цель после коррекции: %.2f°',
      [RadToDeg(Frame.Controller.CorrectedAngle)]));
    TextOut(8, 148, Format('Измеренный угол: %.4f°',
      [RadToDeg(Frame.Model.MeasuredAngle)]));
    TextOut(8, 168, Format('P = %.6f; I = %.6f; D = %.6f рад/с',
      [Frame.Controller.Wprop, Frame.Controller.Wint, Frame.Controller.Wdiff]));
    TextOut(8, 188, Format('Выход регулятора W = %.6f рад/с',
      [Frame.Controller.W]));
    TextOut(8, 208, Format('Входная угловая скорость W = %.6f рад/с',
      [Frame.Controller.InputW]));
    TextOut(8, CY + 4, Format('%g м', [LeftX]));
    TextOut(W - 90, CY + 4, Format('%g м', [RightX]));
    TextOut(W - 25, CY - 20, 'X');
    TextOut(CX + 10, 8, Format('Y  %g м', [TopY]));
    TextOut(CX + 10, H - 22, Format('%g м', [BottomY]));
    TextOut(CX + 10, CY + 4, '0; Z');
    Brush.Style := bsSolid;
  end;
  Image1.Invalidate;
end;

end.

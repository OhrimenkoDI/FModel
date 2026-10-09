unit uMain;

interface

uses
  Winapi.Windows, System.SysUtils, System.Classes, System.Math, Vcl.Graphics,
  Vcl.Controls, Vcl.Forms, Vcl.Dialogs, Vcl.ExtCtrls, Vcl.Menus, System.UITypes,
  System.IniFiles, Vcl.StdCtrls, Vcl.ComCtrls, Winapi.MMSystem, uMotionModel,
  uAdaptiveRegulator, uParameters;

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
    Controller: TAdaptiveRegulator;
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
    FController: TAdaptiveRegulator;
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
    FDraggingReferencePoint: Boolean;
    FReferencePointOffsetX, FReferencePointOffsetY: Double;
    FOriginX, FOriginY, FDragOffsetX, FDragOffsetY: Integer;
    FMuXYZ, FFiXYZ: TVec3;
    procedure StopProcess;
    procedure AddHistoryFrame;
    procedure UpdateHistoryControls;
    function GetDisplayFrame: TSimulationFrame;
    function GetReferencePointX: Double;
    function GetReferencePointY: Double;
    procedure UpdateProcessLabel;
    function GetParameterValues: TParameterValues;
    procedure ApplyParameterValues(const Values: TParameterValues);
    procedure LoadSettings;
    procedure SaveSettings;
    function NearReferencePoint(X, Y: Integer): Boolean;
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
    property Controller: TAdaptiveRegulator read FController;
    property ReferencePointX: Double read GetReferencePointX;
    property ReferencePointY: Double read GetReferencePointY;
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

// Возвращает координату X неподвижной опорной точки.
function TForm2.GetReferencePointX: Double;
begin
  Result := FModel.ReferencePoint.X;
end;

// Возвращает координату Y неподвижной опорной точки.
function TForm2.GetReferencePointY: Double;
begin
  Result := FModel.ReferencePoint.Y;
end;

// Обновляет время, номер шага и результат расчёта на форме.
procedure TForm2.UpdateProcessLabel;
var Frame: TSimulationFrame;
begin
  Frame := GetDisplayFrame;
  OffsetLabel.Caption := Format('Смещение датчика: %.0f°', [Frame.Model.SensorBiasDegrees]);
  ProcessLabel.Caption := Format('t = %.3f с; dt = %.4f с; шагов: %d',
    [Frame.Time, Frame.Dt, Frame.Step]);
  if FHistoryIndex >= 0 then ProcessLabel.Caption := ProcessLabel.Caption + ' — просмотр';
  if Frame.StoppedOnDistance then
  begin
    ProcessLabel.Caption := ProcessLabel.Caption + ' — стоп: дистанция растёт';
    ResultLabel.Caption := Format('Мин. расстояние: %.3f м', [Frame.MinimumDistance]);
  end
  else
    ResultLabel.Caption := '';
end;

// Уменьшает смещение датчика на один градус.
procedure TForm2.OffsetMinusButtonClick(Sender: TObject);
begin
  FModel.SensorBiasDegrees := FModel.SensorBiasDegrees - 1;
  UpdateProcessLabel;
  SaveSettings;
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
  FModel.SensorBiasDegrees := FModel.SensorBiasDegrees + 1;
  UpdateProcessLabel;
  SaveSettings;
end;

// Сбрасывает движение, время расчёта, траекторию и историю.
procedure TForm2.InitButtonClick(Sender: TObject);
begin
  StopProcess;
  FHistoryCount := 0;
  FHistoryNext := 0;
  FHistoryIndex := -1;
  FModel.ResetMotion;
  FController.Reset;
  FController.AngleCorrectionEnabled := AngleCorrectionCheckBox.Checked;
  FMinimumDistance := Hypot(FModel.ReferencePoint.X - FModel.X, FModel.ReferencePoint.Y - FModel.Y);
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
  FMinimumDistance := Hypot(FModel.ReferencePoint.X - FModel.X, FModel.ReferencePoint.Y - FModel.Y);
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
  DistanceBefore := Hypot(FModel.ReferencePoint.X - FModel.X, FModel.ReferencePoint.Y - FModel.Y);
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
  DistanceAfter := Hypot(FModel.ReferencePoint.X - FModel.X, FModel.ReferencePoint.Y - FModel.Y);
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
    FReferencePointOffsetX := (P.X - FOriginX) / NewScale - FModel.ReferencePoint.X;
    FReferencePointOffsetY := (FOriginY - P.Y) / NewScale - FModel.ReferencePoint.Y;
  end;
  RedrawImage;
  SaveSettings;
end;

// Собирает настройки формы, модели и регулятора; углы для интерфейса — в градусах.
function TForm2.GetParameterValues: TParameterValues;
begin
  Result[0] := FMuXYZ[0];
  Result[1] := FModel.ReferencePoint.InitialX;
  Result[2] := FModel.ReferencePoint.InitialY;
  Result[3] := FModel.Diameter;
  Result[4] := FModel.ReferencePoint.Diameter;
  Result[5] := FModel.AngleCount;
  Result[6] := FModel.SensorBiasDegrees;
  Result[7] := FModel.ReferencePoint.SpeedKmh;
  Result[8] := FModel.ReferencePoint.HeadingDegrees;
  Result[9] := FController.Parameters.Kp;
  Result[10] := FController.Parameters.Ki;
  Result[11] := FController.Parameters.Kd;
  Result[12] := FController.Parameters.IntegralLimit;
  Result[13] := FController.Parameters.BiasAdaptationGain;
  Result[14] := FController.Parameters.BiasAngularSpeedWeight;
  Result[15] := FController.Parameters.BiasAdaptationStart;
  Result[16] := FController.Parameters.BiasFilterTime;
  Result[17] := RadToDeg(FController.Parameters.BiasErrorGate);
  Result[18] := RadToDeg(FController.Parameters.BiasRateLimit);
  Result[19] := RadToDeg(FController.Parameters.BiasLimit);
  Result[20] := FController.Parameters.DerivativeFilterTime;
end;

// Передаёт проверенные настройки их владельцам, не связывая модель с регулятором.
procedure TForm2.ApplyParameterValues(const Values: TParameterValues);
begin
  FMuXYZ[0] := Values[0];
  FModel.ReferencePoint.InitialX := Values[1];
  FModel.ReferencePoint.InitialY := Values[2];
  FModel.Diameter := Values[3];
  FModel.ReferencePoint.Diameter := Values[4];
  FModel.AngleCount := Round(Values[5]);
  FModel.SensorBiasDegrees := Values[6];
  FModel.ReferencePoint.SpeedKmh := Values[7];
  FModel.ReferencePoint.HeadingDegrees := Values[8];
  FController.Parameters.Kp := Values[9];
  FController.Parameters.Ki := Values[10];
  FController.Parameters.Kd := Values[11];
  FController.Parameters.IntegralLimit := Values[12];
  FController.Parameters.BiasAdaptationGain := Values[13];
  FController.Parameters.BiasAngularSpeedWeight := Values[14];
  FController.Parameters.BiasAdaptationStart := Values[15];
  FController.Parameters.BiasFilterTime := Values[16];
  FController.Parameters.BiasErrorGate := DegToRad(Values[17]);
  FController.Parameters.BiasRateLimit := DegToRad(Values[18]);
  FController.Parameters.BiasLimit := DegToRad(Values[19]);
  FController.Parameters.DerivativeFilterTime := Values[20];
  FMuXYZ[1] := FMuXYZ[0];
end;

// Загружает параметры из INI; отсутствующие и некорректные значения заменяются исходными.
procedure TForm2.LoadSettings;
var Ini: TMemIniFile; Values: TParameterValues;
begin
  Ini := TMemIniFile.Create(ChangeFileExt(ParamStr(0), '.ini'));
  try
    Values := DefaultParameterValues;
    ReadParameterValues(Ini, Values);
    ApplyParameterValues(Values);
    FController.AngleCorrectionEnabled := Ini.ReadBool('Controller', 'AngleCorrection', True);
    FOriginX := Ini.ReadInteger('View', 'OriginX', FOriginX);
    FOriginY := Ini.ReadInteger('View', 'OriginY', FOriginY);
  finally Ini.Free; end;
end;

// Сохраняет параметры обеих моделей, регулятора и отображения в INI-файле.
procedure TForm2.SaveSettings;
var Ini: TMemIniFile;
begin
  Ini := TMemIniFile.Create(ChangeFileExt(ParamStr(0), '.ini'));
  try
    WriteParameterValues(Ini, GetParameterValues);
    Ini.DeleteKey('Controller', 'DelayCorrection');
    Ini.DeleteKey('Sensor', 'HeadingOffsetEnabled');
    Ini.WriteBool('Controller', 'AngleCorrection', FController.AngleCorrectionEnabled);
    Ini.WriteInteger('View', 'OriginX', FOriginX);
    Ini.WriteInteger('View', 'OriginY', FOriginY);
    Ini.EraseSection('Target');
    Ini.UpdateFile;
  finally Ini.Free; end;
end;

// Останавливает расчёт и сохраняет настройки при закрытии формы.
procedure TForm2.FormClose(Sender: TObject; var Action: TCloseAction);
begin
  StopProcess;
  SaveSettings;
end;

// Начинает перетаскивание опорной точки или системы координат.
procedure TForm2.Image1MouseDown(Sender: TObject; Button: TMouseButton;
  Shift: TShiftState; X, Y: Integer);
begin
  if Button <> mbLeft then
    Exit;
  FDraggingReferencePoint := (FHistoryIndex < 0) and NearReferencePoint(X, Y);
  FDragging := True;
  FReferencePointOffsetX := (X - FOriginX) / FMuXYZ[0] - FModel.ReferencePoint.X;
  FReferencePointOffsetY := (FOriginY - Y) / FMuXYZ[1] - FModel.ReferencePoint.Y;
  FDragOffsetX := X - FOriginX;
  FDragOffsetY := Y - FOriginY;
  SetCaptureControl(Image1);
  Image1.Cursor := crSizeAll;
end;

// Проверяет попадание указателя мыши в область опорной точки.
function TForm2.NearReferencePoint(X, Y: Integer): Boolean;
var
  DX, DY, HitRadius: Double;
begin
  DX := X - (FOriginX + FModel.ReferencePoint.X * FMuXYZ[0]);
  DY := Y - (FOriginY - FModel.ReferencePoint.Y * FMuXYZ[1]);
  HitRadius := Max(8.0, FModel.ReferencePoint.Diameter * FMuXYZ[0] / 2);
  Result := Sqr(DX) + Sqr(DY) <= Sqr(HitRadius);
end;

// Смещает перетаскиваемую опорную точку или начало координат.
procedure TForm2.MoveDraggedPoint(X, Y: Integer);
begin
  if FDraggingReferencePoint then
  begin
    FModel.ReferencePoint.X := (EnsureRange(X, 0, Image1.Width - 1) - FOriginX) /
      FMuXYZ[0] - FReferencePointOffsetX;
    FModel.ReferencePoint.Y := (FOriginY - EnsureRange(Y, 0, Image1.Height - 1)) /
      FMuXYZ[1] - FReferencePointOffsetY;
    FModel.ReferencePoint.InitialX := FModel.ReferencePoint.X;
    FModel.ReferencePoint.InitialY := FModel.ReferencePoint.Y;
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

// Приостанавливает расчёт; подтверждённые настройки сохраняет и применяет с инициализацией.
procedure TForm2.ParametersMenuClick(Sender: TObject);
var Values: TParameterValues;
begin
  StopProcess;
  Values := GetParameterValues;
  if not EditParameters(Values) then Exit;
  ApplyParameterValues(Values);
  InitButtonClick(nil);
  SaveSettings;
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
  I, W, H, CX, CY, LineHeight, InfoWidth, LabelCount: Integer;
  InfoLines: TArray<string>;
  LabelRects: array[0..6] of TRect;
  A, B, Radius, LeftX, RightX, BottomY, TopY, GridStep, Arrow, Angle: Double;

  // Размещает подпись оси в пределах изображения без пересечения с другими надписями.
  procedure DrawAxisLabel(X, Y: Integer; const S: string);
  var R: TRect; J, LabelWidth, LabelHeight: Integer;
  begin
    with Image1.Picture.Bitmap.Canvas do
    begin
      LabelWidth := Image1.Picture.Bitmap.Canvas.TextWidth(S);
      LabelHeight := Image1.Picture.Bitmap.Canvas.TextHeight(S);
      if (LabelWidth + 8 > W) or (LabelHeight + 8 > H) then Exit;
      X := EnsureRange(X, 4, W - LabelWidth - 4);
      Y := EnsureRange(Y, 4, H - LabelHeight - 4);
      R := Rect(X - 3, Y - 3, X + LabelWidth + 3, Y + LabelHeight + 3);
      for J := 0 to LabelCount - 1 do
        if (R.Left < LabelRects[J].Right) and (R.Right > LabelRects[J].Left) and
           (R.Top < LabelRects[J].Bottom) and (R.Bottom > LabelRects[J].Top) then Exit;
      LabelRects[LabelCount] := R;
      Inc(LabelCount);
      TextOut(X, Y, S);
    end;
  end;
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

  Radius := Frame.Model.ReferencePoint.Diameter / 2;
  if (Frame.Model.ReferencePoint.X + Radius >= LeftX) and (Frame.Model.ReferencePoint.X - Radius <= RightX) and
     (Frame.Model.ReferencePoint.Y + Radius >= BottomY) and (Frame.Model.ReferencePoint.Y - Radius <= TopY) then
    with Image1.Picture.Bitmap.Canvas do
    begin
      Pen.Color := clRed;
      Pen.Width := 1;
      Brush.Style := bsSolid;
      Brush.Color := clRed;
      Ellipse(d2x(Frame.Model.ReferencePoint.X - Radius, Frame.Model.ReferencePoint.Y, 0),
        d2y(Frame.Model.ReferencePoint.X, Frame.Model.ReferencePoint.Y + Radius, 0),
        d2x(Frame.Model.ReferencePoint.X + Radius, Frame.Model.ReferencePoint.Y, 0),
        d2y(Frame.Model.ReferencePoint.X, Frame.Model.ReferencePoint.Y - Radius, 0));
      Brush.Style := bsClear;
    end;

  // Каждому показателю отведена своя строка; высота зависит от фактического шрифта.
  SetLength(InfoLines, 12);
  InfoLines[0] := Format('Масштаб: %.3f пкс/м; диаметры: %g / %g м',
    [FMuXYZ[0], Frame.Model.Diameter, Frame.Model.ReferencePoint.Diameter]);
  InfoLines[1] := Format('Область: %.2f x %.2f м; сетка: %g м',
    [W / FMuXYZ[0], H / FMuXYZ[1], GridStep]);
  InfoLines[2] := Format('X = %.2f м; Y = %.2f м; Fi = %.3f рад; V = %.1f км/ч',
    [Frame.Model.X, Frame.Model.Y, Frame.Model.Fi, Frame.Model.V / KmhToMetresPerSecond]);
  InfoLines[3] := Format('Опорная точка: X = %.2f м; Y = %.2f м; Fi = %.1f°; V = %.1f км/ч',
    [Frame.Model.ReferencePoint.X, Frame.Model.ReferencePoint.Y,
     RadToDeg(Frame.Model.ReferencePoint.Fi), Frame.Model.ReferencePoint.V / KmhToMetresPerSecond]);
  if Frame.Model.TryAngleToPoint(Frame.Model.ReferencePoint.X, Frame.Model.ReferencePoint.Y, Angle) then
    InfoLines[4] := Format('Угол на опорную точку: %.2f°', [Angle])
  else
    InfoLines[4] := 'Угол на опорную точку: не определён';
  InfoLines[5] := 'Задержка: не оценивается';
  if Frame.Controller.AngleCorrectionEnabled then
    InfoLines[6] := Format('Поправка измерения: %.2f°',
      [RadToDeg(Frame.Controller.AdaptiveAngleCorrection)])
  else
    InfoLines[6] := 'Поправка измерения: отключена';
  InfoLines[7] := Format('Измеренный угол после коррекции: %.2f°',
    [RadToDeg(Frame.Controller.CorrectedAngle)]);
  InfoLines[8] := Format('Измеренный угол: %.4f°', [RadToDeg(Frame.Model.MeasuredAngle)]);
  InfoLines[9] := Format('P = %.6f; I = %.6f; D = %.6f рад/с',
    [Frame.Controller.Wprop, Frame.Controller.Wint, Frame.Controller.Wdiff]);
  InfoLines[10] := Format('Выход регулятора W = %.6f рад/с', [Frame.Controller.W]);
  InfoLines[11] := Format('Входная угловая скорость W = %.6f рад/с', [Frame.Controller.InputW]);
  with Image1.Picture.Bitmap.Canvas do
  begin
    Font.Color := clBlack;
    LineHeight := TextHeight('Ag') + 5;
    InfoWidth := 0;
    for I := 0 to High(InfoLines) do
      InfoWidth := Max(InfoWidth, TextWidth(InfoLines[I]));
    LabelRects[0] := Rect(0, 0, Min(W, InfoWidth + 16), Min(H, 16 + Length(InfoLines) * LineHeight));
    LabelCount := 1;
    // Непрозрачный фон отделяет показатели от сетки, осей и траекторий.
    Brush.Style := bsSolid;
    Brush.Color := clWhite;
    FillRect(LabelRects[0]);
    for I := 0 to High(InfoLines) do
      TextOut(8, 8 + I * LineHeight, InfoLines[I]);
    DrawAxisLabel(8, CY + 4, Format('%.2f м', [LeftX]));
    DrawAxisLabel(W - TextWidth(Format('%.2f м', [RightX])) - 8, CY + 4,
      Format('%.2f м', [RightX]));
    DrawAxisLabel(W - 25, CY - 20, 'X');
    DrawAxisLabel(CX + 10, 8, Format('Y  %.2f м', [TopY]));
    DrawAxisLabel(CX + 10, H - LineHeight - 4, Format('%.2f м', [BottomY]));
    DrawAxisLabel(CX + 10, CY + 4, '0; Z');
  end;
  Image1.Invalidate;
end;

end.

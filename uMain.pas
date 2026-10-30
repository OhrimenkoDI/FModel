unit uMain;

interface

uses
  Winapi.Windows, System.SysUtils, System.Classes, System.Math, Vcl.Graphics,
  Vcl.Controls, Vcl.Forms, Vcl.Dialogs, Vcl.ExtCtrls, Vcl.Menus, System.UITypes,
  System.IniFiles, Vcl.StdCtrls, Vcl.ComCtrls, Winapi.MMSystem, uMotionModel,
  uPID, uParameters, uGateNavigation, uAdaptiveGlide, uGateNavigationPlus, uRecordedPlant;

const
  MeasurementHistorySize = 200;
  TrajectoryCapacity = 300;

type
  TVec3 = array[0..2] of Double;
  TTrajectoryPoint = record
    X, Y: Double;
  end;
  TTrajectoryBuffer = array[0..TrajectoryCapacity-1] of TTrajectoryPoint;
  TApproachResult = record
    Finished: Boolean;
    StopTime, Distance, MinimumDistance: Double;
  end;
  TSimulationFrame = record
    Model: TMotionModel;
    Gate: TGateSimulation;
    GatePlus: TGatePlusSimulation;
    GatePlusReferencePoint: TReferencePoint;
    GatePlusResult: TApproachResult;
    GatePlusTrajectory: TTrajectoryBuffer;
    GatePlusTrajectoryCount: Integer;
    AdaptiveModel: TMotionModel;
    AdaptiveController: TAdaptiveState;
    AdaptiveResult: TApproachResult;
    AdaptiveMinimumDistance: Double;
    AdaptiveTrajectory: TTrajectoryBuffer;
    AdaptiveTrajectoryCount: Integer;
    GateReferencePoint: TReferencePoint;
    PIDResult, GateResult: TApproachResult;
    GateTrajectoryCount: Integer;
    GateTrajectory: TTrajectoryBuffer;
    Controller: TPIDState;
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
    AutoStopCheckBox: TCheckBox;
    IntegratorTimer: TTimer;
    HistoryPanel: TPanel;
    HistoryLabel: TLabel;
    HistorySlider: TTrackBar;
    procedure HistorySliderChange(Sender: TObject);
    procedure AngleCorrectionCheckBoxClick(Sender: TObject);
    procedure AutoStopCheckBoxClick(Sender: TObject);
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
    FAutoStopEnabled: Boolean;
    FAutoStopDistance: Double;
    FMinimumDistance: Double;
    FModel: TMotionModel;
    FGate: TGateSimulation;
    FGatePlus: TGatePlusSimulation;
    FGatePlusReferencePoint: TReferencePoint;
    FGatePlusResult: TApproachResult;
    FGatePlusTrajectory: TTrajectoryBuffer;
    FGatePlusTrajectoryCount: Integer;
    FAdaptiveModel: TMotionModel;
    FAdaptiveController: TAdaptiveState;
    FAdaptiveResult: TApproachResult;
    FAdaptiveMinimumDistance: Double;
    FAdaptiveTrajectory: TTrajectoryBuffer;
    FAdaptiveTrajectoryCount: Integer;
    FGateReferencePoint: TReferencePoint;
    FPIDResult, FGateResult: TApproachResult;
    FGateTrajectoryCount: Integer;
    FGateTrajectory: TTrajectoryBuffer;
    FController: TPIDState;
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
    procedure DrawTrajectory(const Points: TTrajectoryBuffer; Count: Integer; Color: TColor;
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
    property Gate: TGateSimulation read FGate;
    property GatePlus: TGatePlusSimulation read FGatePlus;
    property Controller: TPIDState read FController;
    property ReferencePointX: Double read GetReferencePointX;
    property ReferencePointY: Double read GetReferencePointY;
    property TrajectoryCount: Integer read FTrajectoryCount;
    property AutoStopDistance: Double read FAutoStopDistance;
    property AutoStopEnabled: Boolean read FAutoStopEnabled;
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
  FAdaptiveModel.Init;
  FAdaptiveController.Init;
  PIDinit(FController);
  LoadSettings;
  AngleCorrectionCheckBox.Checked := FController.AngleCorrectionEnabled;
  AutoStopCheckBox.Checked := FAutoStopEnabled;
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
  Result.Gate := FGate;
  Result.GatePlus := FGatePlus;
  Result.GatePlusReferencePoint := FGatePlusReferencePoint;
  Result.GatePlusResult := FGatePlusResult;
  Result.GatePlusTrajectory := FGatePlusTrajectory;
  Result.GatePlusTrajectoryCount := FGatePlusTrajectoryCount;
  Result.AdaptiveModel := FAdaptiveModel;
  Result.AdaptiveController := FAdaptiveController;
  Result.AdaptiveResult := FAdaptiveResult;
  Result.AdaptiveMinimumDistance := FAdaptiveMinimumDistance;
  Result.AdaptiveTrajectory := FAdaptiveTrajectory;
  Result.AdaptiveTrajectoryCount := FAdaptiveTrajectoryCount;
  Result.GateReferencePoint := FGateReferencePoint;
  Result.PIDResult := FPIDResult;
  Result.GateResult := FGateResult;
  Result.GateTrajectoryCount := FGateTrajectoryCount;
  Result.GateTrajectory := FGateTrajectory;
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
  if Frame.Model.RecordedPlant.Enabled then
    OffsetLabel.Caption := Format('Общий датчик EDF: %.1f°', [Frame.Model.RecordedPlant.Parameters.CameraBias])
  else OffsetLabel.Caption := Format('Смещение датчика: %.0f°', [Frame.Model.SensorBiasDegrees]);
  ProcessLabel.Caption := Format('t = %.3f с; dt = %.4f с; шагов: %d',
    [Frame.Time, Frame.Dt, Frame.Step]);
  if FHistoryIndex >= 0 then ProcessLabel.Caption := ProcessLabel.Caption + ' — просмотр';
  if Frame.StoppedOnDistance then
    ProcessLabel.Caption := ProcessLabel.Caption + ' — все четыре расчёта завершены'
  else if Frame.PIDResult.Finished or Frame.GateResult.Finished or Frame.AdaptiveResult.Finished or Frame.GatePlusResult.Finished then
    ProcessLabel.Caption := ProcessLabel.Caption + ' — есть зафиксированные результаты';
  if Frame.PIDResult.Finished then
    ResultLabel.Caption := Format('PID: стоп %.3f с; min %.3f м',
      [Frame.PIDResult.StopTime, Frame.PIDResult.MinimumDistance])
  else ResultLabel.Caption := 'PID: расчёт';
  if Frame.GateResult.Finished then
    ResultLabel.Caption := ResultLabel.Caption + Format('   |   СВП: стоп %.3f с; min %.3f м',
      [Frame.GateResult.StopTime, Frame.GateResult.MinimumDistance])
  else ResultLabel.Caption := ResultLabel.Caption + '   |   СВП: расчёт';
  if Frame.AdaptiveResult.Finished then
    ResultLabel.Caption := ResultLabel.Caption + Format('   |   Adaptive: стоп %.3f с; min %.3f м',
      [Frame.AdaptiveResult.StopTime, Frame.AdaptiveResult.MinimumDistance])
  else ResultLabel.Caption := ResultLabel.Caption + '   |   Adaptive: расчёт';
  if Frame.GatePlusResult.Finished then
    ResultLabel.Caption := ResultLabel.Caption + Format('   |   СВП+: стоп %.3f с; min %.3f м',
      [Frame.GatePlusResult.StopTime, Frame.GatePlusResult.MinimumDistance])
  else ResultLabel.Caption := ResultLabel.Caption + '   |   СВП+: расчёт';
  OffsetMinusButton.Enabled := not FPIDResult.Finished and not FModel.RecordedPlant.Enabled;
  OffsetPlusButton.Enabled := not FPIDResult.Finished and not FModel.RecordedPlant.Enabled;
  AngleCorrectionCheckBox.Enabled := not FPIDResult.Finished;
end;

// Уменьшает смещение датчика на один градус.
procedure TForm2.OffsetMinusButtonClick(Sender: TObject);
begin
  if FPIDResult.Finished then Exit;
  FModel.SensorBiasDegrees := FModel.SensorBiasDegrees - 1;
  UpdateProcessLabel;
  SaveSettings;
end;

// Переключает адаптивную поправку угла и сохраняет настройку.
procedure TForm2.AngleCorrectionCheckBoxClick(Sender: TObject);
begin
  if not FReady then Exit;
  if FPIDResult.Finished then Exit;
  FController.AngleCorrectionEnabled := AngleCorrectionCheckBox.Checked;
  SaveSettings;
end;

// Включает или отключает автоостановку без сброса текущего расчёта.
procedure TForm2.AutoStopCheckBoxClick(Sender: TObject);
begin
  if not FReady then Exit;
  FAutoStopEnabled := AutoStopCheckBox.Checked;
  SaveSettings;
end;

// Увеличивает смещение датчика на один градус.
procedure TForm2.OffsetPlusButtonClick(Sender: TObject);
begin
  if FPIDResult.Finished then Exit;
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
  FAdaptiveModel.ReferencePoint := FModel.ReferencePoint;
  FAdaptiveModel.Diameter := FModel.Diameter;
  FAdaptiveModel.ResetMotion;
  FAdaptiveController.Reset;
  FAdaptiveMinimumDistance := Hypot(FAdaptiveModel.ReferencePoint.X, FAdaptiveModel.ReferencePoint.Y);
  FAdaptiveResult := Default(TApproachResult);
  FAdaptiveTrajectoryCount := 0;
  FGateReferencePoint := FModel.ReferencePoint;
  FGate.Reset(FGateReferencePoint.X, FGateReferencePoint.Y);
  FGate.RecordedPlant := FModel.RecordedPlant;
  FPIDResult := Default(TApproachResult);
  FGateResult := Default(TApproachResult);
  FGateTrajectoryCount := 0;
  FGatePlusReferencePoint := FModel.ReferencePoint;
  FGatePlus.Reset(FGatePlusReferencePoint.X, FGatePlusReferencePoint.Y);
  FGatePlus.RecordedPlant := FModel.RecordedPlant;
  if FGatePlus.RecordedPlant.Enabled then FGatePlus.Course := FGatePlus.Heading;
  FGatePlusResult := Default(TApproachResult);
  FGatePlusTrajectoryCount := 0;
  PIDreset(FController);
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
  if FStoppedOnDistance then Exit; // Completed results are reset only by Init.
  // Start a fresh interval on every resume: paused time is excluded.
  FHistoryIndex := -1;
  if not QueryPerformanceCounter(FLastCounter) then
    raise Exception.Create('Не удалось прочитать системное время.');
  FTimerResolutionActive := timeBeginPeriod(1) = TIMERR_NOERROR;
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
  DistanceBefore, DistanceAfter, GateBefore, GateAfter, AdaptiveBefore, AdaptiveAfter, GatePlusBefore, GatePlusAfter: Double;
  ObservedAngle, ObservedGyro: Double;

  procedure FinishApproach(var R: TApproachResult; Distance, Minimum: Double);
  begin
    R.Finished := True;
    R.StopTime := FSimulationTime;
    R.Distance := Distance;
    R.MinimumDistance := Minimum;
  end;

begin
  if IsNan(dt) or IsInfinite(dt) or (dt < 0) then
    raise EArgumentException.Create('dt must be finite and non-negative');
  if (dt = 0) or FStoppedOnDistance then Exit;
  DistanceBefore := Hypot(FModel.ReferencePoint.X - FModel.X, FModel.ReferencePoint.Y - FModel.Y);
  GateBefore := Hypot(FGateReferencePoint.X - FGate.X, FGateReferencePoint.Y - FGate.Y);
  GatePlusBefore := Hypot(FGatePlusReferencePoint.X - FGatePlus.X, FGatePlusReferencePoint.Y - FGatePlus.Y);

  AdaptiveBefore := Hypot(FAdaptiveModel.ReferencePoint.X-FAdaptiveModel.X, FAdaptiveModel.ReferencePoint.Y-FAdaptiveModel.Y);
  // Each algorithm owns a target copy; only an active pair advances in time.
  if not FPIDResult.Finished then
  begin
    ObservedAngle := RadToDeg(FModel.MeasureAngle);
    ObservedGyro := FModel.GyroDegrees;
    FModel.ApplyControl(PIDupdate(FController, dt, ObservedAngle, ObservedGyro), dt);
  end;
  if not FGateResult.Finished then
  begin
    FGate.Step(dt, FGateReferencePoint.X, FGateReferencePoint.Y,
      FGateReferencePoint.V * Cos(FGateReferencePoint.Fi),
      FGateReferencePoint.V * Sin(FGateReferencePoint.Fi));
    FGateReferencePoint.Integrate(dt);
  end;
  if not FGatePlusResult.Finished then
  begin
    FGatePlus.Step(dt, FGatePlusReferencePoint.X, FGatePlusReferencePoint.Y,
      FGatePlusReferencePoint.V * Cos(FGatePlusReferencePoint.Fi),
      FGatePlusReferencePoint.V * Sin(FGatePlusReferencePoint.Fi));
    FGatePlusReferencePoint.Integrate(dt);
  end;
  if not FAdaptiveResult.Finished then
  begin
    ObservedAngle := RadToDeg(FAdaptiveModel.MeasureAngle);
    ObservedGyro := FAdaptiveModel.GyroDegrees;
    FAdaptiveModel.ApplyControl(FAdaptiveController.Update(dt, ObservedAngle, ObservedGyro), dt);
  end;

  FHistoryIndex := -1;
  FLastDt := dt;
  Inc(FStepCount);
  // Append the terminal point before latching each independent completion flag.
  AddTrajectoryPoint;
  FSimulationTime := FSimulationTime + dt;
  if not FPIDResult.Finished then
  begin
    DistanceAfter := Hypot(FModel.ReferencePoint.X - FModel.X, FModel.ReferencePoint.Y - FModel.Y);
    FMinimumDistance := Min(FMinimumDistance, Min(DistanceBefore, DistanceAfter));
    if FAutoStopEnabled and (DistanceAfter < FAutoStopDistance) and
      (DistanceAfter > DistanceBefore + 1E-9) then
      FinishApproach(FPIDResult, DistanceAfter, FMinimumDistance);
  end;
  if not FGateResult.Finished then
  begin
    GateAfter := Hypot(FGateReferencePoint.X - FGate.X, FGateReferencePoint.Y - FGate.Y);
    if FAutoStopEnabled and (GateAfter < FAutoStopDistance) and
      (GateAfter > GateBefore + 1E-9) then
      FinishApproach(FGateResult, GateAfter, FGate.MinimumDistance);
  end;
  if not FAdaptiveResult.Finished then
  begin
    AdaptiveAfter := Hypot(FAdaptiveModel.ReferencePoint.X-FAdaptiveModel.X, FAdaptiveModel.ReferencePoint.Y-FAdaptiveModel.Y);
    FAdaptiveMinimumDistance := Min(FAdaptiveMinimumDistance,Min(AdaptiveBefore,AdaptiveAfter));
    if FAutoStopEnabled and (AdaptiveAfter < FAutoStopDistance) and
      (AdaptiveAfter > AdaptiveBefore + 1E-9) then
      FinishApproach(FAdaptiveResult,AdaptiveAfter,FAdaptiveMinimumDistance);
  end;
  if not FGatePlusResult.Finished then
  begin
    GatePlusAfter := Hypot(FGatePlusReferencePoint.X - FGatePlus.X, FGatePlusReferencePoint.Y - FGatePlus.Y);
    if FAutoStopEnabled and (GatePlusAfter < FAutoStopDistance) and
      (GatePlusAfter > GatePlusBefore + 1E-9) then
      FinishApproach(FGatePlusResult, GatePlusAfter, FGatePlus.MinimumDistance);
  end;
  FStoppedOnDistance := FPIDResult.Finished and FGateResult.Finished and
    FAdaptiveResult.Finished and FGatePlusResult.Finished;
  if FStoppedOnDistance then StopProcess;
  AddHistoryFrame;
  if not FIntegrating then
  begin
    UpdateHistoryControls;
    UpdateProcessLabel;
    RedrawImage;
  end;
end;

// Completed paths keep their terminal point and never shift out of the buffer.
procedure TForm2.AddTrajectoryPoint;
  procedure Append(var Points: TTrajectoryBuffer; var Count: Integer; X, Y: Double);
  var I: Integer;
  begin
    if Count = Length(Points) then
    begin
      for I := 1 to High(Points) do Points[I - 1] := Points[I];
      Dec(Count);
    end;
    Points[Count].X := X;
    Points[Count].Y := Y;
    Inc(Count);
  end;
begin
  if not FPIDResult.Finished then Append(FTrajectory, FTrajectoryCount, FModel.X, FModel.Y);
  if not FGateResult.Finished then Append(FGateTrajectory, FGateTrajectoryCount, FGate.X, FGate.Y);
  if not FGatePlusResult.Finished then Append(FGatePlusTrajectory, FGatePlusTrajectoryCount, FGatePlus.X, FGatePlus.Y);
  if not FAdaptiveResult.Finished then Append(FAdaptiveTrajectory,FAdaptiveTrajectoryCount,FAdaptiveModel.X,FAdaptiveModel.Y);
end;

// Рисует видимую часть траектории выбранного состояния.
procedure TForm2.DrawTrajectory(const Points: TTrajectoryBuffer; Count: Integer; Color: TColor;
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
  for I := 1 to Count - 1 do
  begin
    X := Points[I - 1].X;
    Y := Points[I - 1].Y;
    DX := Points[I].X - X;
    DY := Points[I].Y - Y;
    T0 := 0;
    T1 := 1;
    if Clip(-DX, X - LeftX) and Clip(DX, RightX - X) and
       Clip(-DY, Y - BottomY) and Clip(DY, TopY - Y) then
      Line3D(X + T0 * DX, Y + T0 * DY, 0,
        X + T1 * DX, Y + T1 * DY, 0, Color);
  end;
  with Image1.Picture.Bitmap.Canvas do
  begin
    Brush.Style := bsSolid;
    Brush.Color := Color;
    for I := 0 to Count - 1 do
      if (Points[I].X >= LeftX) and (Points[I].X <= RightX) and
         (Points[I].Y >= BottomY) and (Points[I].Y <= TopY) then
      begin
        PX := d2x(Points[I].X, Points[I].Y, 0);
        PY := d2y(Points[I].X, Points[I].Y, 0);
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
  Result[17] := FController.Parameters.BiasErrorGate;
  Result[18] := FController.Parameters.BiasRateLimit;
  Result[19] := FController.Parameters.BiasLimit;
  Result[20] := FController.Parameters.DerivativeFilterTime;
  Result[21] := FAutoStopDistance;
  Result[22] := FGate.Parameters.N;
  Result[23] := FGate.Parameters.LeadTime;
  Result[24] := FGate.Parameters.KLambda;
  Result[25] := FGate.Parameters.MaxRate;
  Result[26] := FGate.Parameters.FilterTime;
  Result[27] := FGate.Parameters.RateKp;
  Result[28] := FGate.Parameters.RateKi;
  Result[29] := FGate.Parameters.ActuatorTime;
  Result[30] := FGate.Parameters.SlipTime;
  Result[31] := FModel.SpeedKmh;
  Result[32] := FGate.Parameters.FreezeDistance;
  Result[33] := FGate.Parameters.SensorDelay;
  Result[34] := FGate.Parameters.SensorBias;
  Result[35] := FModel.InitialHeadingDegrees;
  Result[36] := FAdaptiveController.Parameters.CommandLimit;
  Result[37] := FAdaptiveController.Parameters.SlidingGain;
  Result[38] := FAdaptiveController.Parameters.SwitchingGain;
  Result[39] := FAdaptiveController.Parameters.Kappa;
  Result[40] := FAdaptiveController.Parameters.Lambda;
  Result[41] := FAdaptiveController.Parameters.IdentifierAlpha;
  Result[42] := FAdaptiveController.Parameters.CovarianceScale;
  Result[43] := FAdaptiveController.Parameters.DenominatorGuard;
  Result[44] := FAdaptiveController.Parameters.RateGain;
  Result[45] := FAdaptiveController.Parameters.AngleLimit;
  Result[46] := FAdaptiveController.Parameters.ReferenceCoefficients[0];
  Result[47] := FAdaptiveController.Parameters.ReferenceCoefficients[1];
  Result[48] := FAdaptiveController.Parameters.ReferenceCoefficients[2];
  Result[49] := FAdaptiveController.Parameters.IdentifierFilter[0];
  Result[50] := FAdaptiveController.Parameters.IdentifierFilter[1];
  Result[51] := FAdaptiveController.Parameters.IdentifierFilter[2];
  Result[52] := FAdaptiveModel.AngleCount;
  Result[53] := FAdaptiveModel.SensorBiasDegrees;
  Result[54] := FGatePlus.Parameters.ObserverSlipTime;
  Result[55] := FGatePlus.Parameters.LeadFilterTime;
  Result[56] := FGatePlus.Parameters.MaxCommandAcceleration;
  Result[57] := FGatePlus.Parameters.TerminalCommandAcceleration;
  Result[58] := FGatePlus.Parameters.TerminalMaxRate;
  Result[59] := FGatePlus.Parameters.PredictionTime;
  Result[60] := FGatePlus.Parameters.SafeRate;
  Result[61] := FGatePlus.Parameters.DetectorHalfAngle;
  Result[62] := FGatePlus.Parameters.InitialSlipDegrees;
  Result[63] := FGatePlus.Parameters.N;
  Result[64] := FGatePlus.Parameters.LeadTime;
  Result[65] := FGatePlus.Parameters.KLambda;
  Result[66] := FGatePlus.Parameters.MaxRate;
  Result[67] := FGatePlus.Parameters.FilterTime;
  Result[68] := FGatePlus.Parameters.RateKp;
  Result[69] := FGatePlus.Parameters.RateKi;
  Result[70] := FGatePlus.Parameters.ActuatorTime;
  Result[71] := FGatePlus.Parameters.SlipTime;
  Result[72] := FGatePlus.Parameters.FreezeDistance;
  Result[73] := FGatePlus.Parameters.SensorDelay;
  Result[74] := FGatePlus.Parameters.SensorBias;
  Result[75] := FModel.RecordedPlant.Parameters.Profile;
  Result[76] := FModel.RecordedPlant.Parameters.GainScale;
  Result[77] := FModel.RecordedPlant.Parameters.ExtraDelay;
  Result[78] := FModel.RecordedPlant.Parameters.FrequencyScale;
  Result[79] := FModel.RecordedPlant.Parameters.DampingScale;
  Result[80] := FModel.RecordedPlant.Parameters.CameraHz;
  Result[81] := FModel.RecordedPlant.Parameters.CameraDelay;
  Result[82] := FModel.RecordedPlant.Parameters.CompensationTime;
  Result[83] := FModel.RecordedPlant.Parameters.NoiseScale;
  Result[84] := FModel.RecordedPlant.Parameters.Seed;
  Result[85] := FModel.RecordedPlant.Parameters.CourseTime;
  Result[86] := FModel.RecordedPlant.Parameters.CameraBias;
  Result[87] := FModel.RecordedPlant.Parameters.OutputLimit;
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
  FController.Parameters.BiasErrorGate := Values[17];
  FController.Parameters.BiasRateLimit := Values[18];
  FController.Parameters.BiasLimit := Values[19];
  FController.Parameters.DerivativeFilterTime := Values[20];
  FAutoStopDistance := Values[21];
  FGate.Parameters.N := Values[22];
  FGate.Parameters.LeadTime := Values[23];
  FGate.Parameters.KLambda := Values[24];
  FGate.Parameters.MaxRate := Values[25];
  FGate.Parameters.FilterTime := Values[26];
  FGate.Parameters.RateKp := Values[27];
  FGate.Parameters.RateKi := Values[28];
  FGate.Parameters.ActuatorTime := Values[29];
  FGate.Parameters.SlipTime := Values[30];
  FGate.Parameters.SpeedKmh := Values[31];
  FGatePlus.Parameters.SpeedKmh := Values[31];
  FModel.SpeedKmh := Values[31];
  FAdaptiveModel.SpeedKmh := Values[31];
  FGate.Parameters.FreezeDistance := Values[32];
  FGate.Parameters.SensorDelay := Round(Values[33]);
  FGate.Parameters.SensorBias := Values[34];
  FModel.InitialHeadingDegrees := Values[35];
  FAdaptiveModel.InitialHeadingDegrees := Values[35];
  FGate.Parameters.InitialHeadingDegrees := Values[35];
  FGatePlus.Parameters.InitialHeadingDegrees := Values[35];
  FAdaptiveController.Parameters.CommandLimit := Values[36];
  FAdaptiveController.Parameters.SlidingGain := Values[37];
  FAdaptiveController.Parameters.SwitchingGain := Values[38];
  FAdaptiveController.Parameters.Kappa := Values[39];
  FAdaptiveController.Parameters.Lambda := Values[40];
  FAdaptiveController.Parameters.IdentifierAlpha := Values[41];
  FAdaptiveController.Parameters.CovarianceScale := Values[42];
  FAdaptiveController.Parameters.DenominatorGuard := Values[43];
  FAdaptiveController.Parameters.RateGain := Values[44];
  FAdaptiveController.Parameters.AngleLimit := Values[45];
  FAdaptiveController.Parameters.ReferenceCoefficients[0] := Values[46];
  FAdaptiveController.Parameters.ReferenceCoefficients[1] := Values[47];
  FAdaptiveController.Parameters.ReferenceCoefficients[2] := Values[48];
  FAdaptiveController.Parameters.IdentifierFilter[0] := Values[49];
  FAdaptiveController.Parameters.IdentifierFilter[1] := Values[50];
  FAdaptiveController.Parameters.IdentifierFilter[2] := Values[51];
  FAdaptiveModel.AngleCount := Round(Values[52]);
  FAdaptiveModel.SensorBiasDegrees := Values[53];
  FGatePlus.Parameters.ObserverSlipTime := Values[54];
  FGatePlus.Parameters.LeadFilterTime := Values[55];
  FGatePlus.Parameters.MaxCommandAcceleration := Values[56];
  FGatePlus.Parameters.TerminalCommandAcceleration := Values[57];
  FGatePlus.Parameters.TerminalMaxRate := Values[58];
  FGatePlus.Parameters.PredictionTime := Values[59];
  FGatePlus.Parameters.SafeRate := Values[60];
  FGatePlus.Parameters.DetectorHalfAngle := Values[61];
  FGatePlus.Parameters.InitialSlipDegrees := Values[62];
  FGatePlus.Parameters.N := Values[63];
  FGatePlus.Parameters.LeadTime := Values[64];
  FGatePlus.Parameters.KLambda := Values[65];
  FGatePlus.Parameters.MaxRate := Values[66];
  FGatePlus.Parameters.FilterTime := Values[67];
  FGatePlus.Parameters.RateKp := Values[68];
  FGatePlus.Parameters.RateKi := Values[69];
  FGatePlus.Parameters.ActuatorTime := Values[70];
  FGatePlus.Parameters.SlipTime := Values[71];
  FGatePlus.Parameters.FreezeDistance := Values[72];
  FGatePlus.Parameters.SensorDelay := Round(Values[73]);
  FGatePlus.Parameters.SensorBias := Values[74];
  FModel.RecordedPlant.Parameters.Profile := Round(Values[75]);
  FModel.RecordedPlant.Parameters.GainScale := Values[76];
  FModel.RecordedPlant.Parameters.ExtraDelay := Values[77];
  FModel.RecordedPlant.Parameters.FrequencyScale := Values[78];
  FModel.RecordedPlant.Parameters.DampingScale := Values[79];
  FModel.RecordedPlant.Parameters.CameraHz := Values[80];
  FModel.RecordedPlant.Parameters.CameraDelay := Values[81];
  FModel.RecordedPlant.Parameters.CompensationTime := Values[82];
  FModel.RecordedPlant.Parameters.NoiseScale := Values[83];
  FModel.RecordedPlant.Parameters.Seed := Round(Values[84]);
  FModel.RecordedPlant.Parameters.CourseTime := Values[85];
  FModel.RecordedPlant.Parameters.CameraBias := Values[86];
  FModel.RecordedPlant.Parameters.OutputLimit := Values[87];
  case Round(Values[75]) of
    1: Caption := 'FModel — по записи EDF, ось Roll';
    2: Caption := 'FModel — по записи EDF, ось Pitch';
    else Caption := 'FModel';
  end;
  FAdaptiveModel.RecordedPlant.Parameters := FModel.RecordedPlant.Parameters;
  FGate.RecordedPlant.Parameters := FModel.RecordedPlant.Parameters;
  FGatePlus.RecordedPlant.Parameters := FModel.RecordedPlant.Parameters;
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
    FAutoStopEnabled := Ini.ReadBool('Simulation', 'AutoStopEnabled', True);
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
    Ini.WriteBool('Simulation', 'AutoStopEnabled', FAutoStopEnabled);
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
  FDraggingReferencePoint := (FHistoryIndex < 0) and not FPIDResult.Finished and
    not FGateResult.Finished and not FAdaptiveResult.Finished and not FGatePlusResult.Finished and NearReferencePoint(X, Y);
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
    if FPIDResult.Finished or FGateResult.Finished or FAdaptiveResult.Finished or FGatePlusResult.Finished then Exit;
    FModel.ReferencePoint.X := (EnsureRange(X, 0, Image1.Width - 1) - FOriginX) /
      FMuXYZ[0] - FReferencePointOffsetX;
    FModel.ReferencePoint.Y := (FOriginY - EnsureRange(Y, 0, Image1.Height - 1)) /
      FMuXYZ[1] - FReferencePointOffsetY;
    FModel.ReferencePoint.InitialX := FModel.ReferencePoint.X;
    FModel.ReferencePoint.InitialY := FModel.ReferencePoint.Y;
    FGateReferencePoint := FModel.ReferencePoint;
    FGatePlusReferencePoint := FModel.ReferencePoint;
    FAdaptiveModel.ReferencePoint := FModel.ReferencePoint;
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

  // Both red target circles remain at their own completion coordinates.
  procedure DrawTarget(const Target: TReferencePoint; const Name: string;
    LabelColor: TColor; Finished: Boolean; LabelOffset: Integer);
  var R: Double; PX, PY: Integer; Caption: string;
  begin
    R := Max(Target.Diameter / 2, 4 / FMuXYZ[0]);
    if (Target.X + R < LeftX) or (Target.X - R > RightX) or
       (Target.Y + R < BottomY) or (Target.Y - R > TopY) then Exit;
    PX := d2x(Target.X, Target.Y, 0); PY := d2y(Target.X, Target.Y, 0);
    with Image1.Picture.Bitmap.Canvas do
    begin
      Pen.Color := clRed; Pen.Width := 2;
      Brush.Color := clRed;
      if Finished then Brush.Style := bsClear else Brush.Style := bsSolid;
      Ellipse(d2x(Target.X - R, Target.Y, 0), d2y(Target.X, Target.Y + R, 0),
        d2x(Target.X + R, Target.Y, 0), d2y(Target.X, Target.Y - R, 0));
      Caption := Name;
      if Finished then Caption := Caption + ' (стоп)';
      Brush.Style := bsClear; Font.Color := LabelColor;
      TextOut(PX + 8, PY + LabelOffset, Caption);
    end;
  end;
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
  DrawTrajectory(Frame.Trajectory, Frame.TrajectoryCount, clBlue, LeftX, RightX, BottomY, TopY);
  DrawTrajectory(Frame.GateTrajectory, Frame.GateTrajectoryCount, clPurple, LeftX, RightX, BottomY, TopY);
  DrawTrajectory(Frame.GatePlusTrajectory, Frame.GatePlusTrajectoryCount, clTeal, LeftX, RightX, BottomY, TopY);
  DrawTrajectory(Frame.AdaptiveTrajectory,Frame.AdaptiveTrajectoryCount,$000080FF,LeftX,RightX,BottomY,TopY);
  if (Frame.AdaptiveModel.X >= LeftX) and (Frame.AdaptiveModel.X <= RightX) and
     (Frame.AdaptiveModel.Y >= BottomY) and (Frame.AdaptiveModel.Y <= TopY) then
  begin
    Radius := 6 / FMuXYZ[0];
    Line3D(Frame.AdaptiveModel.X-Radius,Frame.AdaptiveModel.Y-Radius,0,
      Frame.AdaptiveModel.X+Radius,Frame.AdaptiveModel.Y+Radius,0,$000080FF,2);
    Line3D(Frame.AdaptiveModel.X-Radius,Frame.AdaptiveModel.Y+Radius,0,
      Frame.AdaptiveModel.X+Radius,Frame.AdaptiveModel.Y-Radius,0,$000080FF,2);
  end;
  if (Frame.Gate.X >= LeftX) and (Frame.Gate.X <= RightX) and
     (Frame.Gate.Y >= BottomY) and (Frame.Gate.Y <= TopY) then
  begin
    Radius := 7 / FMuXYZ[0];
    Line3D(Frame.Gate.X - Radius, Frame.Gate.Y, 0, Frame.Gate.X + Radius, Frame.Gate.Y, 0, clPurple, 2);
    Line3D(Frame.Gate.X, Frame.Gate.Y - Radius, 0, Frame.Gate.X, Frame.Gate.Y + Radius, 0, clPurple, 2);
    Line3D(Frame.Gate.X, Frame.Gate.Y, 0,
      Frame.Gate.X + 3 * Radius * Cos(DegToRad(Frame.Gate.Heading)),
      Frame.Gate.Y + 3 * Radius * Sin(DegToRad(Frame.Gate.Heading)), 0, clPurple, 2);
  end;
  if (Frame.GatePlus.X >= LeftX) and (Frame.GatePlus.X <= RightX) and
     (Frame.GatePlus.Y >= BottomY) and (Frame.GatePlus.Y <= TopY) then
  begin
    Radius := 7 / FMuXYZ[0];
    Line3D(Frame.GatePlus.X - Radius, Frame.GatePlus.Y, 0, Frame.GatePlus.X + Radius, Frame.GatePlus.Y, 0, clTeal, 2);
    Line3D(Frame.GatePlus.X, Frame.GatePlus.Y - Radius, 0, Frame.GatePlus.X, Frame.GatePlus.Y + Radius, 0, clTeal, 2);
    Line3D(Frame.GatePlus.X, Frame.GatePlus.Y, 0,
      Frame.GatePlus.X + 3 * Radius * Cos(DegToRad(Frame.GatePlus.Heading)),
      Frame.GatePlus.Y + 3 * Radius * Sin(DegToRad(Frame.GatePlus.Heading)), 0, clTeal, 2);
  end;
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

  if not Frame.PIDResult.Finished and not Frame.GateResult.Finished and not Frame.AdaptiveResult.Finished and not Frame.GatePlusResult.Finished then
    DrawTarget(Frame.Model.ReferencePoint, 'Цель PID / СВП / Adaptive / СВП+', clRed, False, -18)
  else
  begin
    DrawTarget(Frame.Model.ReferencePoint, 'Цель PID', clBlue, Frame.PIDResult.Finished, -18);
    DrawTarget(Frame.GateReferencePoint, 'Цель СВП', clPurple, Frame.GateResult.Finished, 8);
    DrawTarget(Frame.AdaptiveModel.ReferencePoint, 'Цель Adaptive', $000080FF, Frame.AdaptiveResult.Finished, 27);
    DrawTarget(Frame.GatePlusReferencePoint, 'Цель СВП+', clTeal, Frame.GatePlusResult.Finished, 46);
  end;
  // Каждому показателю отведена своя строка; высота зависит от фактического шрифта.
  SetLength(InfoLines, 22);
  InfoLines[0] := Format('Масштаб: %.3f пкс/м; диаметры: %g / %g м',
    [FMuXYZ[0], Frame.Model.Diameter, Frame.Model.ReferencePoint.Diameter]);
  InfoLines[1] := Format('Область: %.2f x %.2f м; сетка: %g м',
    [W / FMuXYZ[0], H / FMuXYZ[1], GridStep]);
  InfoLines[2] := Format('X = %.2f м; Y = %.2f м; Fi = %.3f°; V = %.1f км/ч',
    [Frame.Model.X, Frame.Model.Y, RadToDeg(Frame.Model.Fi), Frame.Model.V / KmhToMetresPerSecond]);
  InfoLines[3] := Format('Цель PID: X = %.2f м; Y = %.2f м; Fi = %.1f°; V = %.1f км/ч',
    [Frame.Model.ReferencePoint.X, Frame.Model.ReferencePoint.Y,
     RadToDeg(Frame.Model.ReferencePoint.Fi), Frame.Model.ReferencePoint.V / KmhToMetresPerSecond]);
  if Frame.Model.TryAngleToPoint(Frame.Model.ReferencePoint.X, Frame.Model.ReferencePoint.Y, Angle) then
    InfoLines[4] := Format('Угол на опорную точку: %.2f°', [Angle])
  else
    InfoLines[4] := 'Угол на опорную точку: не определён';
  InfoLines[5] := 'Задержка: не оценивается';
  if Frame.Controller.AngleCorrectionEnabled then
    InfoLines[6] := Format('Поправка измерения: %.2f°',
      [Frame.Controller.AdaptiveAngleCorrection])
  else
    InfoLines[6] := 'Поправка измерения: отключена';
  InfoLines[7] := Format('Измеренный угол после коррекции: %.2f°',
    [Frame.Controller.CorrectedAngle]);
  InfoLines[8] := Format('Измеренный угол: %.4f°', [RadToDeg(Frame.Model.MeasuredAngle)]);
  InfoLines[9] := Format('P = %.6f; I = %.6f; D = %.6f °/с',
    [Frame.Controller.Wprop, Frame.Controller.Wint, Frame.Controller.Wdiff]);
  InfoLines[10] := Format('Выход регулятора W = %.6f °/с', [Frame.Controller.W]);
  InfoLines[11] := Format('Входная угловая скорость W = %.6f °/с', [Frame.Controller.InputW]);
  InfoLines[12] := 'PID — синий; СВП — фиолетовый; Adaptive — оранжевый; СВП+ — бирюзовый';
  InfoLines[13] := Format('СВП: r = %.2f; r зад = %.2f °/с; скольжение = %.2f°',
    [Frame.Gate.Rate, Frame.Gate.Controller.CommandRate, RadToDeg(ArcTan2(Sin(DegToRad(Frame.Gate.Course - Frame.Gate.Heading)),
      Cos(DegToRad(Frame.Gate.Course - Frame.Gate.Heading))))]);
  InfoLines[14] := Format('СВП: мин. расстояние = %.3f м; скорость ЛВ = %.2f °/с',
    [Frame.Gate.MinimumDistance, Frame.Gate.Controller.LOSRate]);
  if Frame.GateResult.Finished then InfoLines[15] := Format('СВП: завершено; дистанция остановки %.3f м', [Frame.GateResult.Distance])
  else if Frame.Gate.Controller.Frozen then InfoLines[15] := 'СВП: команда заморожена (ближняя зона стенда)'
  else InfoLines[15] := 'СВП: наведение по углу и гироскопу';
  InfoLines[16] := Format('Adaptive: W = %.3f °/с; min = %.3f м',
    [Frame.AdaptiveController.Output,Frame.AdaptiveMinimumDistance]);
  InfoLines[17] := Format('Adaptive: поверхность = %.4f; невязка = %.4f',
    [Frame.AdaptiveController.SlidingSurface,Frame.AdaptiveController.Residual]);
  InfoLines[18] := Format('Adaptive: theta = %.2f; %.2f; %.2f; %.2f',
    [Frame.AdaptiveController.Theta[0],Frame.AdaptiveController.Theta[1],
     Frame.AdaptiveController.Theta[2],Frame.AdaptiveController.Theta[3]]);
  InfoLines[19] := Format('СВП+: r = %.2f; r зад = %.2f °/с; β = %.2f°; оценка = %.2f°',
    [Frame.GatePlus.Rate, Frame.GatePlus.Controller.CommandRate,
     RadToDeg(ArcTan2(Sin(DegToRad(Frame.GatePlus.Heading - Frame.GatePlus.Course)),
       Cos(DegToRad(Frame.GatePlus.Heading - Frame.GatePlus.Course)))), Frame.GatePlus.Controller.EstimatedSlip]);
  InfoLines[20] := Format('СВП+: min = %.3f м; скорость ЛВ = %.2f °/с; остаточный поворот = %.2f°',
    [Frame.GatePlus.MinimumDistance, Frame.GatePlus.Controller.LOSRate, Frame.GatePlus.Controller.PredictedCourseChange]);
  if Frame.GatePlusResult.Finished then InfoLines[21] := Format('СВП+: завершено; дистанция остановки %.3f м', [Frame.GatePlusResult.Distance])
  else if Frame.GatePlus.Controller.PassageConfirmed then
    InfoLines[21] := 'СВП+: ближайшее сближение подтверждено стендом'
  else if Frame.GatePlus.Controller.PredictionExpired and Frame.GatePlus.Controller.TargetLost then
    InfoLines[21] := 'СВП+: цель недоступна; ограниченное гашение скольжения'
  else if Frame.GatePlus.Controller.Mode = gpmPassage then
    InfoLines[21] := 'СВП+: прохождение по короткому прогнозу (без подтверждения)'
  else if Frame.GatePlus.Controller.TargetLost then
    InfoLines[21] := 'СВП+: цель потеряна; короткий прогноз'
  else if Frame.GatePlus.Controller.Mode = gpmTerminal then
    InfoLines[21] := 'СВП+: терминальное наведение (ближняя зона стенда)'
  else InfoLines[21] := 'СВП+: наведение по углу и гироскопу';
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

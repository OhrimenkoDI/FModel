unit uMotionModel;

interface

uses uRecordedPlant;

const
  FAngleCount = 3; // Задержка по умолчанию, число измерений.
  MaxSensorDelaySamples = 1000;
  DefaultSensorBiasDegrees = -20.0; // Sensor simulation only.
  ReferencePointSpeedKmh = 50.0; // Ноль задаёт неподвижную точку.
  ReferencePointHeadingDegrees = -10.0; // От +X против часовой стрелки.

type
  TReferencePoint = record
    X, Y: Double;
    InitialX, InitialY: Double; // Начальные координаты, м.
    SpeedKmh, HeadingDegrees: Double; // Начальные параметры движения.
    Fi, V: Double; // Постоянные направление (рад) и скорость (м/с).
    Diameter: Double; // Metres.
    procedure Init;
    procedure ResetMotion;
    procedure Integrate(dt: Double);
  end;

  TMotionModel = record
  private
    FAngleHistory: array[0..MaxSensorDelaySamples - 1] of Double;
    FAngleNext: Integer;

  public
    RecordedPlant: TRecordedPlant;
    Course: Double; // Degrees, used only by recorded profile.
    X, Y: Double; // Position in metres.
    Diameter: Double; // Metres; preserved by ResetMotion.
    SpeedKmh, InitialHeadingDegrees: Double;
    Fi: Double;   // Heading in radians, counterclockwise from +X towards +Y.
    V: Double;    // Speed in metres per second.
    W: Double;    // Angular velocity in radians per second.
    MeasuredAngle: Double; // Последнее измерение датчика, рад.
    AngleCount: Integer; // Настраиваемая задержка в измерениях; ноль — без задержки.
    SensorBiasDegrees: Double; // Ошибка датчика принадлежит только модели.
    ReferencePoint: TReferencePoint;
    procedure Init;
    procedure ResetMotion;
    function MeasureAngle: Double;
    function GyroDegrees: Double;
    procedure ApplyControl(CommandDegrees, Dt: Double);
    procedure Integrate(dt: Double);
    function TryAngleToPoint(ReferencePointX, ReferencePointY: Double; out Degrees: Double): Boolean;
  end;

const
  DefaultSpeedKmh = 150.0;
  KmhToMetresPerSecond = 1.0 / 3.6;
  deg = 3.141592/180.0;

implementation

uses
  System.Math, System.SysUtils;

// Задаёт начальные координаты, диаметр и движение опорной точки.
procedure TReferencePoint.Init;
begin
  Diameter := 1;
  InitialX := 100;
  InitialY := 100;
  SpeedKmh := ReferencePointSpeedKmh;
  HeadingDegrees := ReferencePointHeadingDegrees;
  ResetMotion;
end;

// Возвращает точку в начальное положение и восстанавливает заданные скорость и угол.
procedure TReferencePoint.ResetMotion;
begin
  X := InitialX;
  Y := InitialY;
  Fi := DegToRad(HeadingDegrees);
  V := SpeedKmh * KmhToMetresPerSecond;
end;

// Перемещает точку прямолинейно с неизменными скоростью и направлением.
procedure TReferencePoint.Integrate(dt: Double);
begin
  if IsNan(dt) or IsInfinite(dt) or (dt < 0) then
    raise EArgumentException.Create('dt must be finite and non-negative');
  if dt = 0 then Exit;
  X := X + V * Cos(Fi) * dt;
  Y := Y + V * Sin(Fi) * dt;
end;

// Инициализирует модель, опорную точку и параметры датчика.
procedure TMotionModel.Init;
begin
  RecordedPlant.Parameters := DefaultRecordedPlantParameters;
  Diameter := 1;
  ReferencePoint.Init;
  SpeedKmh := DefaultSpeedKmh;
  InitialHeadingDegrees := 0;
  SensorBiasDegrees := DefaultSensorBiasDegrees;
  AngleCount := FAngleCount;
  ResetMotion;
end;

// Сбрасывает движение и буфер датчика, сохраняя настройки модели.
procedure TMotionModel.ResetMotion;
begin
  RecordedPlant.Reset;
  Course := InitialHeadingDegrees;
  X := 0;
  Y := 0;
  Fi := DegToRad(InitialHeadingDegrees);
  W := 0;
  MeasuredAngle := 0;
  FillChar(FAngleHistory, SizeOf(FAngleHistory), 0);
  FAngleNext := 0;
  V := SpeedKmh * KmhToMetresPerSecond;
  ReferencePoint.ResetMotion;
end;

// Вычисляет знаковый угол от направления движения к заданной точке.
function TMotionModel.TryAngleToPoint(ReferencePointX, ReferencePointY: Double;
  out Degrees: Double): Boolean;
var
  DX, DY, HeadingX, HeadingY: Double;
begin
  Degrees := 0;
  DX := ReferencePointX - X;
  DY := ReferencePointY - Y;
  Result := (V <> 0) and ((DX <> 0) or (DY <> 0));
  if not Result then
    Exit;
  HeadingX := Cos(Fi);
  HeadingY := Sin(Fi);
  if V < 0 then
  begin
    HeadingX := -HeadingX;
    HeadingY := -HeadingY;
  end;
  // Signed angle from velocity to the line of sight: positive towards +Y.
  Degrees := RadToDeg(ArcTan2(HeadingX * DY - HeadingY * DX,
    HeadingX * DX + HeadingY * DY));
end;

// Приводит угол в радианах к диапазону от минус пи до плюс пи.
function WrapAngle(Angle: Double): Double;
begin
  Result := ArcTan2(Sin(Angle), Cos(Angle));
end;

// Возвращает задержанный угол и записывает новое измерение с ошибкой в буфер.
function TMotionModel.MeasureAngle: Double;
var AngleDegrees: Double;
begin
  if (AngleCount < 0) or (AngleCount > MaxSensorDelaySamples) then
    raise EArgumentOutOfRangeException.Create('Sensor delay sample count');
  TryAngleToPoint(ReferencePoint.X, ReferencePoint.Y, AngleDegrees);
  if RecordedPlant.Enabled then
  begin
    MeasuredAngle := DegToRad(RecordedPlant.Observe(AngleDegrees));
    Exit(MeasuredAngle);
  end;
  if AngleCount = 0 then
  begin
    MeasuredAngle := WrapAngle(DegToRad(AngleDegrees + SensorBiasDegrees));
    Exit(MeasuredAngle);
  end;
  if FAngleNext >= AngleCount then FAngleNext := 0;
  MeasuredAngle := FAngleHistory[FAngleNext];
  FAngleHistory[FAngleNext] := WrapAngle(DegToRad(AngleDegrees + SensorBiasDegrees));
  Inc(FAngleNext);
  if FAngleNext = AngleCount then FAngleNext := 0;
  Result := MeasuredAngle;
end;

function TMotionModel.GyroDegrees: Double;
begin
  if RecordedPlant.Enabled then Result := RecordedPlant.GyroRate
  else Result := RadToDeg(W);
end;

procedure TMotionModel.ApplyControl(CommandDegrees, Dt: Double);
begin
  if Dt = 0 then Exit;
  if RecordedPlant.Enabled then W := DegToRad(RecordedPlant.Advance(CommandDegrees, Dt))
  else W := DegToRad(CommandDegrees);
  Integrate(Dt);
end;

// Перемещает обе точки за один dt; W управляет только основной моделью.
procedure TMotionModel.Integrate(dt: Double);
begin
  if IsNan(dt) or IsInfinite(dt) or (dt < 0) then
    raise EArgumentException.Create('dt must be finite and non-negative');
  if dt = 0 then Exit;
  Fi := Fi + dt * W;
  if RecordedPlant.Enabled then
  begin
    Course := RecordedPlant.AdvanceCourse(RadToDeg(Fi), Course, dt);
    X := X + V * Cos(DegToRad(Course)) * dt;
    Y := Y + V * Sin(DegToRad(Course)) * dt;
  end
  else
  begin
    X := X + V * Cos(Fi-10*deg) * dt;
    Y := Y + V * Sin(Fi-10*deg) * dt;
  end;
  ReferencePoint.Integrate(dt);
end;

end.
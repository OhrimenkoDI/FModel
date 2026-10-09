unit uMotionModel;

interface

const
  FAngleCount = 3;
  DefaultHeadingOffsetDegrees = 10.0; // Sensor simulation only.

type
  TStaticModel = record
    X, Y: Double;
    Diameter: Double; // Metres.
    procedure Init;
  end;

  TMotionModel = record
  private
    FAngleHistory: array[0..FAngleCount - 1] of Double;
    FAngleNext: Integer;

  public
    X, Y: Double; // Position in metres.
    Diameter: Double; // Metres; preserved by ResetMotion.
    Fi: Double;   // Heading in radians, counterclockwise from +X towards +Y.
    V: Double;    // Speed in metres per second.
    W: Double;    // Angular velocity in radians per second.
    MeasuredAngle: Double; // Последнее измерение датчика, рад.
    HeadingOffsetDegrees: Double; // Ошибка датчика принадлежит только модели.
    Target: TStaticModel;
    procedure Init;
    procedure ResetMotion;
    function MeasureAngle: Double;
    procedure Integrate(dt: Double);
    function TryAngleToPoint(TargetX, TargetY: Double; out Degrees: Double): Boolean;
  end;

const
  DefaultSpeedKmh = 150.0;
  KmhToMetresPerSecond = 1.0 / 3.6;

implementation

uses
  System.Math, System.SysUtils;

// Задаёт начальные координаты и диаметр неподвижной цели.
procedure TStaticModel.Init;
begin
  Diameter := 1;
  X := 100;
  Y := 100;
end;

// Инициализирует модель, цель и параметры датчика.
procedure TMotionModel.Init;
begin
  Diameter := 1;
  Target.Init;
  HeadingOffsetDegrees := DefaultHeadingOffsetDegrees;
  ResetMotion;
end;

// Сбрасывает движение и буфер датчика, сохраняя настройки модели.
procedure TMotionModel.ResetMotion;
begin
  X := 0;
  Y := 0;
  Fi := 0;
  W := 0;
  MeasuredAngle := 0;
  FillChar(FAngleHistory, SizeOf(FAngleHistory), 0);
  FAngleNext := 0;
  V := DefaultSpeedKmh * KmhToMetresPerSecond;
end;

// Вычисляет знаковый угол от направления движения к заданной точке.
function TMotionModel.TryAngleToPoint(TargetX, TargetY: Double;
  out Degrees: Double): Boolean;
var
  DX, DY, HeadingX, HeadingY: Double;
begin
  Degrees := 0;
  DX := TargetX - X;
  DY := TargetY - Y;
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
  MeasuredAngle := FAngleHistory[FAngleNext];
  TryAngleToPoint(Target.X, Target.Y, AngleDegrees);
  FAngleHistory[FAngleNext] := WrapAngle(DegToRad(AngleDegrees + HeadingOffsetDegrees));
  Inc(FAngleNext);
  if FAngleNext = FAngleCount then FAngleNext := 0;
  Result := MeasuredAngle;
end;

// Перемещает модель за dt с заданной извне угловой скоростью W.
procedure TMotionModel.Integrate(dt: Double);
begin
  if IsNan(dt) or IsInfinite(dt) or (dt < 0) then
    raise EArgumentException.Create('dt must be finite and non-negative');
  if dt = 0 then Exit;
  Fi := Fi + dt * W;
  X := X + V * Cos(Fi) * dt;
  Y := Y + V * Sin(Fi) * dt;
end;

end.
unit uPID;

interface

const
  Kp = 3.87;
  Ki = 22.88;
  Kd = 0.10;
  IntegralLimit = 20.0; // Максимальный вклад I, градусы/с.
  // Подстройка смещения после непрерывного удержания цели в пределах порога.
  BiasAdaptationGain = 0.6;
  BiasAngularSpeedWeight = 2.0;
  BiasAdaptationStart = 0.2; // Выдержка захвата, с.
  BiasFilterTime = 0.15;
  BiasErrorGate = 1.0;
  BiasRateLimit = 20.0;
  BiasLimit = 45.0;
  DerivativeFilterTime = 0.0; // Не используется: D берётся по CurrentW.
  BiasGyroDeadband = 0.1; // Допуск нулевого вращения, градусы/с.

type
  TPIDParameters = record
    Kp, Ki, Kd, IntegralLimit: Double;
    // BiasAngularSpeedWeight и DerivativeFilterTime оставлены для совместимости.
    BiasAdaptationGain, BiasAngularSpeedWeight, BiasAdaptationStart,
      BiasFilterTime: Double;
    BiasErrorGate, BiasRateLimit, BiasLimit, DerivativeFilterTime: Double;
  end;

  TPIDState = record
  private
    FPreviousMeasuredAngle, FPreviousDt: Double;
    FHasMeasuredAngle: Boolean;
    FCaptureTime: Double;
  public
    Parameters: TPIDParameters;
    // Оценка смещения по вращению после захвата неподвижной цели.
    AngleCorrectionEnabled: Boolean;
    BiasAdaptationActive: Boolean;
    BiasGyroRate: Double; // Сглаженная скорость для оценки смещения, градусы/с.
    AdaptiveAngleCorrection, CorrectedAngle: Double; // Градусы.
    W, Wprop, Wint, Wdiff: Double; // Градусы/с (номинальная команда для EDF).
    InputW: Double; // Измеренная угловая скорость, градусы/с.
    MeasuredAngleRate: Double; // Скорость изменения измеренного угла, градусы/с.
    HasMeasuredAngleRate: Boolean; // False на первом шаге и после сброса.
  end;

function PIDDefaultParameters: TPIDParameters;
procedure PIDinit(var State: TPIDState);
procedure PIDreset(var State: TPIDState);
function PIDupdate(var State: TPIDState;
  dt, MeasuredAngle, CurrentW: Double): Double;

implementation

uses System.Math, System.SysUtils;

function Wrap(A: Double): Double;
begin
  Result := A - Int(A / 360) * 360;
  if Result > 180 then Result := Result - 360
  else if Result < -180 then Result := Result + 360;
end;

function PIDDefaultParameters: TPIDParameters;
begin
  Result := Default(TPIDParameters);
  Result.Kp := uPID.Kp;
  Result.Ki := uPID.Ki;
  Result.Kd := uPID.Kd;
  Result.IntegralLimit := uPID.IntegralLimit;
  Result.BiasAdaptationGain := uPID.BiasAdaptationGain;
  Result.BiasAngularSpeedWeight := uPID.BiasAngularSpeedWeight;
  Result.BiasAdaptationStart := uPID.BiasAdaptationStart;
  Result.BiasFilterTime := uPID.BiasFilterTime;
  Result.BiasErrorGate := uPID.BiasErrorGate;
  Result.BiasRateLimit := uPID.BiasRateLimit;
  Result.BiasLimit := uPID.BiasLimit;
  Result.DerivativeFilterTime := uPID.DerivativeFilterTime;
end;

procedure PIDinit(var State: TPIDState);
begin
  State := Default(TPIDState);
  State.Parameters := PIDDefaultParameters;
  State.AngleCorrectionEnabled := True;
end;

// Сбрасывает интегратор и показания, сохраняя параметры одной записи.
procedure PIDreset(var State: TPIDState);
var P: TPIDParameters; Enabled: Boolean;
begin
  P := State.Parameters;
  Enabled := State.AngleCorrectionEnabled;
  State := Default(TPIDState);
  State.Parameters := P;
  State.AngleCorrectionEnabled := Enabled;
end;

// PID получает угол за вычетом оценки смещения. D = -Kd*CurrentW.
// Оценка использует только угол, гироскоп и dt; истинное смещение модели недоступно.
// Условие применимости: неподвижная цель, движение вперёд без бокового сноса.
function PIDupdate(var State: TPIDState;
  dt, MeasuredAngle, CurrentW: Double): Double;
var Angle, Error, Alpha, BiasRate: Double;
begin
  if IsNan(dt) or IsInfinite(dt) or (dt <= 0) or
    IsNan(MeasuredAngle) or IsInfinite(MeasuredAngle) or
    IsNan(CurrentW) or IsInfinite(CurrentW) then
    raise EArgumentException.Create('Finite angle, angular speed and positive dt required');

  Angle := Wrap(MeasuredAngle);
  // Производная только исходного измерения, не поправки; пока для диагностики.
  State.HasMeasuredAngleRate := State.FHasMeasuredAngle;
  if State.HasMeasuredAngleRate then
    State.MeasuredAngleRate := Wrap(Angle - State.FPreviousMeasuredAngle) / State.FPreviousDt
  else State.MeasuredAngleRate := 0;

  State.InputW := CurrentW;
  State.BiasAdaptationActive := False;
  if State.Parameters.BiasFilterTime = 0 then Alpha := 1
  else Alpha := 1 - Exp(-dt / State.Parameters.BiasFilterTime);
  if not State.FHasMeasuredAngle then State.BiasGyroRate := CurrentW
  else State.BiasGyroRate := State.BiasGyroRate + Alpha*(CurrentW - State.BiasGyroRate);

  Error := Angle;
  if State.AngleCorrectionEnabled then
  begin
    Error := Wrap(Angle - State.AdaptiveAngleCorrection);
    if State.FHasMeasuredAngle and (Abs(Error) < State.Parameters.BiasErrorGate) then
      State.FCaptureTime := State.FCaptureTime + dt
    else State.FCaptureTime := 0;

    // Вне захвата или около нулевой скорости оценка сохраняется, не обнуляется.
    if State.FHasMeasuredAngle and (Abs(Error) < State.Parameters.BiasErrorGate) and
      (State.FCaptureTime >= State.Parameters.BiasAdaptationStart) and
      (Abs(State.BiasGyroRate) > BiasGyroDeadband) and
      (State.Parameters.BiasAdaptationGain > 0) then
    begin
      BiasRate := EnsureRange(-State.Parameters.BiasAdaptationGain*State.BiasGyroRate,
        -State.Parameters.BiasRateLimit, State.Parameters.BiasRateLimit);
      State.AdaptiveAngleCorrection := EnsureRange(State.AdaptiveAngleCorrection + BiasRate*dt,
        -State.Parameters.BiasLimit, State.Parameters.BiasLimit);
      State.BiasAdaptationActive := True;
    end;
    Error := Wrap(Angle - State.AdaptiveAngleCorrection);
  end
  else State.FCaptureTime := 0;

  State.CorrectedAngle := Error;
  State.Wprop := State.Parameters.Kp * Error;
  if State.Parameters.Ki = 0 then State.Wint := 0
  else State.Wint := EnsureRange(State.Wint + State.Parameters.Ki * Error * dt,
    -State.Parameters.IntegralLimit, State.Parameters.IntegralLimit);
  State.Wdiff := -State.Parameters.Kd * CurrentW;
  State.W := State.Wprop + State.Wint + State.Wdiff;

  State.FPreviousMeasuredAngle := Angle;
  State.FPreviousDt := dt;
  State.FHasMeasuredAngle := True;
  Result := State.W;
end;

end.
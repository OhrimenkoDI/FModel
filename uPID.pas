unit uPID;
interface
const
  Kp = 6.0;
  Ki = 10.0;
  Kd = 0.500;
  IntegralLimit = 60.0; // °/с, эквивалент прежних 10 рад/с.
  BiasAdaptationGain = 0.15;
  BiasAngularSpeedWeight = 2.0;
  BiasAdaptationStart = 0.8; // Время на начальный разворот, с.
  BiasFilterTime = 0.15;
  BiasErrorGate = 15.0;
  BiasRateLimit = 20.0; // Максимальная скорость поправки, °/с.
  BiasLimit = 45.0;
  DerivativeFilterTime = 0.02;
type
  TPIDParameters = record
    Kp, Ki, Kd, IntegralLimit: Double;
    BiasAdaptationGain, BiasAngularSpeedWeight, BiasAdaptationStart, BiasFilterTime: Double;
    BiasErrorGate, BiasRateLimit, BiasLimit, DerivativeFilterTime: Double;
  end;

  TPIDState = record
  private
    FPreviousError, FPreviousDt: Double;
    FTime, FFilteredIntegral, FFilteredW: Double;
    FHasAngle, FPreviousAngleCorrection: Boolean;
  public
    Parameters: TPIDParameters;
    AngleCorrectionEnabled: Boolean;
    AdaptiveAngleCorrection, CorrectedAngle: Double; // Градусы.
    W, Wprop, Wint, Wdiff: Double; // Градусы в секунду.
    InputW: Double; // Полученная текущая угловая скорость, °/с.
  end;
// Для каждого датчика хранится отдельная запись TPIDState. Углы — градусы, скорости — °/с.
function PIDDefaultParameters: TPIDParameters;
procedure PIDinit(var State: TPIDState);
procedure PIDreset(var State: TPIDState);
function PIDupdate(var State: TPIDState; dt, MeasuredAngle, CurrentW: Double): Double;

implementation
uses System.Math, System.SysUtils;
// Нормализует угол в градусах в пределах от -180 до 180.
function Wrap(A: Double): Double;
begin
  Result:=A-Int(A/360)*360;
  if Result>180 then Result:=Result-360
  else if Result< -180 then Result:=Result+360;
end;
// Заполняет параметры регулятора значениями по умолчанию.
function PIDDefaultParameters: TPIDParameters;
begin
  Result.Kp:=uPID.Kp;
  Result.Ki:=uPID.Ki;
  Result.Kd:=uPID.Kd;
  Result.IntegralLimit:=uPID.IntegralLimit;
  Result.BiasAdaptationGain:=uPID.BiasAdaptationGain;
  Result.BiasAngularSpeedWeight:=uPID.BiasAngularSpeedWeight;
  Result.BiasAdaptationStart:=uPID.BiasAdaptationStart;
  Result.BiasFilterTime:=uPID.BiasFilterTime;
  Result.BiasErrorGate:=uPID.BiasErrorGate;
  Result.BiasRateLimit:=uPID.BiasRateLimit;
  Result.BiasLimit:=uPID.BiasLimit;
  Result.DerivativeFilterTime:=uPID.DerivativeFilterTime;
end;

// Обнуляет состояние регулятора и задаёт начальные настройки.
procedure PIDinit(var State: TPIDState);
begin
  State:=Default(TPIDState);
  State.Parameters:=PIDDefaultParameters;
  State.AngleCorrectionEnabled:=True;
  State.FPreviousAngleCorrection:=True;
end;

// Сбрасывает интеграторы, поправку, фильтры и историю одного датчика, сохраняя настройки.
procedure PIDreset(var State: TPIDState);
var SavedParameters: TPIDParameters; Enabled: Boolean;
begin
  SavedParameters:=State.Parameters;
  Enabled:=State.AngleCorrectionEnabled;
  State:=Default(TPIDState);
  State.Parameters:=SavedParameters;
  State.AngleCorrectionEnabled:=Enabled;
  State.FPreviousAngleCorrection:=Enabled;
end;

// Возвращает квадратный корень модуля с сохранением знака.
function sqrtp(p:double):double;
begin

  if p<0 then
    result:=-sqrt(-p)
  else result:=sqrt(p);
end;
// Адаптирует поправку по Wint и текущей W, затем возвращает сумму P, I и D.
// dt — текущий шаг, с; MeasuredAngle — полученный угол к опорной точке, градусы;
// CurrentW — угловая скорость до нового воздействия, °/с.
// Результат — рассчитанная угловая скорость объекта, °/с.
// Входные данные: измеренный угол, длительность шага и текущая угловая скорость объекта.
function PIDupdate(var State: TPIDState; dt, MeasuredAngle, CurrentW: Double): Double;
var First: Boolean; Alpha, BiasRate, RawDerivative: Double;
begin
  // Некорректный вход отклоняем до изменения внутреннего состояния.
  if IsNan(Dt) or IsInfinite(Dt) or (Dt<=0) or
    IsNan(MeasuredAngle) or IsInfinite(MeasuredAngle) or
    IsNan(CurrentW) or IsInfinite(CurrentW) then
    raise EArgumentException.Create('Finite angle, angular speed and positive dt required');

  // На первом шаге нет предыдущего угла для производной.
  // При переключении коррекции также сбрасываем D, чтобы избежать её скачка.
  First:=not State.FHasAngle or (State.AngleCorrectionEnabled<>State.FPreviousAngleCorrection);


  // Сохраняем входную скорость для отображения и просмотра истории.
  State.InputW:=CurrentW;

  // Сглаживаем State.Wint прошлого шага и текущую скорость для медленной адаптации.
  // Alpha учитывает dt; State.Parameters.BiasFilterTime задаёт постоянную времени фильтра.
  if State.Parameters.BiasFilterTime = 0 then Alpha:=1
  else Alpha:=1-Exp(-dt/State.Parameters.BiasFilterTime);
  if State.Parameters.Ki = 0 then begin State.Wint:=0; State.FFilteredIntegral:=0; end;
  State.FFilteredIntegral:=State.FFilteredIntegral+Alpha*(State.Wint-State.FFilteredIntegral);
  State.FFilteredW:=State.FFilteredW+Alpha*(CurrentW-State.FFilteredW);

  // Поправку меняем только при включённой коррекции, после начального разворота
  // и при небольшом исправленном угле прошлого шага. Иначе сохраняем её значение.
  if State.AngleCorrectionEnabled and State.FHasAngle and (State.FTime>=State.Parameters.BiasAdaptationStart) and
    (Abs(State.FPreviousError)<State.Parameters.BiasErrorGate) then
  begin
    // Подстраиваем поправку против взвешенной суммы сглаженных State.Wint и State.W.
    // BiasRate — скорость изменения поправки, ограниченная в °/с.
    // Адаптация использует сумму двух сигналов; взаимная компенсация возможна при ненулевых слагаемых.
    BiasRate:=EnsureRange(-State.Parameters.BiasAdaptationGain*
      (State.FFilteredIntegral+State.Parameters.BiasAngularSpeedWeight*State.FFilteredW),-State.Parameters.BiasRateLimit,State.Parameters.BiasRateLimit);

    // Интегрируем скорость поправки по времени и ограничиваем сам угол.
    // State.AdaptiveAngleCorrection — экспериментальная поправка угла по состоянию обратной связи.
    State.AdaptiveAngleCorrection:=EnsureRange(State.AdaptiveAngleCorrection+BiasRate*dt,-State.Parameters.BiasLimit,State.Parameters.BiasLimit);
  end;

  // Вычитаем поправку только при включённой коррекции.
  // Wrap возвращает угол в диапазон [-180; 180]. Задержку здесь не компенсируем.
  State.CorrectedAngle:=MeasuredAngle;
  if State.AngleCorrectionEnabled then State.CorrectedAngle:=State.CorrectedAngle-State.AdaptiveAngleCorrection;
  State.CorrectedAngle:=Wrap(State.CorrectedAngle);

  // I: накапливаем исправленную ошибку. Ограничиваем вклад в выход, °/с.
  // При нулевой ошибке накопленное значение интегратора сохраняется.
  State.Wint:=EnsureRange(State.Wint+State.Parameters.Ki*State.CorrectedAngle*Dt,-State.Parameters.IntegralLimit,State.Parameters.IntegralLimit);

  // D: оцениваем скорость изменения исправленного угла, включая изменение поправки.
  if First or (State.Parameters.Kd = 0) then State.Wdiff:=0
  else
  begin
    // Измерения поступают перед движением: их разделяет предыдущий шаг времени.
    // Wrap исключает ложный скачок на полный оборот при переходе через +/-180°.
    RawDerivative:=State.Parameters.Kd*Wrap(State.CorrectedAngle-State.FPreviousError)/State.FPreviousDt;

    // Сглаживаем производную, уменьшая резкие реакции на задержанные измерения.
    if State.Parameters.DerivativeFilterTime = 0 then State.Wdiff:=RawDerivative
    else State.Wdiff:=State.Wdiff+(1-Exp(-dt/State.Parameters.DerivativeFilterTime))*(RawDerivative-State.Wdiff);
  end;

  // P: немедленная реакция, пропорциональная исправленному углу.
  State.Wprop:=State.Parameters.Kp*State.CorrectedAngle;

  // Складываем три вклада в °/с. Общего ограничения выхода State.W сейчас нет.
  State.W:=State.Wprop+State.Wdiff +State.Wint;

  // Запоминаем состояние для следующего вызова и продвигаем внутренние часы.
  State.FPreviousError:=State.CorrectedAngle;
  State.FHasAngle:=True;
  State.FPreviousAngleCorrection:=State.AngleCorrectionEnabled;
  State.FPreviousDt:=Dt;
  State.FTime:=State.FTime+dt;
  Result:=State.W;
end;
end.

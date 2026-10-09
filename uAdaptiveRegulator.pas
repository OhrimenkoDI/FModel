unit uAdaptiveRegulator;
interface
const
  Kp = 6.0;
  Ki = 10.0;
  Kd = 0.500;
  IntegralLimit = 10.0;
  BiasAdaptationGain = 0.15;
  BiasAngularSpeedWeight = 2.0;
  BiasAdaptationStart = 0.8; // Время на начальный разворот, с.
  BiasFilterTime = 0.15;
  BiasErrorGate = 15.0 * Pi / 180;
  BiasRateLimit = 20.0 * Pi / 180; // Максимальная скорость поправки, рад/с.
  BiasLimit = 45.0 * Pi / 180;
  DerivativeFilterTime = 0.02;
type
  TRegulatorParameters = record
    Kp, Ki, Kd, IntegralLimit: Double;
    BiasAdaptationGain, BiasAngularSpeedWeight, BiasAdaptationStart, BiasFilterTime: Double;
    BiasErrorGate, BiasRateLimit, BiasLimit, DerivativeFilterTime: Double;
    procedure Init;
  end;

  TAdaptiveRegulator = record
  private
    FPreviousError, FPreviousDt: Double;
    FTime, FFilteredIntegral, FFilteredW: Double;
    FHasAngle, FPreviousAngleCorrection: Boolean;
  public
    Parameters: TRegulatorParameters;
    AngleCorrectionEnabled: Boolean;
    AdaptiveAngleCorrection, CorrectedAngle: Double;
    W, Wprop, Wint, Wdiff: Double;
    InputW: Double; // Полученная текущая угловая скорость, рад/с.
    procedure Init;
    procedure Reset;
    function Update(dt, MeasuredAngle, CurrentW: Double): Double;
  end;
implementation
uses System.Math, System.SysUtils;
// Нормализует угол в радианах в пределах от -Pi до Pi.
function Wrap(A: Double): Double;
begin
  Result:=ArcTan2(Sin(A),Cos(A));
end;
// Заполняет параметры регулятора значениями по умолчанию.
procedure TRegulatorParameters.Init;
begin
  Self.Kp:=uAdaptiveRegulator.Kp;
  Self.Ki:=uAdaptiveRegulator.Ki;
  Self.Kd:=uAdaptiveRegulator.Kd;
  Self.IntegralLimit:=uAdaptiveRegulator.IntegralLimit;
  Self.BiasAdaptationGain:=uAdaptiveRegulator.BiasAdaptationGain;
  Self.BiasAngularSpeedWeight:=uAdaptiveRegulator.BiasAngularSpeedWeight;
  Self.BiasAdaptationStart:=uAdaptiveRegulator.BiasAdaptationStart;
  Self.BiasFilterTime:=uAdaptiveRegulator.BiasFilterTime;
  Self.BiasErrorGate:=uAdaptiveRegulator.BiasErrorGate;
  Self.BiasRateLimit:=uAdaptiveRegulator.BiasRateLimit;
  Self.BiasLimit:=uAdaptiveRegulator.BiasLimit;
  Self.DerivativeFilterTime:=uAdaptiveRegulator.DerivativeFilterTime;
end;

// Обнуляет состояние регулятора и задаёт начальные настройки.
procedure TAdaptiveRegulator.Init;
begin
  Self:=Default(TAdaptiveRegulator);
  Parameters.Init;
  AngleCorrectionEnabled:=True;
  FPreviousAngleCorrection:=True;
end;
// Обнуляет накопленные величины, сохраняя параметры и состояние коррекции.
procedure TAdaptiveRegulator.Reset;
var SavedParameters: TRegulatorParameters; Enabled: Boolean;
begin
  SavedParameters:=Parameters;
  Enabled:=AngleCorrectionEnabled;
  Init;
  Parameters:=SavedParameters;
  AngleCorrectionEnabled:=Enabled;
  FPreviousAngleCorrection:=Enabled;
end;

// Адаптирует поправку по Wint и текущей W, затем возвращает сумму P, I и D.
// dt — текущий шаг, с; MeasuredAngle — полученный угол к опорной точке, рад;
// CurrentW — угловая скорость до нового воздействия, рад/с.
// Результат — рассчитанная угловая скорость объекта, рад/с.
// Входные данные: измеренный угол, длительность шага и текущая угловая скорость объекта.
function TAdaptiveRegulator.Update(dt, MeasuredAngle, CurrentW: Double): Double;
var First: Boolean; Alpha, BiasRate, RawDerivative: Double;
begin
  // Некорректный вход отклоняем до изменения внутреннего состояния.
  if IsNan(Dt) or IsInfinite(Dt) or (Dt<=0) or
    IsNan(MeasuredAngle) or IsInfinite(MeasuredAngle) or
    IsNan(CurrentW) or IsInfinite(CurrentW) then
    raise EArgumentException.Create('Finite angle, angular speed and positive dt required');

  // На первом шаге нет предыдущего угла для производной.
  // При переключении коррекции также сбрасываем D, чтобы избежать её скачка.
  First:=not FHasAngle or (AngleCorrectionEnabled<>FPreviousAngleCorrection);

  // Сохраняем входную скорость для отображения и просмотра истории.
  InputW:=CurrentW;

  // Сглаживаем Wint прошлого шага и текущую скорость для медленной адаптации.
  // Alpha учитывает dt; Parameters.BiasFilterTime задаёт постоянную времени фильтра.
  if Parameters.BiasFilterTime = 0 then Alpha:=1
  else Alpha:=1-Exp(-dt/Parameters.BiasFilterTime);
  if Parameters.Ki = 0 then begin Wint:=0; FFilteredIntegral:=0; end;
  FFilteredIntegral:=FFilteredIntegral+Alpha*(Wint-FFilteredIntegral);
  FFilteredW:=FFilteredW+Alpha*(CurrentW-FFilteredW);

  // Поправку меняем только при включённой коррекции, после начального разворота
  // и при небольшом исправленном угле прошлого шага. Иначе сохраняем её значение.
  if AngleCorrectionEnabled and FHasAngle and (FTime>=Parameters.BiasAdaptationStart) and
    (Abs(FPreviousError)<Parameters.BiasErrorGate) then
  begin
    // Подстраиваем поправку против взвешенной суммы сглаженных Wint и W.
    // BiasRate — скорость изменения поправки, ограниченная в рад/с.
    // Адаптация использует сумму двух сигналов; взаимная компенсация возможна при ненулевых слагаемых.
    BiasRate:=EnsureRange(-Parameters.BiasAdaptationGain*
      (FFilteredIntegral+Parameters.BiasAngularSpeedWeight*FFilteredW),-Parameters.BiasRateLimit,Parameters.BiasRateLimit);

    // Интегрируем скорость поправки по времени и ограничиваем сам угол.
    // AdaptiveAngleCorrection — экспериментальная поправка угла по состоянию обратной связи.
    AdaptiveAngleCorrection:=EnsureRange(AdaptiveAngleCorrection+BiasRate*dt,-Parameters.BiasLimit,Parameters.BiasLimit);
  end;

  // Вычитаем поправку только при включённой коррекции.
  // Wrap возвращает угол в диапазон [-Pi; Pi]. Задержку здесь не компенсируем.
  CorrectedAngle:=MeasuredAngle;
  if AngleCorrectionEnabled then CorrectedAngle:=CorrectedAngle-AdaptiveAngleCorrection;
  CorrectedAngle:=Wrap(CorrectedAngle);

  // I: накапливаем исправленную ошибку. Ограничиваем вклад в выход, рад/с.
  // При нулевой ошибке накопленное значение интегратора сохраняется.
  Wint:=EnsureRange(Wint+Parameters.Ki*CorrectedAngle*Dt,-Parameters.IntegralLimit,Parameters.IntegralLimit);

  // D: оцениваем скорость изменения исправленного угла, включая изменение поправки.
  if First or (Parameters.Kd = 0) then Wdiff:=0
  else
  begin
    // Измерения поступают перед движением: их разделяет предыдущий шаг времени.
    // Wrap исключает ложный скачок на полный оборот при переходе через +/-Pi.
    RawDerivative:=Parameters.Kd*Wrap(CorrectedAngle-FPreviousError)/FPreviousDt;

    // Сглаживаем производную, уменьшая резкие реакции на задержанные измерения.
    if Parameters.DerivativeFilterTime = 0 then Wdiff:=RawDerivative
    else Wdiff:=Wdiff+(1-Exp(-dt/Parameters.DerivativeFilterTime))*(RawDerivative-Wdiff);
  end;

  // P: немедленная реакция, пропорциональная исправленному углу.
  Wprop:=Parameters.Kp*CorrectedAngle;

  // Складываем три вклада в рад/с. Общего ограничения выхода W сейчас нет.
  W:=Wprop+Wint+Wdiff;

  // Запоминаем состояние для следующего вызова и продвигаем внутренние часы.
  FPreviousError:=CorrectedAngle;
  FHasAngle:=True;
  FPreviousAngleCorrection:=AngleCorrectionEnabled;
  FPreviousDt:=Dt;
  FTime:=FTime+dt;
  Result:=W;
end;
end.

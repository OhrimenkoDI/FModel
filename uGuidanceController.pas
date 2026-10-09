unit uGuidanceController;
interface
const
  Kp = 6.0;
  Ki = 10.0;
  Kd = 0.90;
  IntegralLimit = 50.0;
  BiasAdaptationGain = 0.15;
  BiasAngularSpeedWeight = 2.0;
  BiasAdaptationStart = 0.8; // Время на начальный разворот, с.
  BiasFilterTime = 0.15;
  BiasErrorGate = 15.0 * Pi / 180;
  BiasRateLimit = 20.0 * Pi / 180; // Максимальная скорость поправки, рад/с.
  BiasLimit = 45.0 * Pi / 180;
  DerivativeFilterTime = 0.02;
type
  TGuidanceController = record
  private
    FPreviousError, FPreviousDt: Double;
    FTime, FFilteredIntegral, FFilteredW: Double;
    FHasAngle, FPreviousAngleCorrection: Boolean;
  public
    AngleCorrectionEnabled: Boolean;
    BiasEstimate, CorrectedAngle: Double;
    W, Wprop, Wint, Wdiff: Double;
    InputW: Double; // Полученная текущая угловая скорость, рад/с.
    procedure Init;
    function Update(dt, MeasuredAngle, CurrentW: Double): Double;
  end;
implementation
uses System.Math, System.SysUtils;
// Нормализует угол в радианах в пределах от -Pi до Pi.
function Wrap(A: Double): Double;
begin
  Result:=ArcTan2(Sin(A),Cos(A));
end;
// Обнуляет состояние регулятора и задаёт начальные настройки.
procedure TGuidanceController.Init;
begin
  Self:=Default(TGuidanceController);
  AngleCorrectionEnabled:=True;
  FPreviousAngleCorrection:=True;
end;
// Адаптирует поправку по Wint и текущей W, затем возвращает сумму P, I и D.
// dt — текущий шаг, с; MeasuredAngle — полученный угол на цель, рад;
// CurrentW — угловая скорость до нового воздействия, рад/с.
// Результат — новая команда угловой скорости, рад/с.
// Координаты, истинные смещение и задержка датчика здесь неизвестны.
function TGuidanceController.Update(dt, MeasuredAngle, CurrentW: Double): Double;
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
  // Alpha учитывает dt; BiasFilterTime задаёт постоянную времени фильтра.
  Alpha:=1-Exp(-dt/BiasFilterTime);
  FFilteredIntegral:=FFilteredIntegral+Alpha*(Wint-FFilteredIntegral);
  FFilteredW:=FFilteredW+Alpha*(CurrentW-FFilteredW);

  // Поправку меняем только при включённой коррекции, после начального разворота
  // и при небольшом исправленном угле прошлого шага. Иначе сохраняем её значение.
  if AngleCorrectionEnabled and FHasAngle and (FTime>=BiasAdaptationStart) and
    (Abs(FPreviousError)<BiasErrorGate) then
  begin
    // Подстраиваем поправку против взвешенной суммы сглаженных Wint и W.
    // BiasRate — скорость изменения поправки, ограниченная в рад/с.
    // Это эвристика: сигналы могут взаимно компенсироваться, не став нулевыми.
    BiasRate:=EnsureRange(-BiasAdaptationGain*
      (FFilteredIntegral+BiasAngularSpeedWeight*FFilteredW),-BiasRateLimit,BiasRateLimit);

    // Интегрируем скорость поправки по времени и ограничиваем сам угол.
    // BiasEstimate — поправка управления, а не гарантированная ошибка датчика.
    BiasEstimate:=EnsureRange(BiasEstimate+BiasRate*dt,-BiasLimit,BiasLimit);
  end;

  // Вычитаем поправку только при включённой коррекции.
  // Wrap возвращает угол в диапазон [-Pi; Pi]. Задержку здесь не компенсируем.
  CorrectedAngle:=MeasuredAngle;
  if AngleCorrectionEnabled then CorrectedAngle:=CorrectedAngle-BiasEstimate;
  CorrectedAngle:=Wrap(CorrectedAngle);

  // I: накапливаем исправленную ошибку. Ограничиваем вклад в выход, рад/с.
  // При нулевой ошибке накопленное значение интегратора сохраняется.
  Wint:=EnsureRange(Wint+Ki*CorrectedAngle*Dt,-IntegralLimit,IntegralLimit);

  // D: оцениваем скорость изменения исправленного угла, включая изменение поправки.
  if First then Wdiff:=0
  else
  begin
    // Измерения поступают перед движением: их разделяет предыдущий шаг времени.
    // Wrap исключает ложный скачок на полный оборот при переходе через +/-Pi.
    RawDerivative:=Kd*Wrap(CorrectedAngle-FPreviousError)/FPreviousDt;

    // Сглаживаем производную, уменьшая резкие реакции на задержанные измерения.
    Wdiff:=Wdiff+(1-Exp(-dt/DerivativeFilterTime))*(RawDerivative-Wdiff);
  end;

  // P: немедленная реакция, пропорциональная исправленному углу.
  Wprop:=Kp*CorrectedAngle;

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

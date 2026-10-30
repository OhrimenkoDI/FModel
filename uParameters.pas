unit uParameters;

interface

uses System.Classes, System.IniFiles, Vcl.Forms, Vcl.StdCtrls, Vcl.ComCtrls;

const ParameterCount = 88;
type
  TParameterValues = array[0..ParameterCount - 1] of Double;
  TParametersForm = class(TForm)
  private
    FPages: TPageControl;
    FEditors: array[0..ParameterCount - 1] of TEdit;
    procedure DefaultsClick(Sender: TObject);
    procedure AcceptClick(Sender: TObject);
  public
    constructor Create(AOwner: TComponent); override;
    procedure SetValues(const Values: TParameterValues);
    function TryGetValues(out Values: TParameterValues; out InvalidIndex: Integer): Boolean;
  end;

function DefaultParameterValues: TParameterValues;
procedure ReadParameterValues(Ini: TMemIniFile; var Values: TParameterValues);
procedure WriteParameterValues(Ini: TMemIniFile; const Values: TParameterValues);
function EditParameters(var Values: TParameterValues): Boolean;

implementation

uses System.SysUtils, System.Math, System.StrUtils, Vcl.Controls, Vcl.ExtCtrls, Vcl.Dialogs,
  System.UITypes, uMotionModel, uPID;

type
  TParameterDefinition = record
    Section, Key, Caption: string;
    DefaultValue, Minimum, Maximum: Double;
    Page: Integer;
  end;
const
  Definitions: array[0..ParameterCount - 1] of TParameterDefinition = (
    (Section: 'View'; Key: 'PixelsPerMetre'; Caption: 'Масштаб, пкс/м'; DefaultValue: 10; Minimum: 0.1; Maximum: 10000; Page: 0),
    (Section: 'ReferencePoint'; Key: 'X'; Caption: 'Начальная координата X, м'; DefaultValue: 100; Minimum: -1E9; Maximum: 1E9; Page: 0),
    (Section: 'ReferencePoint'; Key: 'Y'; Caption: 'Начальная координата Y, м'; DefaultValue: 100; Minimum: -1E9; Maximum: 1E9; Page: 0),
    (Section: 'Model'; Key: 'Diameter'; Caption: 'Диаметр основной модели, м'; DefaultValue: 1; Minimum: 0.000001; Maximum: 10000; Page: 0),
    (Section: 'ReferencePoint'; Key: 'Diameter'; Caption: 'Диаметр красной точки, м'; DefaultValue: 1; Minimum: 0.000001; Maximum: 10000; Page: 0),
    (Section: 'Sensor'; Key: 'AngleCount'; Caption: 'Задержка, измерений (FAngleCount)'; DefaultValue: FAngleCount; Minimum: 0; Maximum: MaxSensorDelaySamples; Page: 1),
    (Section: 'Sensor'; Key: 'BiasDegrees'; Caption: 'Смещение датчика, °'; DefaultValue: DefaultSensorBiasDegrees; Minimum: -180; Maximum: 180; Page: 1),
    (Section: 'ReferencePoint'; Key: 'SpeedKmh'; Caption: 'Скорость красной точки, км/ч'; DefaultValue: ReferencePointSpeedKmh; Minimum: 0; Maximum: 100000; Page: 1),
    (Section: 'ReferencePoint'; Key: 'HeadingDegrees'; Caption: 'Начальный угол красной точки, °'; DefaultValue: ReferencePointHeadingDegrees; Minimum: -360; Maximum: 360; Page: 1),
    (Section: 'Controller'; Key: 'Kp'; Caption: 'Пропорциональный коэффициент Kp'; DefaultValue: Kp; Minimum: 0; Maximum: 1E6; Page: 2),
    (Section: 'Controller'; Key: 'Ki'; Caption: 'Интегральный коэффициент Ki'; DefaultValue: Ki; Minimum: 0; Maximum: 1E6; Page: 2),
    (Section: 'Controller'; Key: 'Kd'; Caption: 'Дифференциальный коэффициент Kd'; DefaultValue: Kd; Minimum: 0; Maximum: 1E6; Page: 2),
    (Section: 'Controller'; Key: 'IntegralLimitDegreesPerSecond'; Caption: 'Предел I (IntegralLimit), °/с'; DefaultValue: IntegralLimit; Minimum: 0; Maximum: 1E6; Page: 2),
    (Section: 'Controller'; Key: 'BiasAdaptationGain'; Caption: 'Коэффициент адаптации (BiasAdaptationGain)'; DefaultValue: BiasAdaptationGain; Minimum: 0; Maximum: 1E6; Page: 3),
    (Section: 'Controller'; Key: 'BiasAngularSpeedWeight'; Caption: 'Вес W (BiasAngularSpeedWeight)'; DefaultValue: BiasAngularSpeedWeight; Minimum: 0; Maximum: 1E6; Page: 3),
    (Section: 'Controller'; Key: 'BiasAdaptationStart'; Caption: 'Начало адаптации (BiasAdaptationStart), с'; DefaultValue: BiasAdaptationStart; Minimum: 0; Maximum: 1E6; Page: 3),
    (Section: 'Controller'; Key: 'BiasFilterTime'; Caption: 'Время фильтра адаптации (BiasFilterTime), с'; DefaultValue: BiasFilterTime; Minimum: 0; Maximum: 1E6; Page: 3),
    (Section: 'Controller'; Key: 'BiasErrorGateDegrees'; Caption: 'Порог угла (BiasErrorGate), °'; DefaultValue: BiasErrorGate; Minimum: 0; Maximum: 180; Page: 3),
    (Section: 'Controller'; Key: 'BiasRateLimitDegrees'; Caption: 'Скорость поправки (BiasRateLimit), °/с'; DefaultValue: BiasRateLimit; Minimum: 0; Maximum: 1E6; Page: 3),
    (Section: 'Controller'; Key: 'BiasLimitDegrees'; Caption: 'Предел поправки (BiasLimit), °'; DefaultValue: BiasLimit; Minimum: 0; Maximum: 180; Page: 3),
    (Section: 'Controller'; Key: 'DerivativeFilterTime'; Caption: 'Время фильтра D (DerivativeFilterTime), с'; DefaultValue: DerivativeFilterTime; Minimum: 0; Maximum: 1E6; Page: 3),
    (Section: 'Simulation'; Key: 'AutoStopDistance'; Caption: 'Автоостановка при расстоянии меньше, м'; DefaultValue: 5; Minimum: 0; Maximum: 1E9; Page: 0),
    (Section: 'GateNavigation'; Key: 'N'; Caption: 'Навигационная постоянная N'; DefaultValue: 4; Minimum: 0; Maximum: 10; Page: 4),
    (Section: 'GateNavigation'; Key: 'LeadTime'; Caption: 'Компенсация скольжения τ, с'; DefaultValue: 0.4; Minimum: 0; Maximum: 10; Page: 4),
    (Section: 'GateNavigation'; Key: 'KLambda'; Caption: 'Удержание цели Kλ, 1/с (0 = чистая ПН)'; DefaultValue: 0.5; Minimum: 0; Maximum: 10; Page: 4),
    (Section: 'GateNavigation'; Key: 'MaxRate'; Caption: 'Предельная угловая скорость, °/с'; DefaultValue: 90; Minimum: 0.01; Maximum: 720; Page: 4),
    (Section: 'GateNavigation'; Key: 'FilterTime'; Caption: 'Время фильтра линии визирования, с'; DefaultValue: 0.15; Minimum: 0.01; Maximum: 10; Page: 4),
    (Section: 'GateNavigation'; Key: 'RateKp'; Caption: 'ПИ угловой скорости: Kp'; DefaultValue: 4; Minimum: 0; Maximum: 100; Page: 4),
    (Section: 'GateNavigation'; Key: 'RateKi'; Caption: 'ПИ угловой скорости: Ki, 1/с'; DefaultValue: 20; Minimum: 0; Maximum: 1000; Page: 4),
    (Section: 'GateNavigation'; Key: 'ActuatorTime'; Caption: 'Постоянная времени рулевого мотора, с'; DefaultValue: 0.05; Minimum: 0.01; Maximum: 10; Page: 4),
    (Section: 'GateNavigation'; Key: 'SlipTime'; Caption: 'Постоянная времени скольжения модели, с'; DefaultValue: 0.4; Minimum: 0; Maximum: 10; Page: 4),
    (Section: 'Motion'; Key: 'SpeedKmh'; Caption: 'Общая скорость четырёх моделей, км/ч'; DefaultValue: DefaultSpeedKmh; Minimum: 0; Maximum: 1000; Page: 5),
    (Section: 'GateNavigation'; Key: 'FreezeDistance'; Caption: 'Заморозка по дальности стенда, м (0 = выкл.)'; DefaultValue: 5; Minimum: 0; Maximum: 1000; Page: 4),
    (Section: 'GateNavigation'; Key: 'SensorDelay'; Caption: 'Задержка детектора СВП, измерений'; DefaultValue: 3; Minimum: 0; Maximum: 1000; Page: 4),
    (Section: 'GateNavigation'; Key: 'SensorBias'; Caption: 'Смещение детектора СВП, °'; DefaultValue: 0; Minimum: -180; Maximum: 180; Page: 4),
    (Section: 'Motion'; Key: 'InitialHeadingDegrees'; Caption: 'Начальный угол корпуса четырёх моделей, ° от +X'; DefaultValue: 0; Minimum: -360; Maximum: 360; Page: 5),
    (Section: 'AdaptiveGlide'; Key: 'CommandLimit'; Caption: 'Предел исходной команды, рад'; DefaultValue: 0.3; Minimum: 0.001; Maximum: 3; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'SlidingGain'; Caption: 'Коэффициент скользящего регулятора'; DefaultValue: 9.62; Minimum: 0; Maximum: 1000; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'SwitchingGain'; Caption: 'Переключающий коэффициент'; DefaultValue: 0.00475; Minimum: 0; Maximum: 100; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'Kappa'; Caption: 'Параллельный шунт: κ'; DefaultValue: -0.136; Minimum: -100; Maximum: 100; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'Lambda'; Caption: 'Параллельный шунт: λ'; DefaultValue: 1.56; Minimum: 0; Maximum: 100; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'IdentifierAlpha'; Caption: 'Коэффициент забывания идентификатора'; DefaultValue: 0.001; Minimum: 0; Maximum: 50; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'CovarianceScale'; Caption: 'Масштаб начальной ковариации'; DefaultValue: 0.001; Minimum: 0.001; Maximum: 1000000; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'DenominatorGuard'; Caption: 'Минимальный модуль знаменателя'; DefaultValue: 0.35; Minimum: 0.001; Maximum: 100; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'RateGain'; Caption: 'Команда → угловая скорость, 1/с (со знаком)'; DefaultValue: -5; Minimum: -100; Maximum: 100; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'AngleLimit'; Caption: 'Ограничение угла датчика, рад'; DefaultValue: 0.68; Minimum: 0.001; Maximum: 3.14159265358979; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'Reference1'; Caption: 'Фильтр задания: am1'; DefaultValue: 47.74; Minimum: 0.001; Maximum: 100; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'Reference2'; Caption: 'Фильтр задания: am2'; DefaultValue: 576.54; Minimum: 0.001; Maximum: 1000; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'Reference3'; Caption: 'Фильтр задания: am3'; DefaultValue: 3420.84; Minimum: 0.001; Maximum: 10000; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'Identifier1'; Caption: 'Фильтр идентификатора: d1'; DefaultValue: 20; Minimum: 0.001; Maximum: 100; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'Identifier2'; Caption: 'Фильтр идентификатора: d2'; DefaultValue: 200; Minimum: 0.001; Maximum: 1000; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'Identifier3'; Caption: 'Фильтр идентификатора: d3'; DefaultValue: 1000; Minimum: 0.001; Maximum: 10000; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'SensorDelay'; Caption: 'Задержка датчика Adaptive, измерений'; DefaultValue: 3; Minimum: 0; Maximum: 1000; Page: 6),
    (Section: 'AdaptiveGlide'; Key: 'SensorBias'; Caption: 'Смещение датчика Adaptive, °'; DefaultValue: 0; Minimum: -180; Maximum: 180; Page: 6),
    (Section: 'GateNavigationPlus'; Key: 'ObserverSlipTime'; Caption: 'Модельное τ наблюдателя скольжения, с'; DefaultValue: 0.4; Minimum: 0; Maximum: 10; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'LeadFilterTime'; Caption: 'Фильтр упреждения, с'; DefaultValue: 0.15; Minimum: 0.01; Maximum: 10; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'MaxCommandAcceleration'; Caption: 'Резкость задания при наведении, °/с²'; DefaultValue: 360; Minimum: 0.01; Maximum: 10000; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'TerminalCommandAcceleration'; Caption: 'Резкость задания в ближней зоне, °/с²'; DefaultValue: 90; Minimum: 0.01; Maximum: 10000; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'TerminalMaxRate'; Caption: 'Предел задания в ближней зоне, °/с'; DefaultValue: 30; Minimum: 0.01; Maximum: 720; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'PredictionTime'; Caption: 'Горизонт прогноза потери цели, с'; DefaultValue: 0.3; Minimum: 0.01; Maximum: 10; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'SafeRate'; Caption: 'Предел задания после прогноза, °/с'; DefaultValue: 15; Minimum: 0.01; Maximum: 720; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'DetectorHalfAngle'; Caption: 'Полуугол обзора детектора, °'; DefaultValue: 180; Minimum: 1; Maximum: 180; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'InitialSlipDegrees'; Caption: 'Начальное скольжение модели, °'; DefaultValue: 0; Minimum: -90; Maximum: 90; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'N'; Caption: 'Навигационная постоянная N'; DefaultValue: 4; Minimum: 0; Maximum: 10; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'LeadTime'; Caption: 'Компенсация скольжения τ, с'; DefaultValue: 0.4; Minimum: 0; Maximum: 10; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'KLambda'; Caption: 'Удержание цели Kλ, 1/с (0 = чистая ПН)'; DefaultValue: 0.5; Minimum: 0; Maximum: 10; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'MaxRate'; Caption: 'Предельная угловая скорость, °/с'; DefaultValue: 90; Minimum: 0.01; Maximum: 720; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'FilterTime'; Caption: 'Время фильтра линии визирования, с'; DefaultValue: 0.15; Minimum: 0.01; Maximum: 10; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'RateKp'; Caption: 'ПИ угловой скорости: Kp'; DefaultValue: 4; Minimum: 0; Maximum: 100; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'RateKi'; Caption: 'ПИ угловой скорости: Ki, 1/с'; DefaultValue: 20; Minimum: 0; Maximum: 1000; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'ActuatorTime'; Caption: 'Постоянная времени рулевого мотора, с'; DefaultValue: 0.05; Minimum: 0.01; Maximum: 10; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'SlipTime'; Caption: 'Постоянная времени скольжения модели, с'; DefaultValue: 0.4; Minimum: 0; Maximum: 10; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'FreezeDistance'; Caption: 'Ближняя зона стенда, м (0 = выкл.)'; DefaultValue: 5; Minimum: 0; Maximum: 1000; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'SensorDelay'; Caption: 'Задержка детектора СВП+, измерений'; DefaultValue: 3; Minimum: 0; Maximum: 1000; Page: 7),
    (Section: 'GateNavigationPlus'; Key: 'SensorBias'; Caption: 'Смещение детектора СВП+, °'; DefaultValue: 0; Minimum: -180; Maximum: 180; Page: 7),
    (Section: 'RecordedPlant'; Key: 'Profile'; Caption: 'Модель: 0 = прежняя, 1 = EDF Roll, 2 = EDF Pitch, 3 = СВП Roll'; DefaultValue: 0; Minimum: 0; Maximum: 3; Page: 8),
    (Section: 'RecordedPlant'; Key: 'GainScale'; Caption: 'Множитель RC-характеристики'; DefaultValue: 1; Minimum: 0.1; Maximum: 3; Page: 8),
    (Section: 'RecordedPlant'; Key: 'ExtraDelay'; Caption: 'Дополнительная задержка команды, с'; DefaultValue: 0; Minimum: 0; Maximum: 0.3; Page: 8),
    (Section: 'RecordedPlant'; Key: 'FrequencyScale'; Caption: 'Множитель частоты динамики'; DefaultValue: 1; Minimum: 0.25; Maximum: 3; Page: 8),
    (Section: 'RecordedPlant'; Key: 'DampingScale'; Caption: 'Множитель демпфирования'; DefaultValue: 1; Minimum: 0.5; Maximum: 1.5; Page: 8),
    (Section: 'RecordedPlant'; Key: 'CameraHz'; Caption: 'Частота сообщений камеры, Гц'; DefaultValue: 60; Minimum: 10; Maximum: 100; Page: 8),
    (Section: 'RecordedPlant'; Key: 'CameraDelay'; Caption: 'Физическая задержка камеры, с'; DefaultValue: 0.035; Minimum: 0; Maximum: 0.5; Page: 8),
    (Section: 'RecordedPlant'; Key: 'CompensationTime'; Caption: 'Поправка угла минус Gyro × время, с'; DefaultValue: 0.035; Minimum: 0; Maximum: 0.3; Page: 8),
    (Section: 'RecordedPlant'; Key: 'NoiseScale'; Caption: 'Множитель эффективного шума угла'; DefaultValue: 1; Minimum: 0; Maximum: 5; Page: 8),
    (Section: 'RecordedPlant'; Key: 'Seed'; Caption: 'Seed шума (одинаковый для моделей)'; DefaultValue: 20261023; Minimum: 1; Maximum: 2147483647; Page: 8),
    (Section: 'RecordedPlant'; Key: 'CourseTime'; Caption: 'Время изменения курса, с (гипотеза)'; DefaultValue: 0.4; Minimum: 0; Maximum: 10; Page: 8),
    (Section: 'RecordedPlant'; Key: 'CameraBias'; Caption: 'Общее смещение датчика четырёх моделей, °'; DefaultValue: 0; Minimum: -180; Maximum: 180; Page: 8),
    (Section: 'RecordedPlant'; Key: 'OutputLimit'; Caption: 'Предел выхода всех регуляторов, ед. (RC=1500+2,5*выход)'; DefaultValue: 120; Minimum: 1; Maximum: 120; Page: 8)
  );

// Проверяет диапазон числа и целочисленность длины задержки.
function ValidValue(Index: Integer; Value: Double): Boolean;
begin
  Result := not IsNan(Value) and not IsInfinite(Value);
  if not Result then Exit;
  Result := (Value >= Definitions[Index].Minimum) and (Value <= Definitions[Index].Maximum);
  if Result and ((Index = 5) or (Index = 33) or (Index = 52) or (Index = 73) or (Index = 75) or (Index = 84)) then Result := Frac(Value) = 0;
end;

// Возвращает единый набор значений по умолчанию для диалога и INI.
function DefaultParameterValues: TParameterValues;
var I: Integer;
begin
  for I := 0 to High(Result) do Result[I] := Definitions[I].DefaultValue;
end;

// Загружает только корректные значения, поддерживая прежнее имя секции координат.
procedure ReadParameterValues(Ini: TMemIniFile; var Values: TParameterValues);
var I: Integer; V: Double; Section: string;
begin
  for I := 0 to High(Values) do
  begin
    Section := Definitions[I].Section;
    // One-time fallback for the enhanced SVP settings before it became model four.
    // Once the independent section exists, absent keys use its own defaults.
    if (Section = 'GateNavigationPlus') and not Ini.SectionExists(Section) then
      Section := 'GateNavigation';
    if (Section = 'ReferencePoint') and not Ini.SectionExists(Section) and Ini.SectionExists('Target') then
      Section := 'Target';
    // Migrate the former SVP speed only when the shared setting is absent.
    if (I = 31) and not Ini.ValueExists('Motion', 'SpeedKmh') then
      Section := 'GateNavigation';
    // Прежний IntegralLimit хранился в рад/с. Новый ключ исключает повторный пересчёт.
    if (I = 12) and not Ini.ValueExists(Section, Definitions[I].Key) then
    begin
      if TryStrToFloat(Ini.ReadString(Section, 'IntegralLimit', ''), V, TFormatSettings.Invariant) then
      begin
        V := RadToDeg(V);
        if ValidValue(I, V) then Values[I] := V;
      end;
    end
    else if TryStrToFloat(Ini.ReadString(Section, Definitions[I].Key, ''), V, TFormatSettings.Invariant) then
      if ValidValue(I, V) then Values[I] := V;
  end;
end;

// Записывает параметры в INI с независимым от языка десятичным разделителем.
procedure WriteParameterValues(Ini: TMemIniFile; const Values: TParameterValues);
var I: Integer;
begin
  for I := 0 to High(Values) do
    Ini.WriteString(Definitions[I].Section, Definitions[I].Key,
      FloatToStr(Values[I], TFormatSettings.Invariant));
  Ini.DeleteKey('Controller', 'IntegralLimit');
  Ini.DeleteKey('GateNavigation', 'SpeedKmh');
end;

// Создаёт окно с вкладками и отдельной кнопкой возврата к исходным значениям.
constructor TParametersForm.Create(AOwner: TComponent);
const Titles: array[0..8] of string = ('Отображение', 'Датчик и точка', 'PID', 'Адаптация', 'СВП', 'Движение', 'Adaptive', 'СВП+', 'По записи EDF');
var Tabs: array[0..8] of TTabSheet; Rows: array[0..8] of Integer; Panels: array[0..8] of TScrollBox;
  I, Index, PageIndex, Y: Integer; L: TLabel; Footer: TPanel; B: TButton;
begin
  inherited CreateNew(AOwner);
  Caption := 'Параметры';
  Position := poScreenCenter;
  BorderStyle := bsDialog;
  Font.Name := 'Segoe UI'; Font.Size := 9;
  ClientWidth := 680; ClientHeight := 590;
  Footer := TPanel.Create(Self); Footer.Parent := Self;
  Footer.Align := alBottom; Footer.Height := 78; Footer.BevelOuter := bvNone;
  L := TLabel.Create(Self); L.Parent := Footer; L.SetBounds(12, 6, 620, 18);
  L.Caption := 'ОК сохраняет параметры и сбрасывает расчёт. Отмена сохраняет прежние значения.';
  B := TButton.Create(Self); B.Parent := Footer; B.Name := 'DefaultsButton';
  B.SetBounds(12, 36, 225, 30); B.Caption := 'Сбросить текущую вкладку'; B.OnClick := DefaultsClick;
  B := TButton.Create(Self); B.Parent := Footer; B.Name := 'AcceptButton';
  B.SetBounds(438, 36, 90, 30); B.Caption := 'ОК'; B.Default := True; B.OnClick := AcceptClick;
  B := TButton.Create(Self); B.Parent := Footer; B.Name := 'CancelButton';
  B.SetBounds(538, 36, 100, 30); B.Caption := 'Отмена'; B.Cancel := True; B.ModalResult := mrCancel;
  FPages := TPageControl.Create(Self); FPages.Parent := Self; FPages.Align := alClient;
  for I := 0 to 8 do
  begin
    Tabs[I] := TTabSheet.Create(Self); Tabs[I].PageControl := FPages; Tabs[I].Caption := Titles[I];
    Panels[I] := TScrollBox.Create(Self); Panels[I].Parent := Tabs[I];
    Panels[I].Align := alClient; Panels[I].BorderStyle := bsNone;
    Rows[I] := 0;
  end;
  for I := 0 to ParameterCount - 1 do
  begin
    Index := I;
    if (I >= 54) and (I < 66) then Index := I + 9
    else if (I >= 66) and (I < 75) then Index := I - 12;
    PageIndex := Definitions[Index].Page; Y := 14 + Rows[PageIndex] * 35; Inc(Rows[PageIndex]);
    L := TLabel.Create(Self); L.Parent := Panels[PageIndex]; L.SetBounds(12, Y + 4, 450, 22);
    L.Caption := Definitions[Index].Caption;
    FEditors[Index] := TEdit.Create(Self); FEditors[Index].Parent := Panels[PageIndex];
    FEditors[Index].Name := 'Parameter' + IntToStr(Index);
    FEditors[Index].SetBounds(478, Y, 145, 25);
    L.FocusControl := FEditors[Index];
  end;
  FPages.ActivePageIndex := 0;
end;

// Заполняет поля, не изменяя действующие настройки расчёта.
procedure TParametersForm.SetValues(const Values: TParameterValues);
var I: Integer;
begin
  for I := 0 to High(Values) do FEditors[I].Text := FloatToStr(Values[I]);
end;

// Читает поля и сообщает индекс первого некорректного значения.
function TParametersForm.TryGetValues(out Values: TParameterValues; out InvalidIndex: Integer): Boolean;
var I: Integer;
begin
  for I := 0 to High(Values) do
    if not (TryStrToFloat(FEditors[I].Text, Values[I]) or
      TryStrToFloat(FEditors[I].Text, Values[I], TFormatSettings.Invariant)) or
      not ValidValue(I, Values[I]) then
    begin
      InvalidIndex := I; Exit(False);
    end;
  InvalidIndex := -1; Result := True;
end;

// Подставляет исходные значения; применение выполняется только кнопкой ОК.
procedure TParametersForm.DefaultsClick(Sender: TObject);
var I: Integer;
begin
  for I := 0 to High(FEditors) do
    if Definitions[I].Page = FPages.ActivePageIndex then
      FEditors[I].Text := FloatToStr(Definitions[I].DefaultValue);
end;

// Проверяет ввод перед закрытием окна и показывает ошибочное поле.
procedure TParametersForm.AcceptClick(Sender: TObject);
var Values: TParameterValues; I: Integer;
begin
  if TryGetValues(Values, I) then ModalResult := mrOk
  else
  begin
    FPages.ActivePageIndex := Definitions[I].Page;
    FEditors[I].SetFocus;
    FEditors[I].SelectAll;
    MessageDlg(Format('%s: введите число от %g до %g.%s',
      [Definitions[I].Caption, Definitions[I].Minimum, Definitions[I].Maximum,
       IfThen((I = 5) or (I = 33) or (I = 52) or (I = 73), ' Количество измерений должно быть целым.', '')]), mtError, [mbOK], 0);
  end;
end;

// Возвращает новые параметры только после подтверждения пользователем.
function EditParameters(var Values: TParameterValues): Boolean;
var F: TParametersForm; I: Integer;
begin
  F := TParametersForm.Create(nil);
  try
    F.SetValues(Values);
    Result := F.ShowModal = mrOk;
    if Result then Result := F.TryGetValues(Values, I);
  finally F.Free; end;
end;

end.

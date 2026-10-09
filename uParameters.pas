unit uParameters;

interface

uses System.Classes, System.IniFiles, Vcl.Forms, Vcl.StdCtrls, Vcl.ComCtrls;

const ParameterCount = 21;
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
  System.UITypes, uMotionModel, uAdaptiveRegulator;

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
    (Section: 'Controller'; Key: 'IntegralLimit'; Caption: 'Предел I (IntegralLimit), рад/с'; DefaultValue: IntegralLimit; Minimum: 0; Maximum: 1E6; Page: 2),
    (Section: 'Controller'; Key: 'BiasAdaptationGain'; Caption: 'Коэффициент адаптации (BiasAdaptationGain)'; DefaultValue: BiasAdaptationGain; Minimum: 0; Maximum: 1E6; Page: 3),
    (Section: 'Controller'; Key: 'BiasAngularSpeedWeight'; Caption: 'Вес W (BiasAngularSpeedWeight)'; DefaultValue: BiasAngularSpeedWeight; Minimum: 0; Maximum: 1E6; Page: 3),
    (Section: 'Controller'; Key: 'BiasAdaptationStart'; Caption: 'Начало адаптации (BiasAdaptationStart), с'; DefaultValue: BiasAdaptationStart; Minimum: 0; Maximum: 1E6; Page: 3),
    (Section: 'Controller'; Key: 'BiasFilterTime'; Caption: 'Время фильтра адаптации (BiasFilterTime), с'; DefaultValue: BiasFilterTime; Minimum: 0; Maximum: 1E6; Page: 3),
    (Section: 'Controller'; Key: 'BiasErrorGateDegrees'; Caption: 'Порог угла (BiasErrorGate), °'; DefaultValue: BiasErrorGate * 180 / Pi; Minimum: 0; Maximum: 180; Page: 3),
    (Section: 'Controller'; Key: 'BiasRateLimitDegrees'; Caption: 'Скорость поправки (BiasRateLimit), °/с'; DefaultValue: BiasRateLimit * 180 / Pi; Minimum: 0; Maximum: 1E6; Page: 3),
    (Section: 'Controller'; Key: 'BiasLimitDegrees'; Caption: 'Предел поправки (BiasLimit), °'; DefaultValue: BiasLimit * 180 / Pi; Minimum: 0; Maximum: 180; Page: 3),
    (Section: 'Controller'; Key: 'DerivativeFilterTime'; Caption: 'Время фильтра D (DerivativeFilterTime), с'; DefaultValue: DerivativeFilterTime; Minimum: 0; Maximum: 1E6; Page: 3)
  );

// Проверяет диапазон числа и целочисленность длины задержки.
function ValidValue(Index: Integer; Value: Double): Boolean;
begin
  Result := not IsNan(Value) and not IsInfinite(Value);
  if not Result then Exit;
  Result := (Value >= Definitions[Index].Minimum) and (Value <= Definitions[Index].Maximum);
  if Result and (Index = 5) then Result := Frac(Value) = 0;
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
    if (Section = 'ReferencePoint') and not Ini.SectionExists(Section) and Ini.SectionExists('Target') then
      Section := 'Target';
    if TryStrToFloat(Ini.ReadString(Section, Definitions[I].Key, ''), V, TFormatSettings.Invariant) then
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
end;

// Создаёт окно с вкладками и отдельной кнопкой возврата к исходным значениям.
constructor TParametersForm.Create(AOwner: TComponent);
const Titles: array[0..3] of string = ('Отображение', 'Датчик и точка', 'PID', 'Адаптация');
var Tabs: array[0..3] of TTabSheet; Rows: array[0..3] of Integer;
  I, PageIndex, Y: Integer; L: TLabel; Footer: TPanel; B: TButton;
begin
  inherited CreateNew(AOwner);
  Caption := 'Параметры';
  Position := poScreenCenter;
  BorderStyle := bsDialog;
  Font.Name := 'Segoe UI'; Font.Size := 9;
  ClientWidth := 650; ClientHeight := 425;
  Footer := TPanel.Create(Self); Footer.Parent := Self;
  Footer.Align := alBottom; Footer.Height := 78; Footer.BevelOuter := bvNone;
  L := TLabel.Create(Self); L.Parent := Footer; L.SetBounds(12, 6, 620, 18);
  L.Caption := 'ОК сохраняет параметры и сбрасывает расчёт. Отмена сохраняет прежние значения.';
  B := TButton.Create(Self); B.Parent := Footer; B.Name := 'DefaultsButton';
  B.SetBounds(12, 36, 225, 30); B.Caption := 'Сбросить по умолчанию'; B.OnClick := DefaultsClick;
  B := TButton.Create(Self); B.Parent := Footer; B.Name := 'AcceptButton';
  B.SetBounds(438, 36, 90, 30); B.Caption := 'ОК'; B.Default := True; B.OnClick := AcceptClick;
  B := TButton.Create(Self); B.Parent := Footer; B.Name := 'CancelButton';
  B.SetBounds(538, 36, 100, 30); B.Caption := 'Отмена'; B.Cancel := True; B.ModalResult := mrCancel;
  FPages := TPageControl.Create(Self); FPages.Parent := Self; FPages.Align := alClient;
  for I := 0 to 3 do
  begin
    Tabs[I] := TTabSheet.Create(Self); Tabs[I].PageControl := FPages; Tabs[I].Caption := Titles[I];
    Rows[I] := 0;
  end;
  for I := 0 to ParameterCount - 1 do
  begin
    PageIndex := Definitions[I].Page; Y := 14 + Rows[PageIndex] * 35; Inc(Rows[PageIndex]);
    L := TLabel.Create(Self); L.Parent := Tabs[PageIndex]; L.SetBounds(12, Y + 4, 450, 22);
    L.Caption := Definitions[I].Caption;
    FEditors[I] := TEdit.Create(Self); FEditors[I].Parent := Tabs[PageIndex];
    FEditors[I].Name := 'Parameter' + IntToStr(I);
    FEditors[I].SetBounds(478, Y, 145, 25);
    L.FocusControl := FEditors[I];
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
begin
  SetValues(DefaultParameterValues);
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
       IfThen(I = 5, ' Количество измерений должно быть целым.', '')]), mtError, [mbOK], 0);
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

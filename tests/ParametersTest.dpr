program ParametersTest;
{$APPTYPE CONSOLE}
uses System.SysUtils, System.Classes, System.Math, System.IniFiles,
  Vcl.Forms, Vcl.ComCtrls, Vcl.StdCtrls, Vcl.Graphics, Vcl.Imaging.pngimage,
  uParameters in '..\uParameters.pas', uMain in '..\uMain.pas',
  uMotionModel in '..\uMotionModel.pas', uAdaptiveRegulator in '..\uAdaptiveRegulator.pas';
// Проверяет условие и сообщает причину отказа.
procedure Check(B: Boolean; const S: string);
begin if not B then raise Exception.Create(S); end;
// Сравнивает числа с учётом округления.
procedure Near(A,B: Double; const S: string);
begin Check(Abs(A-B)<1E-9,S); end;
var Values, Loaded, Defaults: TParameterValues; Ini: TMemIniFile; Name: string;
  D: TParametersForm; F: TForm2; M, Snapshot: TMotionModel; C: TAdaptiveRegulator;
  I, Bad: Integer; Action: TCloseAction; Bitmap: TBitmap; Png: TPngImage;
begin
 try
  Application.Initialize;
  Defaults:=DefaultParameterValues;
  Near(Defaults[5],3,'Default delay'); Near(Defaults[6],-20,'Default sensor bias');
  Near(Defaults[7],50,'Default point speed'); Near(Defaults[8],-10,'Default point heading');
  Near(Defaults[11],0.5,'Default Kd'); Near(Defaults[12],10,'Default integral limit');
  D:=TParametersForm.Create(nil);
  try
   D.SetValues(Defaults);
   TEdit(D.FindComponent('Parameter5')).Text:='2.5';
   Check(not D.TryGetValues(Loaded,Bad) and (Bad=5),'Fractional delay accepted');
   TEdit(D.FindComponent('Parameter5')).Text:='1001';
   Check(not D.TryGetValues(Loaded,Bad),'Oversized delay accepted');
   TButton(D.FindComponent('DefaultsButton')).Click;
   Check(D.TryGetValues(Loaded,Bad),'Defaults invalid');
   for I:=0 to High(Loaded) do Near(Loaded[I],Defaults[I],'Defaults button');
   TEdit(D.FindComponent('Parameter10')).Text:='0';
   TEdit(D.FindComponent('Parameter11')).Text:='0';
   TEdit(D.FindComponent('Parameter16')).Text:='0';
   TEdit(D.FindComponent('Parameter20')).Text:='0';
   Check(D.TryGetValues(Loaded,Bad),'Zero gains or filter times rejected');
   TEdit(D.FindComponent('Parameter20')).Text:='-1';
   Check(not D.TryGetValues(Loaded,Bad),'Negative filter accepted');
   TButton(D.FindComponent('DefaultsButton')).Click;
   for I:=0 to D.ComponentCount-1 do
    if D.Components[I] is TPageControl then TPageControl(D.Components[I]).ActivePageIndex:=3;
   D.Show; Application.ProcessMessages;
   Bitmap:=TBitmap.Create; Png:=TPngImage.Create;
   try
    Bitmap.SetSize(D.ClientWidth,D.ClientHeight); D.PaintTo(Bitmap.Canvas,0,0);
    Png.Assign(Bitmap); Png.SaveToFile('..\Win32\LabelCheck\ParametersPreview.png');
   finally Png.Free; Bitmap.Free; end;
  finally D.Free; end;
  Values:=Defaults;
  Values[5]:=7; Values[6]:=12.5; Values[7]:=72; Values[8]:=45;
  for I:=9 to High(Values) do Values[I]:=Defaults[I]*0.7;
  Name:=ChangeFileExt(ParamStr(0),'.ini'); DeleteFile(Name);
  Ini:=TMemIniFile.Create(Name);
  try WriteParameterValues(Ini,Values); Ini.UpdateFile; finally Ini.Free; end;
  F:=TForm2.Create(nil);
  try
   Near(F.Model.AngleCount,7,'Main delay'); Near(F.Model.SensorBiasDegrees,12.5,'Main sensor bias');
   Near(F.Model.ReferencePoint.V,20,'Point speed conversion');
   Near(F.Model.ReferencePoint.Fi,Pi/4,'Point angle conversion');
   Near(F.Controller.Parameters.Kp,Values[9],'Main Kp');
   Near(F.Controller.Parameters.Ki,Values[10],'Main Ki');
   Near(F.Controller.Parameters.Kd,Values[11],'Main Kd');
   Near(F.Controller.Parameters.IntegralLimit,Values[12],'Main I limit');
   Near(F.Controller.Parameters.BiasAdaptationGain,Values[13],'Adaptation gain');
   Near(F.Controller.Parameters.BiasAngularSpeedWeight,Values[14],'W weight');
   Near(F.Controller.Parameters.BiasAdaptationStart,Values[15],'Adaptation start');
   Near(F.Controller.Parameters.BiasFilterTime,Values[16],'Bias filter');
   Near(RadToDeg(F.Controller.Parameters.BiasErrorGate),Values[17],'Gate conversion');
   Near(RadToDeg(F.Controller.Parameters.BiasRateLimit),Values[18],'Rate conversion');
   Near(RadToDeg(F.Controller.Parameters.BiasLimit),Values[19],'Limit conversion');
   Near(F.Controller.Parameters.DerivativeFilterTime,Values[20],'D filter');
   F.Integrate(0.01); F.InitButtonClick(nil);
   Near(F.Controller.Parameters.Kd,Values[11],'Init reset parameters');
   Near(F.Model.AngleCount,7,'Init reset delay');
   F.OffsetPlusButtonClick(nil); Values[6]:=Values[6]+1;
   Action:=caFree; F.FormClose(nil,Action);
  finally F.Free; end;
  Ini:=TMemIniFile.Create(Name);
  try
   Loaded:=Defaults; ReadParameterValues(Ini,Loaded);
   for I:=0 to High(Values) do Near(Loaded[I],Values[I],'INI round trip '+IntToStr(I));
   Ini.WriteString('Sensor','AngleCount','1.5');
   Ini.WriteString('Controller','BiasFilterTime','-1');
   Loaded:=Defaults; ReadParameterValues(Ini,Loaded);
   Near(Loaded[5],Defaults[5],'Invalid delay fallback');
   Near(Loaded[16],Defaults[16],'Invalid filter fallback');
  finally Ini.Free; end;
  DeleteFile(Name);
  M.Init; M.AngleCount:=0;
  Near(M.MeasureAngle,DegToRad(45+DefaultSensorBiasDegrees),'Zero delay');
  M.AngleCount:=2; M.ResetMotion; Snapshot:=M;
  Near(M.MeasureAngle,0,'Delay slot 1'); Near(M.MeasureAngle,0,'Delay slot 2');
  Near(M.MeasureAngle,DegToRad(45+DefaultSensorBiasDegrees),'Delayed sample');
  Near(Snapshot.MeasureAngle,0,'Snapshot ring mutated');
  C.Init; C.Parameters.Ki:=0; C.Parameters.Kd:=0;
  C.Parameters.BiasFilterTime:=0; C.Parameters.DerivativeFilterTime:=0;
  C.Wint:=3; C.Wdiff:=4; C.Update(0.01,0.1,0);
  Near(C.Wint,0,'Disabled I retained'); Near(C.Wdiff,0,'Disabled D retained');
  Near(C.W,C.Parameters.Kp*0.1,'P only output');
  C.Reset; Near(C.Parameters.Ki,0,'Regulator reset parameters');
  Writeln('PASS: defaults, validation, UI reset, all settings INI round trip, init, delay 0/2, snapshots, zero I/D');
 except on E: Exception do begin Writeln(E.Message); ExitCode:=1; end; end;
end.

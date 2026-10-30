program SharedMotionTest;
{$APPTYPE CONSOLE}
uses System.SysUtils, System.Math, System.IniFiles, Vcl.Forms, Vcl.StdCtrls,
 uMain in '..\uMain.pas', uParameters in '..\uParameters.pas', uMotionModel in '..\uMotionModel.pas';
procedure Check(B:Boolean; const S:string);
begin if not B then raise Exception.Create(S); end;
procedure Near(A,B:Double; const S:string);
begin Check(Abs(A-B)<1E-9,S); end;
var Ini:TMemIniFile; V,L:TParameterValues; F:TForm2; D:TParametersForm; I,Bad:Integer; Name:string;
begin
 try
  Application.Initialize;
  Name:=ChangeFileExt(ParamStr(0),'.ini'); DeleteFile(Name);
  Ini:=TMemIniFile.Create(Name);
  try
   V:=DefaultParameterValues;
   Ini.WriteString('GateNavigation','SpeedKmh','310');
   ReadParameterValues(Ini,V); Near(V[31],310,'Legacy speed migration');
   V[35]:=90; WriteParameterValues(Ini,V);
   Check(not Ini.ValueExists('GateNavigation','SpeedKmh'),'Legacy key removed');
   Ini.WriteString('GateNavigation','SpeedKmh','50');
   L:=DefaultParameterValues; ReadParameterValues(Ini,L); Near(L[31],310,'Shared speed precedence');
   Near(L[35],90,'Angle persistence');
   Ini.WriteBool('Simulation','AutoStopEnabled',False);
   Ini.UpdateFile;
  finally Ini.Free; end;
  F:=TForm2.Create(nil);
  try
   Near(F.Model.V,310/3.6,'PID speed'); Near(F.Gate.Parameters.SpeedKmh,310,'Gate speed');
   Near(RadToDeg(F.Model.Fi),90,'PID initial angle');
   Near(F.Gate.Heading,90,'Gate initial heading'); Near(F.Gate.Course,90,'Gate initial course');
   for I:=1 to 20 do F.Integrate(0.01);
   F.InitButtonClick(nil);
   Near(F.Model.V,310/3.6,'PID speed reset'); Near(F.Gate.Parameters.SpeedKmh,310,'Gate speed reset');
   Near(RadToDeg(F.Model.Fi),90,'PID angle reset'); Near(F.Gate.Heading,90,'Gate angle reset');
  finally F.Free; end;
  Ini:=TMemIniFile.Create(Name);
  try
   V:=DefaultParameterValues; V[31]:=0; V[35]:=-90;
   WriteParameterValues(Ini,V); Ini.UpdateFile;
  finally Ini.Free; end;
  F:=TForm2.Create(nil);
  try
   F.Integrate(0.01); Near(F.Model.X,0,'Zero PID speed'); Near(F.Gate.X,0,'Zero Gate speed');
   Near(F.Model.Y,0,'Zero PID speed Y'); Near(F.Gate.Y,0,'Zero Gate speed Y');
   F.InitButtonClick(nil); Near(RadToDeg(F.Model.Fi),-90,'Negative PID angle'); Near(F.Gate.Course,-90,'Negative Gate angle');
  finally F.Free; end;
  D:=TParametersForm.Create(nil);
  try
   D.SetValues(DefaultParameterValues);
   TEdit(D.FindComponent('Parameter35')).Text:='361';
   Check(not D.TryGetValues(V,Bad) and (Bad=35),'Angle bounds');
   TEdit(D.FindComponent('Parameter35')).Text:='-90';
   Check(D.TryGetValues(V,Bad),'Negative angle accepted');
  finally D.Free; end;
  DeleteFile(Name);
  Writeln('PASS: shared speed/heading, migration, INI, reset, zero speed, negative angle, validation');
 except on E:Exception do begin Writeln(E.Message); ExitCode:=1; end; end;
end.

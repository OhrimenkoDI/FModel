program GateSettingsTest;
{$APPTYPE CONSOLE}
uses System.SysUtils, System.Math, System.IniFiles, Vcl.Forms,
  uMain in '..\uMain.pas', uParameters in '..\uParameters.pas',
  uGateNavigationPlus in '..\uGateNavigationPlus.pas';
procedure Check(B:Boolean;const S:string);
begin if not B then raise Exception.Create(S);end;
procedure Near(A,B:Double;const S:string);
begin Check(Abs(A-B)<1E-9,S);end;
procedure CheckFields(const P:TGatePlusParameters;const V:TParameterValues);
begin
 Near(P.ObserverSlipTime,V[54],'ObserverSlipTime'); Near(P.LeadFilterTime,V[55],'LeadFilterTime');
 Near(P.MaxCommandAcceleration,V[56],'MaxCommandAcceleration');
 Near(P.TerminalCommandAcceleration,V[57],'TerminalCommandAcceleration');
 Near(P.TerminalMaxRate,V[58],'TerminalMaxRate');Near(P.PredictionTime,V[59],'PredictionTime');
 Near(P.SafeRate,V[60],'SafeRate');Near(P.DetectorHalfAngle,V[61],'DetectorHalfAngle');
 Near(P.InitialSlipDegrees,V[62],'InitialSlipDegrees');
end;
var Ini:TMemIniFile;V,L:TParameterValues;F:TForm2;Name:string;Action:TCloseAction;I:Integer;
begin
 try
  Application.Initialize;Name:=ChangeFileExt(ParamStr(0),'.ini');
  V:=DefaultParameterValues;CheckFields(DefaultGatePlusParameters,V);
  if FileExists(Name) then DeleteFile(Name);
  Ini:=TMemIniFile.Create(Name);
  try
   // Old files retain their chosen aiming term and near-zone radius.
   Ini.WriteString('GateNavigation','FreezeDistance','7');
   Ini.WriteString('GateNavigation','KLambda','0.25');Ini.UpdateFile;
  finally Ini.Free;end;
  F:=TForm2.Create(nil);
  try
   CheckFields(F.GatePlus.Parameters,V);Near(F.GatePlus.Parameters.FreezeDistance,7,'Legacy near zone');
   Near(F.GatePlus.Parameters.KLambda,0.25,'Existing aim setting');
  finally F.Free;end;
  V[54]:=0.7;V[55]:=0.2;V[56]:=200;V[57]:=50;V[58]:=20;
  V[59]:=0.4;V[60]:=10;V[61]:=70;V[62]:=-15;
  Ini:=TMemIniFile.Create(Name);
  try WriteParameterValues(Ini,V);Ini.UpdateFile;finally Ini.Free;end;
  F:=TForm2.Create(nil);
  try
   CheckFields(F.GatePlus.Parameters,V);
   Near(F.GatePlus.Heading-F.GatePlus.Course,-15,'Initial slip applied');
   for I:=1 to 100 do F.Integrate(0.01);
   Check(Abs(F.GatePlus.Controller.EstimatedSlip)>0.01,'Observer active');
   F.InitButtonClick(nil);CheckFields(F.GatePlus.Parameters,V);
   Near(F.GatePlus.Controller.EstimatedSlip,0,'Observer reset');
   Near(F.GatePlus.Heading-F.GatePlus.Course,-15,'Initial slip restored');
   Action:=caFree;F.FormClose(nil,Action);
  finally F.Free;end;
  Ini:=TMemIniFile.Create(Name);
  try
   L:=DefaultParameterValues;ReadParameterValues(Ini,L);
   for I:=54 to High(V) do Near(V[I],L[I],'Form save round trip '+IntToStr(I));
  finally Ini.Free;end;
  DeleteFile(Name);
  Writeln('PASS: gate defaults, legacy settings, all new fields load/save, slip initialization/reset');
 except on E:Exception do begin Writeln(E.ClassName,': ',E.Message);ExitCode:=1;end;end;
end.
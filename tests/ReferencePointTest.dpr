program ReferencePointTest;
{$APPTYPE CONSOLE}
uses
  System.SysUtils, System.Math, System.Classes, System.IniFiles, Vcl.Forms, Vcl.Controls,
  uMain in '..\uMain.pas', uMotionModel in '..\uMotionModel.pas',
  uAdaptiveRegulator in '..\uAdaptiveRegulator.pas';
// Завершает проверку с ошибкой, если условие не выполнено.
procedure Check(B: Boolean; const S: string);
begin
  if not B then raise Exception.Create(S);
end;
var
  M: TMotionModel;
  P, Q: TReferencePoint;
  I: Integer;
  CloseAction: TCloseAction;
  A: Double;
  F: TForm2;
  Ini: TMemIniFile;
  Name: string;
begin
  try
    P.Init;
    Check(Abs(P.V - ReferencePointSpeedKmh / 3.6) < 1E-9, 'Initial speed');
    Check(Abs(P.Fi - DegToRad(ReferencePointHeadingDegrees)) < 1E-9, 'Initial heading');
    P.InitialX := 12; P.InitialY := -8; P.ResetMotion;
    P.V := 10; P.Fi := Pi / 3 - 3*deg; Q := P;
    P.Integrate(2);
    for I := 1 to 200 do Q.Integrate(0.01);
    Check((Abs(P.X - 22) < 1E-9) and (Abs(P.Y - (-8 + 10*Sqrt(3))) < 1E-9), 'Uniform oblique motion');
    Check((Abs(P.X-Q.X) < 1E-9) and (Abs(P.Y-Q.Y) < 1E-9), 'Subdivision invariance');
    Check((P.V = 10) and (Abs(P.Fi - (Pi/3 - 3*deg)) < 1E-12), 'Speed or heading changed');
    P.V := 0; Q := P; P.Integrate(5);
    Check((P.X = Q.X) and (P.Y = Q.Y), 'Zero speed');
    P.ResetMotion;
    Check((P.X = 12) and (P.Y = -8), 'Initial coordinates reset');
    Check(Abs(P.V - ReferencePointSpeedKmh/3.6) < 1E-9, 'Speed reset');
    M.Init;
    Check(M.TryAngleToPoint(100, 100, A) and (Abs(A - 45) < 1E-9), '45 degrees');
    Check(M.TryAngleToPoint(0, -100, A) and (Abs(A + 90) < 1E-9), '-90 degrees');
    M.Fi := Pi / 2;
    Check(M.TryAngleToPoint(100, 0, A) and (Abs(A + 90) < 1E-9), 'Heading');
    Check(not M.TryAngleToPoint(0, 0, A), 'Coincident points');
    M.V := 0;
    Check(not M.TryAngleToPoint(100, 100, A), 'Zero speed');
    Application.Initialize;
    Name := ChangeFileExt(ParamStr(0), '.ini');
    DeleteFile(Name);
    Ini := TMemIniFile.Create(Name);
    try
      Ini.WriteBool('Controller', 'DelayCorrection', True);
      Ini.WriteBool('Sensor', 'HeadingOffsetEnabled', False);
      Ini.WriteString('View', 'PixelsPerMetre', '1');
      Ini.WriteInteger('View', 'OriginX', 200);
      Ini.WriteInteger('View', 'OriginY', 200);
      Ini.WriteString('Model', 'Diameter', '2.5');
      Ini.WriteString('Target', 'Diameter', '4');
      Ini.UpdateFile;
    finally Ini.Free; end;
    F := TForm2.Create(nil);
    try
      Check((F.ReferencePointX = 100) and (F.ReferencePointY = 100), 'Defaults');
      Check(F.Model.SensorBiasDegrees = DefaultSensorBiasDegrees, 'Default heading offset');
      F.OffsetPlusButton.Click;
      Check(F.Model.SensorBiasDegrees = DefaultSensorBiasDegrees + 1, 'Plus heading offset');
      F.OffsetMinusButton.Click;
      Check(F.Model.SensorBiasDegrees = DefaultSensorBiasDegrees, 'Minus heading offset');
      F.AngleCorrectionCheckBox.Checked := False;
      Check(not F.Controller.AngleCorrectionEnabled, 'Angle correction toggle');
      Check((F.Model.Diameter = 2.5) and (F.Model.ReferencePoint.Diameter = 4), 'Diameter load');
      F.Image1MouseDown(F.Image1, mbLeft, [ssLeft], 302, 102);
      F.Image1MouseMove(F.Image1, [ssLeft], 312, 122);
      F.Image1MouseUp(F.Image1, mbLeft, [], 312, 122);
      Check((F.ReferencePointX = 110) and (F.ReferencePointY = 80), 'Drag in world units');
      F.InitButtonClick(nil);
      Check(not F.Controller.AngleCorrectionEnabled, 'Init changed toggles');
      Check((F.Model.Diameter = 2.5) and (F.Model.ReferencePoint.Diameter = 4), 'Init changed diameter');
      F.Integrate(0.01);
      Check(Abs(F.ReferencePointX - (110 + F.Model.ReferencePoint.V*Cos(F.Model.ReferencePoint.Fi + 3*deg)*0.01)) < 1E-9, 'Moving reference X');
      Check(Abs(F.ReferencePointY - (80 + F.Model.ReferencePoint.V*Sin(F.Model.ReferencePoint.Fi + 3*deg)*0.01)) < 1E-9, 'Moving reference Y');
      F.InitButtonClick(nil);
      Check((F.ReferencePointX = 110) and (F.ReferencePointY = 80), 'Init restores start');
      F.Integrate(0.2);
      CloseAction := caFree;
      F.FormClose(nil, CloseAction);
    finally F.Free; end;
    F := TForm2.Create(nil);
    try
      Check((F.ReferencePointX = 110) and (F.ReferencePointY = 80), 'INI restore');
      Check(not F.AngleCorrectionCheckBox.Checked,
        'Toggle INI restore');
      Check(not F.Controller.AngleCorrectionEnabled,
        'Controller toggle INI restore');
      Check((F.Model.Diameter = 2.5) and (F.Model.ReferencePoint.Diameter = 4), 'Diameter restore');
      F.Image1MouseDown(F.Image1, mbLeft, [ssLeft], 200, 200);
      F.Image1MouseUp(F.Image1, mbLeft, [], 220, 220);
      Check((F.ReferencePointX = 110) and (F.ReferencePointY = 80), 'Origin drag changed reference point');
    finally F.Free; end;
    Ini := TMemIniFile.Create(Name);
    try
      Check(not Ini.ValueExists('Controller', 'DelayCorrection'), 'Obsolete delay setting');
      Check(not Ini.ValueExists('Sensor', 'HeadingOffsetEnabled'), 'Obsolete sensor setting');
      Check(Ini.ReadString('ReferencePoint', 'Diameter', '') = '4', 'Point settings migration');
      Check(not Ini.SectionExists('Target'), 'Legacy point section retained');
    finally Ini.Free; end;
    DeleteFile(Name);
    Writeln('PASS: angle, defaults, uniform motion, dragging, reset, initial coordinates INI restore');
  except
    on E: Exception do begin Writeln(E.Message); ExitCode := 1; end;
  end;
end.

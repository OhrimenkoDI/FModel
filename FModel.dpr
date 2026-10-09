program FModel;

uses
  Vcl.Forms,
  uMain in 'uMain.pas' {Form2},
  uMotionModel in 'uMotionModel.pas',
  uAdaptiveRegulator in 'uAdaptiveRegulator.pas',
  uParameters in 'uParameters.pas';

{$R *.res}

begin
  Application.Initialize;
  Application.MainFormOnTaskbar := True;
  Application.CreateForm(TForm2, Form2);
  Application.Run;
end.

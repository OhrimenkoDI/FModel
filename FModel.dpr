program FModel;

uses
  Vcl.Forms,
  uMain in 'uMain.pas' {Form2},
  uMotionModel in 'uMotionModel.pas',
  uPID in 'uPID.pas',
  uParameters in 'uParameters.pas',
  uGateNavigation in 'uGateNavigation.pas',
  uGateNavigationPlus in 'uGateNavigationPlus.pas',
  uAdaptiveGlide in 'uAdaptiveGlide.pas',
  uRecordedPlant in 'uRecordedPlant.pas';

{$R *.res}

begin
  Application.Initialize;
  Application.MainFormOnTaskbar := True;
  Application.CreateForm(TForm2, Form2);
  Application.Run;
end.

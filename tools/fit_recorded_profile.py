import numpy as np,json
from pathlib import Path
import argparse
parser=argparse.ArgumentParser(description='Fit the 2026-10-23 EDF RC/gyro profile; requires NumPy. Does not edit Delphi constants.')
parser.add_argument('edf',type=Path)
parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'profiles'/'EDF-20261023-refit')
args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
with args.edf.open('rb') as f:
 header=f.read(256);count=int(header[252:256]);size=int(header[184:192]);duration=float(header[244:252]);signal=f.read(size-256)
 fields=[];offset=0
 for width in (16,80,8,8,8,8,8,80,8,32):
  fields.append([signal[offset+j*width:offset+(j+1)*width].decode('ascii').strip() for j in range(count)]);offset+=width*count
 samples=[int(x) for x in fields[8]]
 if len(set(samples))!=1 or abs(samples[0]/duration-100)>1e-8:raise ValueError('This calibration requires all signals at 100 Hz')
 raw=np.frombuffer(f.read(),dtype='<i2').reshape(-1,count,samples[0]);D={}
 for j,label in enumerate(fields[0]):
  low,high,dl,dh=[float(fields[k][j]) for k in (3,4,5,6)]
  D[label]=(raw[:,j,:].ravel().astype(float)-dl)*(high-low)/(dh-dl)+low
D['t']=np.arange(len(D['AUX1']))/100
t=D['t'];manual=D['AUX1']<1000;valid=manual.copy()
for k in range(1,51):valid[k:]&=manual[:-k];valid[:-k]&=manual[k:]
valid&=(t>=3)&(D['Throttle']>-60)
result={}
for axis,g,sig,lag in [('Roll','X',1,-3),('Pitch','Y',-1,-2)]:
 x=D[axis]*sig;y=D['MV.BNO.Gyro_Sn.'+g];idx=np.where(valid)[0];idx=idx[(idx+lag>=0)&(idx+lag<len(t))]
 bins=[]
 for lo,hi in [(2,5),(5,10),(10,15),(15,20),(20,25),(25,30),(30,40),(40,50)]:
  jj=idx[(abs(x[idx])>=lo)&(abs(x[idx])<hi)]
  if len(jj)<10:continue
  bins.append((float(np.mean(abs(x[jj]))),float(np.median(y[jj+lag]*np.sign(x[jj]))),len(jj)))
 b=np.array(bins); A=np.column_stack([b[:,0],b[:,0]**3]);coef=np.linalg.lstsq(A,b[:,1],rcond=None)[0]
 # Fit dynamics on first 1.0 s after Gate saturation; reserve the remaining segment.
 start,end=27312,27518;u=D['SOL_Filter'+axis][start:end]/2;obs=y[start:end];target=coef[0]*u+coef[1]*u**3
 fs,zs,ds=np.meshgrid(np.arange(3,15.001,.25),np.arange(.2,.951,.025),np.arange(0,10),indexing='ij');fs=fs.ravel();zs=zs.ravel();ds=ds.ravel();w=2*np.pi*fs;wd=w*np.sqrt(1-zs*zs);dec=np.exp(-zs*w*.01);c=np.cos(wd*.01);s=np.sin(wd*.01)
 rate=np.full(len(fs),obs[0]);vel=np.zeros(len(fs));sse=np.zeros(len(fs));trainend=112
 # Gyro[k] is observed before issuing command[k].
 for k in range(1,trainend):
  ix=k-1-ds;v=np.where(ix>=0,target[np.maximum(ix,0)],0.);e=rate-v
  rate_new=v+dec*(e*c+(vel+zs*w*e)*s/wd);vel=dec*(vel*c-(zs*w*vel+w*w*e)*s/wd);rate=rate_new
  if k>=8:sse+=(rate-obs[k])**2
 best=np.argmin(sse);f,z,delay=float(fs[best]),float(zs[best]),int(ds[best]);om=2*np.pi*f;od=om*np.sqrt(1-z*z);de=np.exp(-z*om*.01);cc=np.cos(od*.01);ss=np.sin(od*.01)
 pred=[float(obs[0])];v=0.
 for k in range(1,len(obs)):
  q=target[k-1-delay] if k-1-delay>=0 else 0.;e=pred[-1]-q
  r=q+de*(e*cc+(v+z*om*e)*ss/od);v=de*(v*cc-(z*om*v+om*om*e)*ss/od);pred.append(float(r))
 pred=np.array(pred);print(axis,'curve',coef,'dynamics',f,z,delay*.01,'trainRMSE',np.sqrt(np.mean((pred[8:trainend]-obs[8:trainend])**2)),'holdRMSE',np.sqrt(np.mean((pred[trainend:]-obs[trainend:])**2)),'holdcorr',np.corrcoef(pred[trainend:],obs[trainend:])[0,1])
 result[axis]=dict(linear=float(coef[0]),cubic=float(coef[1]),frequency_hz=f,damping=z,delay_s=delay*.01,bins=bins,train_rmse=float(np.sqrt(np.mean((pred[8:trainend]-obs[8:trainend])**2))),holdout_rmse=float(np.sqrt(np.mean((pred[trainend:]-obs[trainend:])**2))),holdout_correlation=float(np.corrcoef(pred[trainend:],obs[trainend:])[0,1]))
 np.savetxt(args.output/(axis+'_replay.csv'),np.column_stack([t[start:end],D['SOL_Filter'+axis][start:end],obs,pred]),delimiter=',',header='time,command,gyro,model',comments='')
(args.output/'fit.json').write_text(json.dumps(result,indent=2))

"""Read all Hovercraft EDFs; fit causal command -> gyro dynamics on whole-log split."""
from pathlib import Path
import sys,json,hashlib
import numpy as np
import argparse
from types import SimpleNamespace

def read_header(f):
 fixed=f.read(256);count=int(fixed[252:256]);size=int(fixed[184:192]);raw=f.read(size-256);fields=[];offset=0
 for width in (16,80,8,8,8,8,8,80,8,32):
  fields.append([raw[offset+i*width:offset+(i+1)*width] for i in range(count)]);offset+=width*count
 samples=[int(x) for x in fields[8]]
 if len(set(samples))!=1 or samples[0]/float(fixed[244:252])!=100:raise ValueError('Expected common 100 Hz EDF channels')
 return SimpleNamespace(record_count=int(fixed[236:244]),labels=[x.decode('ascii').strip() for x in fields[0]],samples_per_record=samples,signal_fields=fields)

parser=argparse.ArgumentParser(description='Fit Hovercraft angular dynamics; excludes LOG0001-4 and non-excited/short records.')
parser.add_argument('source',type=Path)
parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'profiles'/'Hovercraft-refit')
parser.add_argument('--decode-only',action='store_true')
args=parser.parse_args();SRC=args.source;OUT=args.output;OUT.mkdir(parents=True,exist_ok=True)
records=[];manifest=[]
for file in sorted(SRC.glob('LOG*.EDF')):
 num=int(file.stem[3:])
 with file.open('rb') as f:
  h=read_header(f);a=np.frombuffer(f.read(),dtype='<i2').reshape(h.record_count,len(h.labels),h.samples_per_record[0])
 d={}
 for i,k in enumerate(h.labels):
  lo,hi,dl,dh=[float(h.signal_fields[j][i]) for j in (3,4,5,6)];d[k]=(a[:,i].ravel().astype(float)-dl)*(hi-lo)/(dh-dl)+lo
 t=d.get('RecordTick100Hz',np.arange(len(next(iter(d.values()))))*.01)
 ends=np.flatnonzero(np.diff(t)>0);n=int(ends[-1]+2) if len(ends) else len(t)
 d={k:v[:n] for k,v in d.items()};t=t[:n]
 np.savez_compressed(OUT/(file.stem+'.npz'),**d)
 if 'CTRL_Branch' not in d:continue
 active=np.rint(d['CTRL_Branch'])>=2
 ix=np.flatnonzero(active)
 row={'log':num,'sha256':hashlib.sha256(file.read_bytes()).hexdigest(),'duration':float(t[-1]),'used':False}
 if len(ix):
  row.update(start=float(t[ix[0]]),end=float(t[ix[-1]]),branch=int(round(d['CTRL_Branch'][ix[0]])),error=float(d['SOL_ErrRoll'][ix[0]]))
  # Same a-priori exclusions for every run: startup .1 s and terminal .5 s.
  first=ix[0]+10;last=ix[-1]-50
  if num>=5 and last-first>=90 and np.all(active[first:last]):
   u=(d['RES_RollRC'][first:last]-1500)/2.5;y=d['MV.BNO.Gyro_Sn.X'][first:last]
   if np.std(u)>1 and np.std(y)>0.5:
    records.append(dict(log=num,u=u,y=y,t=t[first:last]));row['used']=True;row['split']='train' if num%2 else 'holdout'
 manifest.append(row)
print('LOG duration active_start end branch fit',flush=True)
for r in manifest:print(r['log'],round(r['duration'],2),r.get('start'),r.get('end'),r.get('branch'),r['used'],flush=True)
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2))
if '--decode-only' in sys.argv:sys.exit()
# Fixed-seed candidate search; unit-gain second order response, exact ZOH.
rng=np.random.default_rng(20261029);count=6000
freq=np.exp(rng.uniform(np.log(.7),np.log(15),count));zeta=rng.uniform(.25,.98,count);delay=rng.integers(0,26,count);dead=rng.choice([0.,1.,2.,3.,5.],count)
w=2*np.pi*freq;wd=w*np.sqrt(1-zeta*zeta);dec=np.exp(-zeta*w*.01);cs=np.cos(wd*.01);sn=np.sin(wd*.01)
cross=np.zeros(count);power=np.zeros(count);yy=0
for r in records:
 if r['log']%2==0:continue
 u,y=r['u'],r['y'];rate=np.zeros(count);acc=np.zeros(count)
 # Initial-rate homogeneous response is independent of fitted gain.
 init=np.full(count,y[0]);iv=np.zeros(count)
 scale=max(8.,np.std(y));weight=1/(len(y)*scale**2)
 for k in range(len(y)):
  if k>=35:
   target=y[k]-init;cross+=weight*rate*target;power+=weight*rate*rate;yy+=weight*y[k]**2
  idx=np.maximum(0,k-delay);cmd=u[idx];cmd=np.sign(cmd)*np.maximum(0,abs(cmd)-dead)
  err=rate-cmd;nr=cmd+dec*(err*cs+(acc+zeta*w*err)*sn/wd);acc=dec*(acc*cs-(zeta*w*acc+w*w*err)*sn/wd);rate=nr
  nr=dec*(init*cs+(iv+zeta*w*init)*sn/wd);iv=dec*(iv*cs-(zeta*w*iv+w*w*init)*sn/wd);init=nr
gain=np.clip(cross/np.maximum(power,1e-10),.05,5)
# Rank by exact residual, including initial state; least-squares gain alone gives same ranking after initial transient.
loss=np.zeros(count)
for r in records:
 if r['log']%2==0:continue
 u,y=r['u'],r['y'];rate=np.full(count,y[0]);acc=np.zeros(count);weight=1/(len(y)*max(8.,np.std(y))**2)
 for k in range(len(y)):
  if k>=35:loss+=weight*(rate-y[k])**2
  cmd=u[np.maximum(0,k-delay)];target=gain*np.sign(cmd)*np.maximum(0,abs(cmd)-dead);err=rate-target
  nr=target+dec*(err*cs+(acc+zeta*w*err)*sn/wd);acc=dec*(acc*cs-(zeta*w*acc+w*w*err)*sn/wd);rate=nr
best=int(np.argmin(loss));fit={k:float(v[best]) for k,v in dict(gain=gain,frequency=freq,damping=zeta,delay=delay*.01,deadzone=dead).items()}
print('FIT',fit,flush=True)
results=[]
for r in records:
 u,y=r['u'],r['y'];rate=y[0];acc=0.;pred=[];j=best
 for k in range(len(y)):
  pred.append(rate);cmd=u[max(0,k-int(delay[j]))];target=gain[j]*np.sign(cmd)*max(0,abs(cmd)-dead[j]);err=rate-target
  nr=target+dec[j]*(err*cs[j]+(acc+zeta[j]*w[j]*err)*sn[j]/wd[j]);acc=dec[j]*(acc*cs[j]-(zeta[j]*w[j]*acc+w[j]**2*err)*sn[j]/wd[j]);rate=nr
 pred=np.array(pred);obs=y[35:];res=pred[35:]-obs
 item={'log':r['log'],'split':'train' if r['log']%2 else 'holdout','rmse':float(np.sqrt(np.mean(res**2))),'r2':float(1-np.sum(res**2)/max(1e-9,np.sum((obs-obs.mean())**2))),'gyro_std':float(np.std(obs))}
 results.append(item);print(item,flush=True)
 np.savetxt(OUT/f"replay_{r['log']:04}.csv",np.c_[r['t'],u,y,pred],delimiter=',',header='time,command,gyro,predicted',comments='')
(OUT/'fit.json').write_text(json.dumps({'fit':fit,'results':results},indent=2))

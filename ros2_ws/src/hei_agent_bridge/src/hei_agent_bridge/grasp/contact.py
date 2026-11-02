"""Fresh sustained jaw/load contact evidence, never commanded current or force calibration."""
import math
from statistics import median


def contact_evidence(samples, *, now, closed_gap_m, open_gap_m, gap_per_rad):
    result=dict(physical_capture_verified=False,contact_state='feedback_unavailable',torque_nm=0.)
    rows=[r for r in samples if 0<=now-r[0]<=.4]
    if len(rows)<3 or rows[-1][0]-rows[0][0]<.25 or now-rows[-1][0]>.2:
        return result
    if any(not all(math.isfinite(x) for x in r) for r in rows):return result
    if any(not 0<b[0]-a[0]<=.1+1e-9 for a,b in zip(rows,rows[1:])):
        result['contact_state']='feedback_discontinuous'
        return result
    gaps=[max(0.,-r[1]*abs(gap_per_rad)) for r in rows]
    torques=[r[3] for r in rows]
    result['torque_nm']=float(median(torques))
    if any(r[4]!=1 for r in rows):result['contact_state']='motor_disabled'
    elif min(gaps)<=closed_gap_m+.002:result['contact_state']='closed_without_object'
    elif max(gaps)>=open_gap_m-.002:result['contact_state']='jaw_open'
    elif max(gaps)-min(gaps)>.001 or any(abs(r[2])>.05 for r in rows):result['contact_state']='jaw_moving'
    elif abs(median(torques))<.1 or sum(abs(t)>=.1 for t in torques)/len(torques)<.8:
        result['contact_state']='insufficient_measured_load'
    else:
        result.update(physical_capture_verified=True,contact_state='sustained_jaw_load_contact')
    return result

"""Read-only wrist image benchmark. No ROS, IK planning or hardware calls.

Use an explicit calibration for the entire supplied corpus. A replayed image
match is NOT a closed-loop grasp success and does not establish ground truth.
"""
import argparse
import json
from pathlib import Path
import tempfile
import time
import cv2
import numpy as np
from hei_agent_bridge.grasp.kinematics import RightArmFK
from hei_agent_bridge.grasp.wrist_tracking import create_anchor,track_translation,_track_sparse_translation,calibration_key
from hei_agent_bridge.grasp.wrist_dense import dense_translation
from hei_agent_bridge.grasp.wrist_keyframe import create_keyframe
from hei_agent_bridge.grasp.wrist import WristTargetError


def benchmark(corpus,calibration,urdf,negative_checks=False):
    fk=RightArmFK(str(urdf));rows=[];skipped=[];negatives=[];started=time.monotonic()
    with tempfile.TemporaryDirectory(prefix='hei-wrist-replay-') as temp:
        for directory in sorted(Path(corpus).iterdir()):
            if not directory.is_dir() or not (directory/'wrist_00').is_dir():continue
            try:
                first=directory/'wrist_00'
                request=json.loads((first/'request.json').read_text());diagnostics=json.loads((first/'wrist.json').read_text())
                if not (first/'rgb.jpg').is_file() or not (first/'wrist_mask.png').is_file() or 'delta_m' not in diagnostics:
                    raise ValueError('no accepted initial mask/image')
                if request['origin_plan'].get('urdf_sha256')!=fk.sha256:raise ValueError('different or unknown URDF')
                image=cv2.imread(str(first/'rgb.jpg'));mask=cv2.imread(str(first/'wrist_mask.png'),0)
                camera=fk(request['state']['right'])@calibration['T_tcp_camera'];obb=request['reference_plan']['target_obb']
                anchor=create_anchor(image,mask,calibration,camera,obb,np.array(diagnostics['delta_m']),Path(temp)/(directory.name+'.npz'))
                bound=request['profile']['wrist_refinement']['max_total_m']
            except (OSError,KeyError,ValueError) as exc:
                skipped.append({'plan_id':directory.name,'reason':str(exc)});continue
            for observation in sorted(directory.glob('wrist_[0-9][0-9]'))[1:]:
                row={'plan_id':directory.name,'iteration':observation.name}
                try:
                    r=json.loads((observation/'request.json').read_text());current=cv2.imread(str(observation/'rgb.jpg'))
                    if current is None:raise ValueError('missing image')
                    pose=fk(r['state']['right'])@calibration['T_tcp_camera'];target=r['reference_plan']['target_obb']
                    diag={}
                    try:
                        _track_sparse_translation(current,calibration,pose,target,anchor,diag);row['sparse_accepted']=True
                    except WristTargetError as exc:
                        row.update(sparse_accepted=False,sparse_error=str(exc))
                    diag={};t=time.monotonic()
                    try:
                        delta=track_translation(current,calibration,pose,target,anchor,diag,max_distance=bound)
                        row.update(combined_accepted=True,delta_m=delta.tolist())
                        if not diag.get('keyframe_recovery'):
                            try:
                                anchor['keyframe']=create_keyframe(current,calibration,pose,target,delta,anchor,
                                    Path(temp)/(directory.name+observation.name+'.npz'))
                            except (ValueError,cv2.error) as refresh_error:row['keyframe_refresh_error']=str(refresh_error)
                        if negative_checks and anchor.get('keyframe'):
                            with np.load(anchor['keyframe']['anchor']) as data:kmask=data['mask']*255
                            removed=cv2.inpaint(current,cv2.dilate(kmask,np.ones((15,15),np.uint8)),5,cv2.INPAINT_TELEA)
                            cases={'blank':np.full_like(current,100),'noise':np.random.default_rng(37).integers(0,256,current.shape,dtype=np.uint8),
                                   'removed':removed,'wrong_color':current[:,:,::-1].copy()}
                            for kind,negative in cases.items():
                                nd={}
                                try:
                                    track_translation(negative,calibration,pose,target,anchor,nd,max_distance=bound);accepted=True
                                except WristTargetError:accepted=False
                                negatives.append({'plan_id':directory.name,'iteration':observation.name,'kind':kind,'accepted':accepted,'diagnostics':nd})

                    except WristTargetError as exc:row.update(combined_accepted=False,error=str(exc))
                    row.update(compute_sec=time.monotonic()-t,diagnostics=diag)
                except (OSError,KeyError,ValueError) as exc:row['skipped']=str(exc)
                rows.append(row)
            if negative_checks:
                removed=cv2.inpaint(image,cv2.dilate(mask,np.ones((15,15),np.uint8)),5,cv2.INPAINT_TELEA)
                negative_images={'blank':np.full_like(image,100),'noise':np.random.default_rng(37).integers(0,256,image.shape,dtype=np.uint8),
                                 'removed':removed,'wrong_color':image[:,:,::-1].copy()}
                for kind,current in negative_images.items():
                    diag={}
                    try:
                        dense_translation(current,calibration,camera,obb,anchor,diag,bound);accepted=True
                    except WristTargetError:accepted=False
                    negatives.append({'plan_id':directory.name,'kind':kind,'accepted':accepted,'diagnostics':diag})
    valid=[r for r in rows if 'skipped' not in r]
    return {'scope':'saved_image_registration_only_not_closed_loop_success',
            'calibration_policy':'explicit_supplied_calibration_for_all_sequences; historical calibration not independently recoverable',
            'calibration_key':calibration_key(calibration),'urdf_sha256':fk.sha256,
            'eligible_plans':len({r['plan_id'] for r in valid}),'valid_frames':len(valid),'skipped_plans':skipped,
            'sparse_accepted':sum(r['sparse_accepted'] for r in valid),'combined_accepted':sum(r['combined_accepted'] for r in valid),
            'negative_cases':len(negatives),'negative_false_accepts':sum(r['accepted'] for r in negatives),
            'elapsed_sec':time.monotonic()-started,'rows':rows,'negatives':negatives}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--corpus',required=True,type=Path);parser.add_argument('--calibration',required=True,type=Path)
    parser.add_argument('--urdf',required=True,type=Path);parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--negative-checks',action='store_true');args=parser.parse_args()
    result=benchmark(args.corpus,json.loads(args.calibration.read_text()),args.urdf,args.negative_checks)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','negatives','skipped_plans')}))

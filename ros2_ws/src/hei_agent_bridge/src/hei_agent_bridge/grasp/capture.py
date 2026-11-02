"""Bounded wrist-image acquisition after measured joint settling; no motion commands."""
import asyncio
from collections import deque
import json
import time

import numpy as np
from .wrist import frame_quality


class CaptureRetry(RuntimeError):
    """Pose-dependent work must be discarded and reacquired, never executed."""


def delta(before,after):
    result={key:(np.asarray(after[key])-np.asarray(before[key])).tolist() for key in before if key in after}
    right=np.asarray(result['right'])
    result['max_right_joint']=f'a_right_joint{int(np.argmax(np.abs(right)))+1}'
    result['max_right_delta_rad']=float(np.max(np.abs(right)))
    return result


def scene_check(service,before,after):
    if not service.scene_matches(before,after) or abs(before['right_gripper']-after['right_gripper'])>.02:
        raise RuntimeError('Wrist scene/gripper changed: '+json.dumps(delta(before,after)))


def record(directory,rows,phase,**value):
    rows.append(dict(phase=phase,elapsed_monotonic=time.monotonic(),**value))
    (directory/'stability.json').write_text(json.dumps(rows,allow_nan=False))


async def acquire(service,directory,invocation,config,attempt,rows):
    """Require a continuous stable window, then a fresh stable image sequence."""
    history=deque();deadline=time.monotonic()+config['joint_timeout_sec']
    anchor=service.state(at_rest=False)
    while True:
        now=time.monotonic();state=service.state(at_rest=False);scene_check(service,anchor,state)
        history.append((now,state))
        # Keep the sample immediately before the window boundary as well.
        while len(history)>2 and history[1][0]<=now-config['joint_stable_sec']:history.popleft()
        joints=np.asarray([v['right'] for _,v in history]);span=np.ptp(joints,axis=0)
        sample_window=getattr(service.backend,'wrist_joint_window',None)
        feedback=sample_window(config['joint_stable_sec']) if sample_window else None
        if feedback:span=np.maximum(span,feedback['joint_span_rad'])
        record(directory,rows,'settling',attempt=attempt,state=state,joint_span_rad=span.tolist(),feedback_window=feedback)
        feedback_ready=not sample_window or feedback is not None
        if feedback_ready and len(history)>=3 and now-history[0][0]>=config['joint_stable_sec'] and np.max(span)<=config['joint_span_rad']:
            break
        if now>=deadline:
            raise CaptureRetry('Right arm did not settle: '+json.dumps(dict(joint_span_rad=span.tolist(),
                max_right_joint=f'a_right_joint{int(np.argmax(span))+1}',limit_rad=config['joint_span_rad'],fresh_feedback_window_available=feedback_ready)))
        await asyncio.sleep(.05)
    after,after_wall=time.monotonic(),time.time();before=state
    deadline=time.monotonic()+config['frame_timeout_sec']
    previous=None;last_stamp=after_wall;stable=0;quality=[];last_sync_error=''
    (directory/'capture_quality.json').write_text('[]')
    while True:
        captured=await service.cameras.capture({'camera':'right_wrist'},invocation)
        image=service.cameras._get(captured['observation_id'])
        state=service.state(at_rest=False);scene_check(service,before,state)
        changes=delta(before,state)
        record(directory,rows,'capture',attempt=attempt,state=state,delta=changes)
        if changes['max_right_delta_rad']>config['joint_span_rad']:
            raise CaptureRetry('Right arm moved during wrist capture: '+json.dumps(changes))
        if (not image['simulated'] and image.get('received',0)>after and image.get('stamp_sec',0)>last_stamp):
            if image.get('frame_id')!='right_wrist_camera_optical_frame':raise RuntimeError('Wrist image optical frame mismatch')
            sync=getattr(service.backend,'wrist_joint_state',None)
            alignment=sync(image['stamp_sec'],max_skew_sec=config['joint_sample_skew_sec'],
                           max_span_rad=config['joint_span_rad']) if sync else None
            # Production ROS must supply bracketing samples; never fall back to latest pose.
            if sync and alignment is None:
                last_sync_error='No bracketing joint feedback within configured image timestamp skew'
                record(directory,rows,'alignment_pending',attempt=attempt,image_stamp_sec=image['stamp_sec'],reason=last_sync_error)
            else:
                last_stamp=image['stamp_sec']
                previous,metrics=frame_quality(image['assets']['rgb'][1],previous)
                good=(metrics['sharpness']>=config['min_sharpness'] and metrics['frame_difference'] is not None and
                      metrics['frame_difference']<=config['max_frame_difference'])
                stable=stable+1 if good else 0
                quality.append(dict(stamp_sec=last_stamp,stable_count=stable,**metrics))
                (directory/'capture_quality.json').write_text(json.dumps(quality,allow_nan=False))
                (directory/'rgb.jpg').write_bytes(image['assets']['rgb'][1])
                if stable>=config['stable_frames']:
                    if alignment:
                        matched={**state,'right':alignment['right']}
                        changes=delta(matched,state)
                        if changes['max_right_delta_rad']>config['joint_span_rad']:
                            raise CaptureRetry('Image pose differs from current feedback: '+json.dumps(changes))
                    else:matched=state  # Offline/test backends have no ROS clock.
                    record(directory,rows,'accepted',attempt=attempt,state=matched,image_stamp_sec=last_stamp,
                           alignment=alignment or dict(method='offline_capture_window'))
                    return captured,image,matched
        if time.monotonic()>=deadline:
            if last_sync_error:raise CaptureRetry(last_sync_error)
            raise RuntimeError(f'No stable fresh wrist RGB after hover; inspect {directory}/capture_quality.json and rgb.jpg')
        await asyncio.sleep(.05)

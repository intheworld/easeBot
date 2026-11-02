#!/usr/bin/env python3
"""Operator direct joint-zero command; no Agent/IK/collision/scene gating."""
import argparse
import math
import time

VR_SPEEDS=(3.,3.,3.,1.8,2.5,2.5)


def zero_samples(start):
    if len(start)!=6 or not all(math.isfinite(x) for x in start):
        raise ValueError('Need six finite joint positions')
    # Quintic time scaling, maximum derivative 1.875. Bound adjacent commands
    # by the VR reset step of .015 rad and peak speed by the VR motor limits.
    duration=max(.5,1.875*max(abs(q)/v for q,v in zip(start,VR_SPEEDS)))
    count=max(2,math.ceil(1.875*max(map(abs,start))/.015))
    for i in range(count+1):
        u=i/count;s=10*u**3-15*u**4+6*u**5
        yield duration*u,[q*(1-s) for q in start]


def main():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side',choices=('right','left','both'),default='both')
    parser.add_argument('--namespace',default='/hei')
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args);node=rclpy.create_node('arm_zero_command')
    sides=('left','right') if args.side=='both' else (args.side,)
    ns='/'+args.namespace.strip('/') if args.namespace.strip('/') else ''
    samples={};clients={};subscriptions=[];handles=[];sends=[]
    def feedback(msg):
        for name,value in zip(msg.name,msg.position):samples[name]=(value,time.monotonic())
    def wait(future):
        rclpy.spin_until_future_complete(node,future)
        return future.result()
    try:
        for side in sides:
            subscriptions.append(node.create_subscription(JointState,f'{ns}/{side}_joint_states',feedback,qos_profile_sensor_data))
            clients[side]=ActionClient(node,FollowJointTrajectory,f'{ns}/{side}_arm_controller/follow_joint_trajectory')
        for client in clients.values():
            if not client.wait_for_server(timeout_sec=5):raise RuntimeError('Arm controller action server unavailable')
        names={s:[f'{"a_right" if s=="right" else "b_left"}_joint{i}' for i in range(1,7)] for s in sides}
        deadline=time.monotonic()+5
        while not all(n in samples and time.monotonic()-samples[n][1]<1 for group in names.values() for n in group):
            if time.monotonic()>deadline:raise RuntimeError('Joint positions unavailable for zero trajectory start')
            rclpy.spin_once(node,timeout_sec=.1)
        sends=[]
        for side in sides:
            goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=names[side]
            for elapsed,q in zero_samples([samples[n][0] for n in names[side]]):
                point=JointTrajectoryPoint(positions=q)
                point.time_from_start.sec,point.time_from_start.nanosec=divmod(round(elapsed*1e9),1000000000)
                goal.trajectory.points.append(point)
            sends.append(clients[side].send_goal_async(goal))
        for future in sends:
            handle=wait(future);handles.append(handle)
        if not all(h.accepted for h in handles):raise RuntimeError('Controller rejected zero command')
        results=[h.get_result_async() for h in handles]
        for side,future in zip(sides,results):
            result=wait(future)
            if result.status!=4 or result.result.error_code!=0:
                raise RuntimeError(f'{side}: controller status={result.status}, {result.result.error_string}')
            print(f'{side}: zero command completed')
    finally:
        # Cancel outstanding goals on interruption/failure, never silently leave
        # the second arm running after the command process exits.
        for future in sends:
            if not future.done():
                try:wait(future)
                except Exception:continue
            handle=future.result()
            if handle is not None and handle not in handles:handles.append(handle)
        for handle in handles:
            if handle.accepted:
                try:wait(handle.cancel_goal_async())
                except Exception:pass
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()

"""Shared jerk-bounded C2, monotone quintic trajectory for ROS playback and collision sampling.

A global minimum-jerk phase supplies smooth intermediate derivatives. Joints
whose spline would overshoot use monotone local derivatives. Bezier polygons
bound position, speed, acceleration and jerk. Collision and ROS use one curve.
"""
import math

import numpy as np


class JointTrajectory:
    def __init__(self, start, waypoints, speed):
        q = np.asarray([start, *waypoints], dtype=float)
        if q.ndim != 2 or q.shape[1] != 6 or len(q) < 2 or not np.isfinite(q).all():
            raise ValueError('Trajectory needs a start and finite six-joint waypoints')
        if not np.isfinite(speed) or not 0 < speed <= 1.:
            raise ValueError('Joint speed must be in (0, 1] rad/s')
        # Preserve the existing 60% speed allowance. Acceleration reaches that
        # speed no faster than 0.5 seconds, independently of waypoint density.
        self.speed_limit, self.acceleration_limit = speed*.6, speed*1.2
        self.jerk_limit = speed*12.
        candidate = self.__dict__.copy() if self._smooth_phase(q) else None
        delta = np.diff(q, axis=0)
        lengths = np.max(np.abs(delta), axis=1)
        distance = np.r_[0., np.cumsum(lengths)]
        total = float(distance[-1])
        peak = min(self.speed_limit, math.sqrt(total*self.acceleration_limit))
        ramp_distance = peak**2/(2*self.acceleration_limit)
        duration = total/peak+peak/self.acceleration_limit if peak else .1
        def arrival(x):
            if x <= ramp_distance:
                return math.sqrt(2*x/self.acceleration_limit)
            if x >= total-ramp_distance:
                return duration-math.sqrt(max(0., 2*(total-x)/self.acceleration_limit))
            return peak/self.acceleration_limit+(x-ramp_distance)/peak
        h = np.maximum(.1, np.diff([arrival(x) for x in distance]))
        # Refine each interval with damping, allowing conservative intervals to
        # shrink again. Startup ramps must not slow the entire dense path.
        for _ in range(128):
            v = np.zeros_like(q)
            slopes = delta/h[:, None]
            if len(q) > 2:
                same_direction = slopes[:-1]*slopes[1:] > 0
                v[1:-1] = np.where(same_direction,
                    np.sign(slopes[:-1])*np.minimum(np.abs(slopes[:-1]), np.abs(slopes[1:])), 0.)
            controls = self._control_points(q, v, h)
            velocity = 5*np.diff(controls, axis=1)/h[:, None, None]
            acceleration = 4*np.diff(velocity, axis=1)/h[:, None, None]
            jerk = 3*np.diff(acceleration,axis=1)/h[:,None,None]
            factors = np.maximum.reduce((
                self._bound(velocity)/self.speed_limit,
                np.sqrt(self._bound(acceleration)/self.acceleration_limit),
                np.cbrt(self._bound(jerk)/self.jerk_limit)))
            updated = np.maximum(.1, h*np.power(factors, .15))
            if np.max(np.abs(updated-h)) < 1e-7:
                break
            if _ < 127:
                h = updated
        # A final uniform stretch proves the bounds even if iteration has not
        # converged. It preserves the already checked geometric curve exactly.
        scale = max(1., float(np.max(factors)))
        jerk = 3*np.diff(acceleration,axis=1)/h[:,None,None]
        scale = max(scale,float(np.max(self._bound(jerk)) / self.jerk_limit)**(1/3))
        self.smoothing_method='local_monotone_quintic'
        self.local_joints=list(range(6))
        self.positions = q
        self.accelerations = np.zeros_like(q)
        self._controls = controls
        self.intervals = h*scale
        self.velocities = v/scale
        self.times = np.r_[0., np.cumsum(self.intervals)]
        self.duration = float(self.times[-1])
        if candidate is not None and candidate['duration'] <= self.duration:
            self.__dict__.update(candidate)

    def _smooth_phase(self, q):
        """Global quintic interpolating spline, accepted only with no overshoot.

        Normalize knot times using one minimum-jerk phase. A single polynomial
        straight move is therefore independent of how densely IK sampled it.
        Bezier derivative hulls certify limits and the same curve is used for
        collision checking and controller reconstruction. Joints with reversals
        or spline overshoot use local monotone derivatives on the same times.
        """
        from scipy.interpolate import make_interp_spline
        delta = np.diff(q,axis=0)
        distances = np.max(np.abs(delta),axis=1)
        if np.any(distances <= 1e-12):
            return False
        fractions = np.r_[0.,np.cumsum(distances)]/sum(distances)
        lo = np.zeros(len(q));hi = np.ones(len(q))
        for _ in range(50):
            u = (lo+hi)*.5
            value = u**3*(10+u*(-15+6*u))
            lo = np.where(value < fractions,u,lo)
            hi = np.where(value >= fractions,u,hi)
        u = (lo+hi)*.5;u[0]=0.;u[-1]=1.
        spline = make_interp_spline(u,q,k=5,bc_type=([(1,np.zeros(6)),(2,np.zeros(6))],
                                                   [(1,np.zeros(6)),(2,np.zeros(6))]))
        v=spline(u,1);a=spline(u,2);v[[0,-1]]=0.;a[[0,-1]]=0.
        h=np.diff(u);controls=self._control_points(q,v,h,a)
        derivatives=np.diff(controls,axis=1)
        local = np.any(derivatives*np.sign(delta)[:,None,:] < -1e-12,axis=(0,1))
        local |= np.any(np.where((delta==0)[:,None,:],np.abs(derivatives),0.) > 1e-12,axis=(0,1))
        if local.any():
            slopes=delta/h[:,None];local_v=np.zeros_like(q)
            same=slopes[:-1]*slopes[1:] > 0
            local_v[1:-1]=np.where(same,np.sign(slopes[:-1])*np.minimum(abs(slopes[:-1]),abs(slopes[1:])),0.)
            v[:,local]=local_v[:,local];a[:,local]=0.
            controls=self._control_points(q,v,h,a)
            derivatives=np.diff(controls,axis=1)
        velocity=5*derivatives/h[:,None,None]
        acceleration=4*np.diff(velocity,axis=1)/h[:,None,None]
        jerk=3*np.diff(acceleration,axis=1)/h[:,None,None]
        duration=max(.1,float(np.max(self._bound(velocity)))/self.speed_limit,
                     math.sqrt(float(np.max(self._bound(acceleration)))/self.acceleration_limit),
                     float(np.max(self._bound(jerk))/self.jerk_limit)**(1/3))
        self.positions=q;self.velocities=v/duration;self.accelerations=a/duration**2
        self._controls=controls;self.intervals=h*duration
        self.times=np.r_[0.,np.cumsum(self.intervals)];self.duration=float(self.times[-1])
        self.smoothing_method='phase_quintic'
        self.local_joints=np.flatnonzero(local).tolist()
        return True

    @staticmethod
    def _bound(points):
        # Subdivision tightens the convex-hull bound without losing its proof.
        # Raw acceleration control polygons overestimate short ramps severely.
        pieces = points[:, None, :, :]
        for _ in range(4):
            work = pieces
            left, right = [work[..., 0, :]], [work[..., -1, :]]
            while work.shape[-2] > 1:
                work = (work[..., :-1, :]+work[..., 1:, :])*.5
                left.append(work[..., 0, :])
                right.append(work[..., -1, :])
            pieces = np.concatenate((np.stack(left,axis=-2),np.stack(right[::-1],axis=-2)),axis=1)
        return np.max(np.abs(pieces),axis=(1,2,3))

    @staticmethod
    def _control_points(q, v, h, a=None):
        hv = h[:, None]/5
        ha = h[:,None]**2/20
        return np.stack((q[:-1], q[:-1]+hv*v[:-1], q[:-1]+2*hv*v[:-1]+(0 if a is None else ha*a[:-1]),
                         q[1:]-2*hv*v[1:]+(0 if a is None else ha*a[1:]), q[1:]-hv*v[1:], q[1:]), axis=1)

    def evaluate(self, segment, fraction):
        """Position/velocity/acceleration at a segment fraction in [0, 1]."""
        if not 0 <= fraction <= 1:
            raise ValueError('Invalid segment fraction')
        controls = self._controls[segment]
        h = self.intervals[segment]
        def bernstein(points):
            n = len(points)-1
            return sum(math.comb(n, i)*fraction**i*(1-fraction)**(n-i)*p
                       for i, p in enumerate(points))
        velocity = 5*np.diff(controls, axis=0)/h
        acceleration = 4*np.diff(velocity, axis=0)/h
        return bernstein(controls), bernstein(velocity), bernstein(acceleration)

    def samples(self, max_step=.005):
        """Derivative bounds guarantee <= max_step joint travel per sample gap."""
        if not np.isfinite(max_step) or not 0 < max_step <= .02:
            raise ValueError('Invalid sampling step')
        yield 0., -1, self.positions[0].copy()
        for i, controls in enumerate(self._controls):
            span_bound = float(np.max(np.abs(5*np.diff(controls, axis=0))))
            count = max(1, int(math.ceil(span_bound/max_step)))
            for fraction in np.linspace(0, 1, count+1)[1:]:
                yield float(self.times[i]+self.intervals[i]*fraction), i, self.evaluate(i, fraction)[0]

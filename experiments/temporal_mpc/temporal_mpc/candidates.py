"""At most two local sides and a waiting reference under one shared budget.

This is an ordered first-feasible policy, not topology/global search. Every
engine keeps the existing all-track/corridor/velocity/stop checks. Late work is
discarded; no scheduling or uncertainty certificate is claimed.
"""
from dataclasses import replace
import time
import numpy as np
from .contracts import ContractError, predict
from .dynamics import braking, rollout_zoh, rollout
from .frontend import Window
from .local_reference import LateralReference
from .realtime_qp import QPConfig, QPResult, RealtimeMPC


class CandidateMPC:
    def __init__(self, config=QPConfig()):
        if config.max_iterations<3:raise ValueError('shared iteration budget too small')
        self.config=config;self.times=config.times
        bounded=replace(config,solver_budget=config.solver_budget/3,
                        max_iterations=min(130,config.max_iterations//3),planning_buffer=.03)
        self.engines=[RealtimeMPC(bounded) for _ in range(3)]
        for engine,side in zip(self.engines[:2],(1.,-1.)):
            engine.lateral=LateralReference(forced_side=side)
        self.active=0;self.side=None

    @property
    def lateral(self):
        return self.engines[self.active].lateral

    def reset(self):
        for engine in self.engines:engine.reset()
        self.active=0;self.side=None

    def _order(self,x,epoch,snapshot,window):
        if self.side is not None:return [self.side,1-self.side,2]
        preferred=0
        if (isinstance(window,Window) and np.shape(window.reference)==(len(self.times),3)
                and np.isfinite(window.reference).all()):
            try:
                timeline=predict(snapshot,epoch,self.times)
                tangent=window.reference[-1,:2]-window.reference[0,:2]
                length=np.linalg.norm(tangent)
                if length>1e-5:
                    tangent/=length;normal=np.array([-tangent[1],tangent[0]])
                    ahead=[c[0] for c in timeline.centers if 0.<(c[0]-x[:2])@tangent<6.]
                    if ahead:
                        center=min(ahead,key=lambda c:(c-x[:2])@tangent)
                        preferred=0 if (x[:2]-center)@normal>=0. else 1
            except ContractError:pass  # Each engine still owns contract rejection.
        return [preferred,1-preferred,2]

    def solve(self,initial,epoch_ns,snapshot,window):
        started=time.perf_counter();cpu=time.process_time();cfg=self.config
        x=np.asarray(initial,float)
        if x.shape!=(6,) or not np.isfinite(x).all():
            self.reset();raise ContractError('invalid measured state; boundary watchdog must stop')
        x=x.copy()
        if abs(x[5])<=1e-8:x[5]=0.
        order=self._order(x,epoch_ns,snapshot,window)
        trace=[];results=[];iterations=0;solver_s=0.;names=('left','right','wait')

        def complete_trace(reason):
            recorded={item['candidate'] for item in trace}
            for index in order:
                if names[index] not in recorded:
                    trace.append(dict(candidate=names[index],attempted=False,reason=reason))

        def finish(result,index):
            complete_trace('first feasible accepted' if result.model_feasible and result.fallback_id is None
                           else 'not attempted before fallback')
            self.active=index
            result.elapsed_s=time.perf_counter()-started;result.cpu_s=time.process_time()-cpu
            result.iterations=iterations;result.solver_s=solver_s
            result.candidate_trace=tuple(trace);result.chosen_candidate=names[index]
            if result.elapsed_s>=cfg.cycle_budget or solver_s>cfg.solver_budget:
                return late_brake('portfolio cycle/solver deadline')
            return result

        def late_brake(reason):
            complete_trace(reason)
            controls=braking(x,2*cfg.nodes,cfg.period,(*cfg.acceleration,2.))
            states=(rollout_zoh if abs(x[5])<=1e-8 else rollout)(x,controls,cfg.period)
            self.reset()
            return QPResult('uncertified_brake',reason,states[1,3:].copy(),controls[0].copy(),
                            states,controls,False,'FollowPathMPPI',time.perf_counter()-started,
                            time.process_time()-cpu,'shared deadline',iterations,(),None,
                            solver_s,tuple(trace),None)

        for index in order:
            remaining=cfg.cycle_budget-(time.perf_counter()-started)
            solver_remaining=cfg.solver_budget-solver_s
            if remaining<=.008 or solver_remaining<=.0001:
                trace.append(dict(candidate=names[index],attempted=False,reason='shared budget exhausted'))
                break
            if index==2 and (not isinstance(window,Window) or window.epoch_ns!=epoch_ns
                    or window.frame!='map' or not window.plan_id
                    or np.shape(window.reference)!=(len(self.times),3) or not np.isfinite(window.reference).all()
                    or np.max(np.abs(window.reference[:,2]-x[2]))>1e-8):
                trace.append(dict(candidate='wait',attempted=False,reason='invalid static reference contract'))
                continue
            engine=self.engines[index]
            engine.config=replace(engine.config,cycle_budget=min(cfg.cycle_budget,remaining),
                                  solver_budget=min(cfg.solver_budget/3,solver_remaining))
            candidate=window
            if index==2:
                candidate=replace(window,reference=np.tile(x[:3],(len(self.times),1)))
            result=engine.solve(x,epoch_ns,snapshot,candidate)
            iterations+=result.iterations;solver_s+=result.solver_s
            trace.append(dict(candidate=names[index],attempted=True,status=result.status,reason=result.reason,
                              feasible=result.model_feasible,fallback=result.fallback_id is not None,
                              iterations=result.iterations,solver_s=result.solver_s,elapsed_s=result.elapsed_s,
                              mode=engine.lateral.mode,track=engine.lateral.track,side=engine.lateral.side,
                              constraint_min=result.constraint_min))
            results.append((index,result))
            if time.perf_counter()-started>=cfg.cycle_budget or solver_s>cfg.solver_budget:
                return late_brake('portfolio cycle/solver deadline')
            if result.model_feasible and result.fallback_id is None:
                if index<2:self.side=index
                return finish(result,index)
        if time.perf_counter()-started>=cfg.cycle_budget or solver_s>cfg.solver_budget:
            return late_brake('portfolio cycle/solver deadline')
        # A fresh checked previous sequence wins over a checked brake. The
        # native plugin independently revalidates again before any execution.
        for status in ('previous_feasible','brake','uncertified_brake'):
            for index,result in results:
                if result.status==status:return finish(result,index)
        return late_brake('portfolio no usable result')

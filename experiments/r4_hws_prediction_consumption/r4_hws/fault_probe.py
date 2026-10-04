"""Offline independent output-process probe; no command ROS topic is opened."""
from dataclasses import asdict
import multiprocessing as mp
import time
from .execution import CommandOffer, GuardVerdict, OutputArbiter


def _publisher(connection):
    arbiter = OutputArbiter((.3, 0., 0.))
    started = time.perf_counter_ns()
    connection.send(dict(ready=started))
    rows = []
    for k in range(10):
        target = started+k*50_000_000
        delay = (target-time.perf_counter_ns())*1e-9
        if delay > 0: time.sleep(delay)
        now = time.perf_counter_ns()
        if connection.poll():
            offer = connection.recv()
            arbiter.submit(offer)
        epoch = 1_000_000_000+k*50_000_000
        output = arbiter.tick(now, epoch, 1, 'static-1', 'current-1',
                              lambda command: GuardVerdict(True, 'simulated_clear', epoch, 'current-1'))
        arbiter.sent(output)
        rows.append(dict(steady_ns=now, **asdict(output)))
    connection.send(rows)
    connection.close()


def run_fault_probe():
    context = mp.get_context('spawn')
    parent, child = context.Pipe()
    publisher = context.Process(target=_publisher, args=(child,))
    publisher.start()
    child.close()
    if not parent.poll(5.):
        publisher.terminate(); publisher.join(1.)
        raise RuntimeError('output probe startup timeout')
    ready = parent.recv()['ready']
    parent.send(CommandOffer((.3, 0., 0.), ready, ready+75_000_000, 1, 'static-1', 'fault-probe', 'r4'))
    # Simulate a compute process that stops servicing input for 350 ms. The
    # separately spawned output owner has no dependency on this blocked loop.
    time.sleep(.35)
    if not parent.poll(5.):
        publisher.terminate(); publisher.join(1.)
        raise RuntimeError('output probe completion timeout')
    rows = parent.recv()
    parent.close()
    publisher.join(2.)
    if publisher.is_alive():
        publisher.terminate(); publisher.join(1.)
        raise RuntimeError('output probe shutdown timeout')
    gaps = [(b['steady_ns']-a['steady_ns'])*1e-9 for a, b in zip(rows, rows[1:])]
    return dict(kind='offline_output_process_probe', compute_block_s=.35,
                current_guard='simulated_clear', physical_braking_certified=False,
                max_output_gap_s=max(gaps), rows=rows)

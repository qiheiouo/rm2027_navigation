from dataclasses import replace
import math
import numpy as np
import pytest
from r4_hws.contracts import ContractError
from r4_hws.execution import CurrentGrid, OutputArbiter, CommandOffer, GuardVerdict
from r4_hws.follow import FollowMPC
from r4_hws.fault_probe import run_fault_probe


@pytest.mark.parametrize('yaw', [0., .3, math.pi/2])
def test_current_sweep_blocks_unknown_and_lethal(yaw):
    grid = np.zeros((60, 60), np.uint8)
    current = CurrentGrid(grid, .02, (-.6, -.6), 1_000_000_000, 'current')
    assert current.check((0., 0., yaw, 0., 0., 0.), 1_000_000_000, (.8, 0., 0.), 1_000_000_000).clear
    grid[30, 30] = 255
    current = CurrentGrid(grid, .02, (-.6, -.6), 1_000_000_000, 'current')
    blocked = current.check((0., 0., yaw, 0., 0., 0.), 1_000_000_000, (.8, 0., 0.), 1_000_000_000)
    assert not blocked.clear and blocked.reason == 'current_occupied_sweep'


def test_current_sweep_detects_between_endpoint_collision():
    # Grazing diagonal sweep: at neither endpoint does the tiny occupied cell
    # touch the rectangle; the translated hull still intersects it in between.
    grid = np.zeros((300, 300), np.uint8)
    origin, resolution = (-.6, -.6), .004
    grid[70, 240] = 254  # centre (.362, -.318), traversed between endpoints
    current = CurrentGrid(grid, resolution, origin, 1_000_000_000, 'current')
    pose = (0., 0., 0., 0., 0., 0.)
    assert current.check(pose, 1_000_000_000, (0., 0., 0.), 1_000_000_000).clear
    assert current.check((.04, .025, 0., 0., 0., 0.), 1_000_000_000, (0., 0., 0.), 1_000_000_000).clear
    assert current.check(pose, 1_000_000_000, (.8, .5, 0.), 1_000_000_000).clear is False


def test_current_guard_TTL_and_map_boundary():
    current = CurrentGrid(np.zeros((30, 30), np.uint8), .1, (-1.5, -1.5), 1_000_000_000, 'current')
    assert current.check((1.4, 0., 0., 0., 0., 0.), 1_000_000_000, (0., 0., 0.), 1_000_000_000).reason == 'current_map_boundary'
    assert current.check((0., 0., 0., 0., 0., 0.), 1_000_000_000, (0., 0., 0.), 1_400_000_001).reason == 'current_input_TTL'


def test_unique_output_lease_current_guard_and_generation():
    arbiter = OutputArbiter((.4, 0., 0.))
    offer = CommandOffer((.5, .1, 0.), 1_000_000_000, 1_075_000_000, 1, 'static', 'snap', 'r4')
    arbiter.submit(offer)
    tick = lambda steady, generation=1, clear=True: arbiter.tick(steady, steady, generation, 'static', 'current',
                        lambda command: GuardVerdict(clear, 'occupied' if not clear else 'clear', steady, 'current'))
    output = tick(1_000_000_000)
    assert output.source == 'r4' and output.command == pytest.approx((.45, .05, 0.))
    arbiter.sent(output)
    stopped = tick(1_050_000_000, clear=False)
    assert stopped.mode == 'stop' and stopped.command == pytest.approx((.4, 0., 0.)) and not stopped.brake_certified
    arbiter.sent(stopped)
    expired = tick(1_100_000_000)
    assert expired.mode == 'stop' and expired.command == pytest.approx((.35, 0., 0.))
    arbiter.submit(replace(offer, acquired_steady_ns=1_150_000_000, valid_until_steady_ns=1_225_000_000, source='mppi'))
    arbiter.sent(expired)
    changed = tick(1_150_000_000, generation=2)
    assert changed.mode == 'stop' and changed.reason == 'static_generation_changed'
    with pytest.raises(ContractError): arbiter.submit(offer)


def test_wait_keeps_offer_and_resumes_without_replan():
    arbiter = OutputArbiter()
    t = 1_000_000_000
    arbiter.submit(CommandOffer((0., 0., 0.), t, t+75_000_000, 1, 'static', 'waiting', 'r4'))
    guard = lambda cmd: GuardVerdict(True, 'clear', t, 'current')
    wait = arbiter.tick(t, t, 1, 'static', 'current', guard)
    assert wait.mode == 'wait'
    arbiter.sent(wait)
    t += 50_000_000
    arbiter.submit(CommandOffer((.05, 0., 0.), t, t+75_000_000, 1, 'static', 'released', 'r4'))
    resumed = arbiter.tick(t, t, 1, 'static', 'current', guard)
    assert resumed.mode == 'follow' and resumed.command[0] == .05


def test_cached_current_guard_cannot_authorize_output():
    arbiter = OutputArbiter()
    arbiter.submit(CommandOffer((.05, 0., 0.), 1_000_000_000, 1_075_000_000, 1, 'static', 'snap', 'r4'))
    output = arbiter.tick(1_000_000_000, 1_000_000_000, 1, 'static', 'current',
                           lambda cmd: GuardVerdict(True, 'clear', 999_999_999, 'current'))
    assert output.mode == 'stop' and 'cached_current_guard' in output.reason


def test_result_failure_clears_previous_command_lease(make_cycle):
    arbiter = OutputArbiter()
    snap = make_cycle()
    solved = FollowMPC().solve(snap)
    arbiter.offer_result(snap, solved)
    assert arbiter.offer is not None
    arbiter.offer_result(snap, replace(solved, status='stop', reason='injected_fault'))
    assert arbiter.offer is None
    with pytest.raises(ContractError): arbiter.offer_result(snap, replace(solved, snapshot_digest='another_cycle'))


def test_output_owner_survives_blocked_compute_process():
    evidence = run_fault_probe()
    rows = evidence['rows']
    assert len(rows) == 10
    assert any(r['source'] == 'r4' for r in rows)
    assert all(r['mode'] == 'stop' for r in rows if r['steady_ns']-rows[0]['steady_ns'] >= 100_000_000)
    assert rows[-1]['command'] == [0., 0., 0.] or tuple(rows[-1]['command']) == (0., 0., 0.)
    assert evidence['physical_braking_certified'] is False

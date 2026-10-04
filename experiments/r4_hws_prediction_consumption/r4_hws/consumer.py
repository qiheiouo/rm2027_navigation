"""Synchronous offline IO boundary: acquire once -> solve -> leased offer.

IO preparation and publisher tick are external. This is a porting seam for the
future Nav2 boundary, not a ROS worker or an independent safety certificate.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
from .contracts import ContractError, ReceiptOrder, freeze_cycle
from .execution import OutputArbiter
from .follow import FollowMPC


@dataclass(frozen=True)
class CycleOutcome:
    status: str
    reason: str
    snapshot: object = None
    solve: object = None


class CycleConsumer:
    def __init__(self, arbiter=None):
        self.arbiter = OutputArbiter() if arbiter is None else arbiter
        self.mpc = FollowMPC()
        self.order = ReceiptOrder()
        self.last_receipt = None
        self.last_epoch = self.last_cycle_id = self.last_state_ns = None

    def compute(self, **values):
        try:
            snapshot = freeze_cycle(**values)
            if (self.last_epoch is not None and (snapshot.epoch_ns <= self.last_epoch
                    or snapshot.cycle_id <= self.last_cycle_id or snapshot.state_ns < self.last_state_ns)):
                raise ContractError('control/state clock order')
            if snapshot.last_command != self.arbiter.last_command:
                raise ContractError('command memory differs from actually sent output')
            receipt_hash = hashlib.sha256(json.dumps(dict(source=snapshot.source_ns,
                                                        sequence=snapshot.source_sequence,
                                                        tracks=[asdict(t) for t in snapshot.tracks]),
                                                    sort_keys=True, allow_nan=False).encode()).hexdigest()
            receipt = (snapshot.source_ns, snapshot.source_sequence, receipt_hash)
            if self.last_receipt is not None and receipt[:2] == self.last_receipt[:2]:
                if receipt != self.last_receipt:
                    raise ContractError('same prediction epoch mutated')
                # A new control cycle may reuse the same unexpired receipt.
            else:
                self.order.accept(snapshot)
            self.last_receipt = receipt
            self.last_epoch, self.last_cycle_id, self.last_state_ns = snapshot.epoch_ns, snapshot.cycle_id, snapshot.state_ns
            result = self.mpc.solve(snapshot)
            self.arbiter.offer_result(snapshot, result)
            return CycleOutcome(result.status, result.reason, snapshot, result)
        except ContractError as error:
            self.mpc.reset()
            self.arbiter.offer = None
            self.order.usable = None
            self.last_receipt = None
            return CycleOutcome('stop', 'input_invalid:'+str(error))

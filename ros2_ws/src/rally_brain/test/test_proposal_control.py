from rally_brain.proposal_control import ProposalControl, REST_PROMPT


def test_response_expiry_context_and_one_time_consumption():
    now = [0.0]
    p = ProposalControl(lambda: now[0])
    context = ('session', 1, 2)
    assert p.propose(context)[0]
    assert p.item['prompt'] == REST_PROMPT
    assert not p.propose(context)[0]
    pid = p.item['proposal_id']
    assert not p.respond(context, 'wrong', 'accept')[0]
    now[0] = 20
    assert not p.respond(context, pid, 'accept')[0]
    assert p.item['status'] == 'expired'
    for decision in ('accept', 'reject'):
        assert p.propose(context)[0]
        pid = p.item['proposal_id']
        assert p.respond(context, pid, decision)[0]
        for repeated in ('accept', 'reject'):
            assert not p.respond(context, pid, repeated)[0]
    for changed, blocked in [(('other',1,2),False),(('session',5,2),False),
                             (('session',1,3),False),(context,True)]:
        p.propose(context)
        assert not p.respond(changed,p.item['proposal_id'],'accept',blocked)[0]
        assert p.item['status'] == 'invalidated'

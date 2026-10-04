"""Explicit authored fixtures; never dynamic truth consumed by control."""

def fixture_profile(name='legacy'):
    if name == 'legacy':
        return dict(name=name, goal=[5.6, 0.], width=160, height=120,
                    resolution=.05, origin=[-1., -3.], corridor_range=6.)
    if name == 'open_long':
        return dict(name=name, goal=[8.5, 0.], width=240, height=200,
                    resolution=.05, origin=[-1., -5.], corridor_range=10.)
    raise ValueError('unknown registered fixture profile')

import math
import pytest
from coppeliasim_contact.contracts import SceneConfig, MotionGuard, wrench_at_tip


def test_polygon_has_open_clearance_and_no_corner_gaps():
    c = SceneConfig()
    assert len(list(c.walls())) == 32
    for size, p, angle in c.walls():
        inner = math.hypot(p[0]-.4, p[1]) - size[0]/2
        assert inner == pytest.approx(c.hole_radius_m)
        assert size[1] > 2*(c.hole_radius_m+size[0]/2)*math.tan(math.pi/32)
        assert p[2]+size[2]/2 == pytest.approx(.2)


@pytest.mark.parametrize('kw', [dict(peg_radius_m=.006), dict(friction=-1),
    dict(soft_erp=2), dict(dynamics_dt_s=.003), dict(stop_depth_m=.03),
    dict(initial_tip_m=(.4,0,.19)), dict(motor_force_n=math.nan)])
def test_bad_scene_rejected(kw):
    with pytest.raises(ValueError):
        SceneConfig(**kw)


def test_rotated_wrench_and_tip_moment_sign():
    m = [0,-1,0,.4, 1,0,0,0, 0,0,1,.23]
    assert wrench_at_tip(m,[1,0,0],[0,0,0],[.4,0,.2],1) == pytest.approx((0,1,0,-.03,0,0))
    assert wrench_at_tip(m,[1,0,0],[0,0,0],[.4,0,.2],-1) == pytest.approx((0,-1,0,.03,0,0))
    with pytest.raises(ValueError):
        wrench_at_tip(m,[math.nan,0,0],[0,0,0],[.4,0,.2],1)


def test_commands_freshness_sign_invalid_sensor_and_overload_latch():
    g = MotionGuard(sign=1)
    args = ('robot_base',10.,10.,5.,(0,0,-.001),(0,0,0))
    assert g.accept(*args)
    v = lambda **kw: g.velocity(kw.get('now',10.1),kw.get('mono',5.1),(.4,0,.23),kw.get('f',(0,)*6),kw.get('valid',True))
    assert v() == (0,0,-.001)
    assert v(valid=False) == (0,0,0)
    assert v(now=11.) == (0,0,0)
    assert v(mono=6.) == (0,0,0)
    assert v(f=(46,0,0,0,0,0)) == (0,0,0)
    g.accept(*args)
    assert v() == (0,0,0)
    unarmed = MotionGuard()
    unarmed.accept(*args)
    assert unarmed.velocity(10.1,5.1,(.4,0,.23),(0,)*6,True) == (0,0,0)


@pytest.mark.parametrize('change', [dict(frame='wrong'), dict(stamp=12.), dict(stamp=1.),
    dict(linear=(math.nan,0,0)),dict(linear=(.08,.08,0)),dict(angular=(0,0,.001))])
def test_invalid_command_clears_previous(change):
    g = MotionGuard(sign=1)
    a = dict(frame='robot_base',stamp=10.,now=10.,received=5.,linear=(0,0,-.001),angular=(0,0,0))
    assert g.accept(**a)
    a.update(change)
    assert not g.accept(**a)
    assert g.velocity(10.,5.,(.4,0,.23),(0,)*6,True) == (0,0,0)

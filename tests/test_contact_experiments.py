from coppeliasim_contact.contracts import SceneConfig
from coppeliasim_contact.experiments import case_config, assess


def test_interventions_and_evidence_do_not_label_free_motion_as_jam():
    cfg=SceneConfig()
    assert case_config(cfg,'offset').hole_offset_x_m==.0004
    assert case_config(cfg,'sidewall').stop_depth_m is None
    assert case_config(cfg,'jam').stop_depth_m==.003
    assert case_config(cfg,'overload').motor_force_n==60
    assert assess([],cfg)['max_force_n'] is None
    r=dict(simulation_time_s=0.,pose=[.4,0,.196],reaction_at_tip=[0]*6,
           contacts=[],command_applied_mps=[0,0,-.001])
    result=assess([r,{**r,'simulation_time_s':.6}],cfg)
    assert result['depth_target_reached']
    assert not result['jam_candidate']
    contact={'point_world_m':[.4,0,.197]}
    r.update(reaction_at_tip=[0,0,15,0,0,0],contacts=[contact])
    result=assess([r,{**r,'simulation_time_s':.6}],cfg)
    assert result['jam_candidate'] and result['below_rim_contact_observed']
    assert not result['overload_observed']

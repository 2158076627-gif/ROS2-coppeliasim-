"""Opt-in real engine probe. User must load a generated NORMAL scene, stopped."""
import os
import pytest
from coppeliasim_contact.backend import CoppeliaBackend

@pytest.mark.skipif(os.environ.get('COPPELIA_LIVE')!='1',reason='CoppeliaSim live engine not requested')
def test_live_dynamic_peg_moves_and_force_sensor_is_valid():
    b=CoppeliaBackend(os.environ.get('COPPELIA_HOST','localhost'),int(os.environ.get('COPPELIA_PORT','23000')))
    try:
        b.start()
        initial=b.advance()
        for _ in range(25):
            b.command((0.,0.,-.001))
            sample=b.advance()
        assert sample['sensor_valid']
        assert .0001 < initial['pose'][2]-sample['pose'][2] < .001
        assert sample['contacts']==[]  # starts 30 mm above entrance
        assert sample['simulation_time_s']>initial['simulation_time_s']
    finally:
        b.close()

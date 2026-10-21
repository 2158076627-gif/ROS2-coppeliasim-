"""Opt-in real engine five-case acceptance, separate from free-motion smoke.

Requires pre-built sealed build bundles + raw calibration bundle on the same
filesystem as a dedicated stopped CoppeliaSim. An opt-in run FAILS if dependencies
or evidence are absent; it must not silently skip a requested physics test.
"""
import os
from pathlib import Path

import pytest

from coppeliasim_contact.acceptance_cli import main
from coppeliasim_contact.experiments import CASES


@pytest.mark.physics
@pytest.mark.skipif(os.environ.get('COPPELIA_ACCEPTANCE')!='1',reason='five-case real engine acceptance not requested')
@pytest.mark.parametrize('case',CASES)
def test_real_engine_five_case_acceptance(case,tmp_path):
    root=Path(os.environ['COPPELIA_ACCEPTANCE_ROOT'])
    args=['capture','--case',case,'--build',str(root/'build'/case),
          '--calibration',str(root/'calibration'),'--out',str(tmp_path/case),
          '--host',os.environ.get('COPPELIA_HOST','localhost'),
          '--port',os.environ.get('COPPELIA_PORT','23000')]
    exit_code=main(args)
    report=(tmp_path/case/'derived/report.json').read_text()
    assert exit_code==0,report

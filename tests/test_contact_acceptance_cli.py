"""Offline end-to-end CLI tests; fixtures are explicitly synthetic."""
import json
from pathlib import Path

from coppeliasim_contact import acceptance_cli as cli
from coppeliasim_contact.acceptance_live import dump,seal,sha256
from test_contact_acceptance import fixture


def build_fixture(directory,meta):
    """Sealed structural fixture, explicitly not a real simulator .ttt file."""
    directory.mkdir()
    (directory/'scene.ttt').write_bytes(b'UNIT TEST ONLY - NOT A COPPELIASIM SCENE')
    meta['scene_sha256']=sha256(directory/'scene.ttt')
    record={key:meta[key] for key in ('config','engine','scene_sha256','build_reload_verified','geometry_readback_verified')}
    record['complete']=True
    dump(directory/'build.json',record)
    dump(directory/'config.json',meta['config'])
    seal(directory)


def test_prepare_uses_existing_case_configs_and_adds_low_force_calibration(tmp_path):
    base=Path(__file__).parents[1]/'src/coppeliasim_contact/config/scene.json'
    assert cli.main(['prepare','--config',str(base),'--out',str(tmp_path/'cfg')])==0
    assert len(list((tmp_path/'cfg').glob('*.json')))==8  # five cases, two calibrations, limits
    assert json.loads((tmp_path/'cfg/jam.json').read_text())['stop_depth_m']==.003
    assert json.loads((tmp_path/'cfg/calibration_lateral.json').read_text())['motor_force_n']==5.


def test_report_writes_measured_displacement_force_csv_svg_and_three_state_result(tmp_path):
    cfg,meta,rows=fixture()
    build_fixture(tmp_path/'build',meta)
    dump(tmp_path/'manifest.json',meta)
    (tmp_path/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    seal(tmp_path)
    result=cli.report(tmp_path,require_calibration_files=False)  # unit fixture, not physical evidence
    assert result['status']=='PASS'
    assert (tmp_path/'derived/curves.csv').exists()
    assert 'max_sampled_geometric_overlap' in (tmp_path/'derived/curves.csv').read_text()
    assert '<svg' in (tmp_path/'derived/force_displacement.svg').read_text()
    (tmp_path/'samples.jsonl').write_text('')
    assert cli.report(tmp_path,require_calibration_files=False)['status']=='INSUFFICIENT'


def test_default_report_cannot_promote_fixture_without_raw_calibration_bundle(tmp_path):
    cfg,meta,rows=fixture()
    build_fixture(tmp_path/'build',meta)
    dump(tmp_path/'manifest.json',meta)
    (tmp_path/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    seal(tmp_path)
    assert cli.report(tmp_path)['status']=='INSUFFICIENT'


def test_preflight_writes_explicit_capabilities_without_starting_any_simulator(tmp_path):
    target=tmp_path/'environment.json'
    assert cli.main(['preflight','--out',str(target)])==0
    data=json.loads(target.read_text())
    assert isinstance(data['ros2_importable'],bool)
    assert data['engine_test_executed'] is False


def test_empty_or_incomplete_seal_is_not_valid_evidence(tmp_path):
    from coppeliasim_contact.acceptance_live import verify_seal
    dump(tmp_path/'artifacts.sha256.json',{})
    assert verify_seal(tmp_path)


def test_malformed_log_row_and_invalid_config_report_insufficient(tmp_path):
    cfg,meta,rows=fixture()
    meta['config']['peg_radius_m']=-1.
    dump(tmp_path/'manifest.json',meta)
    (tmp_path/'samples.jsonl').write_text('[]\n')
    seal(tmp_path)
    assert cli.report(tmp_path)['status']=='INSUFFICIENT'


def test_report_rejects_saved_scene_mismatched_to_run(tmp_path):
    cfg,meta,rows=fixture()
    build_fixture(tmp_path/'build',meta)
    meta['scene_sha256']='f'*64
    dump(tmp_path/'manifest.json',meta)
    (tmp_path/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    seal(tmp_path)
    assert cli.report(tmp_path,require_calibration_files=False)['status']=='INSUFFICIENT'


def test_calibration_trial_flag_never_bypasses_raw_calibration_to_pass(tmp_path):
    cfg,meta,rows=fixture()
    build_fixture(tmp_path/'build',meta)
    meta['calibration_trial']='axial'
    dump(tmp_path/'manifest.json',meta)
    (tmp_path/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    seal(tmp_path)
    assert cli.report(tmp_path)['status']=='INSUFFICIENT'


def test_top_level_null_manifest_is_insufficient_not_an_exception(tmp_path):
    (tmp_path/'manifest.json').write_text('null')
    (tmp_path/'samples.jsonl').write_text('[]\n')
    seal(tmp_path)
    assert cli.report(tmp_path)['status']=='INSUFFICIENT'


def test_ros_manifest_counts_cannot_replace_raw_topic_messages(tmp_path):
    from coppeliasim_contact.acceptance import ROS_TOPICS
    cfg,meta,rows=fixture()
    build_fixture(tmp_path/'build',meta)
    meta.update(mode='ros',ros_topics={t:1 for t in ROS_TOPICS})
    dump(tmp_path/'manifest.json',meta)
    (tmp_path/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    seal(tmp_path)
    assert cli.report(tmp_path,require_calibration_files=False)['status']=='INSUFFICIENT'

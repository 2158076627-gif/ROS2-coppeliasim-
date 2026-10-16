"""Static install/entry point contracts; no ROS installation required."""
import ast
from pathlib import Path
import runpy
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]

class PackagingTests(unittest.TestCase):
    def test_all_packages_are_installable_and_entrypoints_exist(self):
        for package in sorted((ROOT / 'src').iterdir()):
            with self.subTest(package=package.name):
                self.assertTrue((package / 'package.xml').is_file())
                self.assertTrue((package / 'resource' / package.name).is_file())
                self.assertEqual(ET.parse(package / 'package.xml').findtext('name'), package.name)
                with patch('setuptools.setup') as setup:
                    runpy.run_path(str(package / 'setup.py'))
                for entry in setup.call_args.kwargs.get('entry_points', {}).get('console_scripts', []):
                    module, func = entry.split('=')[1].strip().split(':')
                    path = package.joinpath(*module.split('.')).with_suffix('.py')
                    self.assertTrue(path.is_file(), entry)
                    tree = ast.parse(path.read_text())
                    self.assertIn(func, [n.name for n in tree.body if isinstance(n, ast.FunctionDef)])

    def test_launch_references_only_present_executables(self):
        launch = ROOT / 'src/axis_hole_bringup/launch/assembly_agent_demo.launch.py'
        tree = ast.parse(launch.read_text())
        definitions = next(n.value for n in ast.walk(tree) if isinstance(n, ast.Assign)
                           and any(isinstance(t, ast.Name) and t.id == 'definitions' for t in n.targets))
        for package, executable in ast.literal_eval(definitions):
            with patch('setuptools.setup') as setup:
                runpy.run_path(str(ROOT / 'src' / package / 'setup.py'))
            entries = setup.call_args.kwargs['entry_points']['console_scripts']
            self.assertIn(executable, [entry.split('=')[0].strip() for entry in entries])

    def test_declared_install_files_are_present(self):
        import os
        old_cwd = Path.cwd()
        try:
            for package in sorted((ROOT / 'src').iterdir()):
                os.chdir(package)
                with patch('setuptools.setup') as setup:
                    runpy.run_path(str(package / 'setup.py'))
                data = setup.call_args.kwargs['data_files']
                self.assertTrue(any(destination.endswith('/launch') and files
                                    for destination, files in data))
                self.assertTrue(any(destination.endswith('/config') and files
                                    for destination, files in data))
                for _, files in data:
                    for name in files:
                        self.assertTrue(Path(name).is_file(), name)
        finally:
            os.chdir(old_cwd)

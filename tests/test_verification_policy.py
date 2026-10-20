"""Metadata-only policy tests: empty predictions are distinct from broken interfaces."""
import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('observe_ros', ROOT / 'scripts/observe_ros.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class VerificationPolicyTests(unittest.TestCase):
    def verifier(self, output, allow_empty=True):
        topics = {t: dict(count=1) for t in ('/semantic/input/rgb', '/semantic/input/depth',
            '/semantic/image_annotated', '/semantic/label_image', '/semantic/instance_image',
            '/semantic/detector_status', '/mapData')}
        topics['/map'] = dict(known_cells=1)
        topics['/mapGraph'] = dict(poses=1)
        stats = dict(topics=topics, mask_nonzero_messages=0, instance_nonzero_messages=0,
                     semantic_cloud_nonempty=0, semantic_map_nonempty=0, tf_checks=0,
                     detector_frames_with_instances=0, max_tf_point_error_m=0.)
        return SimpleNamespace(output=Path(output), allow_empty=allow_empty, stats=stats,
                               latest_header=None, pipeline_tf_checks=1)

    def test_empty_predictions_are_a_valid_interface(self):
        with tempfile.TemporaryDirectory() as out:
            node = self.verifier(out)
            module.Observer.save(node)
            self.assertTrue(node.stats['success'])
            self.assertEqual(node.stats['segmentation_state'], 'normal_empty_predictions')
            self.assertEqual(node.stats['semantic_transform_validation'], 'not_exercised')

    def test_strict_policy_still_requires_nonempty_predictions(self):
        with tempfile.TemporaryDirectory() as out:
            node = self.verifier(out, allow_empty=False)
            module.Observer.save(node)
            self.assertFalse(node.stats['success'])

    def test_missing_output_or_bad_transform_is_a_failure(self):
        with tempfile.TemporaryDirectory() as out:
            node = self.verifier(out)
            del node.stats['topics']['/semantic/detector_status']
            module.Observer.save(node)
            self.assertFalse(node.stats['success'])
            node = self.verifier(out)
            node.stats['max_tf_point_error_m'] = .01
            module.Observer.save(node)
            self.assertFalse(node.stats['success'])


if __name__ == '__main__':
    unittest.main()

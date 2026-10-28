from types import SimpleNamespace
import unittest
from bpx_quadruped.sdk_compat import validate_sdk_version

class SdkCompatibilityTest(unittest.TestCase):
    def test_supported_versions(self):
        for version in ("1.0.8", "1.0.9"):
            self.assertEqual(version, validate_sdk_version(SimpleNamespace(__version__=version)))

    def test_unknown_versions_rejected(self):
        for version in (None, "1.0.10", "1.0.11", "1.0.12", "1.0.9-custom"):
            with self.assertRaises(RuntimeError):
                validate_sdk_version(SimpleNamespace(__version__=version))

"""Provider entry regressions in ROS image, mocked transport and runtime, no SDK."""
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import Mock, patch
from bpx_quadruped import main as p
from robonix_api import Ok, Err, Deferred


class LifecycleTest(unittest.TestCase):
    def setUp(self):
        p._runtime = p._config = p._publish_thread = None
        p._local_control = p._tf_broadcaster = None
        p._lifecycle_state = 'new'
        p._stop_publish = threading.Event()
        self.runtime = Mock(supports_twist=False)
        self.ros = Mock()
        for target, value in [('build_runtime', Mock(return_value=self.runtime)),
                              ('primitive', Mock()),
                              ('RosBackend', Mock(get=Mock(return_value=self.ros))),
                              ('_publish_loop', lambda: p._stop_publish.wait()),
                              ('_MOTION_CAPABILITY_ENABLED', False),
                              ('_POSTURE_CAPABILITY_ENABLED', False)]:
            patcher = patch.object(p, target, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.addCleanup(self.stop)

    def stop(self):
        p._stop_publish.set()
        thread = p._publish_thread
        if isinstance(thread, threading.Thread) and thread.ident:
            thread.join(2)

    def test_repeated_and_concurrent_init_retains_one_owner(self):
        with ThreadPoolExecutor(max_workers=8) as pool:
            results = list(pool.map(lambda _:p.on_init({}), range(24)))
        self.assertEqual(sum(isinstance(r, Ok) for r in results), 1)
        self.assertEqual(sum(isinstance(r, Deferred) for r in results), 23)
        p.build_runtime.assert_called_once()
        config = p._config
        self.assertIsInstance(p.on_activate(), Ok)
        thread = p._publish_thread
        self.assertIsInstance(p.on_init({'backend':'sdk'}), Deferred)
        self.assertIs(p._runtime, self.runtime)
        self.assertIs(p._config, config)
        self.assertIs(p._publish_thread, thread)
        self.assertEqual(p._lifecycle_state, 'active')
        self.runtime.shutdown.assert_not_called()

    def test_concurrent_activate_and_restart_have_one_worker(self):
        p.on_init({})
        with ThreadPoolExecutor(max_workers=8) as pool:
            results = list(pool.map(lambda _:p.on_activate(), range(24)))
        self.assertTrue(all(isinstance(r, Ok) for r in results))
        self.runtime.activate.assert_called_once()
        original = p._publish_thread
        self.assertTrue(original.is_alive())
        self.assertIsInstance(p.on_deactivate(), Ok)
        self.assertFalse(original.is_alive())
        self.assertIsNone(p._publish_thread)
        self.assertIsInstance(p.on_activate(), Ok)
        self.assertIsNot(original, p._publish_thread)
        self.assertIsInstance(p.on_shutdown(), Ok)
        self.assertIsInstance(p.on_shutdown(), Ok)
        self.runtime.shutdown.assert_called_once()
        self.assertIsInstance(p.on_activate(), Err)
        self.assertIsInstance(p.on_init({}), Deferred)

    def test_failed_init_closes_candidate_socket_runtime_and_ros(self):
        control = Mock()
        with patch.object(p, '_MOTION_CAPABILITY_ENABLED', True), \
             patch.object(p, 'LocalControl', Mock(return_value=control), create=True):
            p.primitive.create_publisher.side_effect = [None, RuntimeError('Atlas unavailable')]
            result = p.on_init({'enable_motion_service':True, 'allow_motion':True})
        self.assertIsInstance(result, Err)
        control.close.assert_called_once()
        self.runtime.shutdown.assert_called_once()
        self.ros.shutdown.assert_called_once()
        self.assertIsNone(p._runtime)
        self.assertIsNone(p._config)
        self.assertIsNone(p._local_control)
        self.assertIsInstance(p.on_init({}), Deferred)
        self.assertIsInstance(p.on_activate(), Err)

    def test_worker_stop_timeout_blocks_reactivation_but_stops_backend(self):
        p.on_init({})
        p._lifecycle_state = 'active'
        thread = Mock(ident=123)
        thread.is_alive.return_value = True
        p._publish_thread = thread
        self.assertIsInstance(p.on_deactivate(), Err)
        self.runtime.deactivate.assert_called_once()
        self.assertIsInstance(p.on_activate(), Err)
        self.runtime.activate.assert_not_called()

    def test_activate_deactivate_race_is_serialized(self):
        p.on_init({})
        entered, resume = threading.Event(), threading.Event()
        self.runtime.activate.side_effect = lambda: (entered.set(), resume.wait(2))
        with ThreadPoolExecutor(max_workers=2) as pool:
            activate = pool.submit(p.on_activate)
            self.assertTrue(entered.wait(1))
            deactivate = pool.submit(p.on_deactivate)
            self.assertFalse(deactivate.done())
            resume.set()
            self.assertIsInstance(activate.result(2), Ok)
            self.assertIsInstance(deactivate.result(2), Ok)
        self.assertEqual(p._lifecycle_state, 'inactive')
        self.assertIsNone(p._publish_thread)

    def test_activation_failure_cleans_up_without_starting_worker(self):
        p.on_init({})
        self.runtime.activate.side_effect = RuntimeError('activation failed')
        self.assertIsInstance(p.on_activate(), Err)
        self.runtime.deactivate.assert_called_once()
        self.assertIsNone(p._publish_thread)
        self.assertEqual(p._lifecycle_state, 'failed')


if __name__ == '__main__':
    unittest.main(verbosity=2)

"""Tests for check_links.py.

The contract of a checker is its exit code. A link checker that cannot fail is
worse than no checker, so this breaks its input and asserts exit 1, and it also
asserts the two behaviours the issue demands: a 404 is a hard failure, but a
transient network error is not (so CI does not go red on an unrelated flaky host).
"""

import importlib.util
import pathlib
import sys
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location(
    "check_links", ROOT / "tools" / "check_links.py")
check_links = importlib.util.module_from_spec(spec)
sys.modules["check_links"] = check_links
spec.loader.exec_module(check_links)


class _FakeResponse:
    def __init__(self, status):
        self.status = status

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


class LinksTest(unittest.TestCase):

    def setUp(self):
        check_links._CACHE.clear()

    def test_extracts_markdown_links(self):
        self.assertEqual(
            ["https://example.com/a"],
            list(check_links.extract_links("[text](https://example.com/a)")))

    def test_extracts_bare_urls(self):
        self.assertEqual(
            ["https://example.com/b"],
            list(check_links.extract_links("see https://example.com/b for details")))

    def test_ignores_an_anchor_only_link(self):
        self.assertEqual([], list(check_links.extract_links("[jump](#section)")))

    def test_404_is_a_hard_failure(self):
        """A link that is gone must fail the check."""
        with mock.patch("urllib.request.urlopen",
                        return_value=_FakeResponse(404)):
            rc = check_links.check_url("https://example.com/gone")
        self.assertEqual(1, rc)

    def test_network_error_is_not_fatal(self):
        """A transient network error must not fail CI (issue #159 requirement)."""
        with mock.patch("urllib.request.urlopen",
                        side_effect=OSError("connection refused")):
            rc = check_links.check_url("https://example.com/flaky")
        self.assertEqual(0, rc)

    def test_ignored_host_is_skipped(self):
        """Hosts on the ignore list are not checked at all."""
        with mock.patch("urllib.request.urlopen",
                        side_effect=AssertionError("should not be called")):
            rc = check_links.check_url("https://calendly.com/anything")
        self.assertEqual(0, rc)

    def test_ok_link_passes(self):
        with mock.patch("urllib.request.urlopen",
                        return_value=_FakeResponse(200)):
            rc = check_links.check_url("https://example.com/ok")
        self.assertEqual(0, rc)

    def test_head_refused_falls_back_to_get(self):
        """A server that answers HEAD with 405 or 404 but serves GET is not dead."""
        for head_status in (405, 404):
            check_links._CACHE.clear()
            with mock.patch("urllib.request.urlopen",
                            side_effect=[_FakeResponse(head_status), _FakeResponse(200)]) as m:
                rc = check_links.check_url("https://example.com/get-only")
            self.assertEqual(0, rc)
            self.assertEqual("GET", m.call_args_list[1][0][0].get_method())

    def test_each_url_is_checked_once_per_run(self):
        """The same link in several files costs one request."""
        with mock.patch("urllib.request.urlopen",
                        return_value=_FakeResponse(200)) as m:
            check_links.check_url("https://example.com/repeated")
            check_links.check_url("https://example.com/repeated")
        self.assertEqual(1, m.call_count)


if __name__ == "__main__":
    unittest.main()

"""Every tool in SCRIPTS must answer --help with its docstring and reject unknown flags."""
import contextlib
import importlib
import io
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

SCRIPTS = [
    "check_config_wiring",
    "check_docs_params",
    "check_node_member_wiring",
    "check_replay_wiring",
    "check_docs_test_count",
    "make_figures",
    "make_paper_figures",
]


class HelpFlag(unittest.TestCase):
    def test_help_prints_the_docstring_and_exits_zero(self):
        for name in SCRIPTS:
            with self.subTest(script=name):
                module = importlib.import_module(name)
                out = io.StringIO()
                with contextlib.redirect_stdout(out), self.assertRaises(SystemExit) as raised:
                    module.main(["--help"])
                self.assertEqual(raised.exception.code, 0)
                first_line = module.__doc__.strip().splitlines()[0]
                self.assertIn(first_line, out.getvalue())

    def test_unknown_flag_is_an_error_not_a_silent_run(self):
        for name in SCRIPTS:
            with self.subTest(script=name):
                module = importlib.import_module(name)
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                    module.main(["--bogus"])
                self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()

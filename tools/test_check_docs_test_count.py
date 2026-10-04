"""Tests for check_docs_test_count.py."""

import importlib.util
import pathlib
import sys
import tempfile
import textwrap
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location(
    "check_docs_test_count", ROOT / "tools" / "check_docs_test_count.py")
check = importlib.util.module_from_spec(spec)
sys.modules["check_docs_test_count"] = check
spec.loader.exec_module(check)


DOC = """\
- 7 individual test cases (3 in fusioncore_core, 2 in fusioncore_ros,
  2 in fusioncore_ublox). Of these, 1 are disabled on purpose.
  `colcon test-result --all` reports 10, which adds the 3 per-package CTest
  wrapper entries.
"""


class RegistrationTest(unittest.TestCase):

    def test_reads_all_three_registration_forms(self):
        cmake = textwrap.dedent("""\
            ament_add_gtest(test_ukf tests/test_ukf.cpp)
            ament_add_gtest(test_gnss tests/test_gnss.cpp TIMEOUT 600)
              add_launch_test(tests/test_from_ll_service.py TIMEOUT 180)
            ament_add_pytest_test(test_bridge_math test/test_bridge_math.py)
            """)
        self.assertEqual(
            ["tests/test_ukf.cpp", "tests/test_gnss.cpp",
             "tests/test_from_ll_service.py", "test/test_bridge_math.py"],
            check.registered_tests(cmake))

    def test_a_commented_out_registration_is_not_counted(self):
        self.assertEqual([], check.registered_tests(
            "# ament_add_gtest(test_old tests/test_old.cpp)\n"))


class CaseCountTest(unittest.TestCase):

    def test_counts_test_and_test_f_and_separates_disabled(self):
        src = textwrap.dedent("""\
            TEST(Ukf, Predicts) {}
            TEST_F(Fixture, Updates) {}
            TEST(Ukf, DISABLED_AcceptanceCriterion) {}
            """)
        self.assertEqual((3, 1), check.count_cases(pathlib.Path("t.cpp"), src))

    def test_ignores_line_and_block_comments(self):
        src = textwrap.dedent("""\
            // TEST(Ukf, InALineComment) {}
            /*
            TEST(Ukf, InABlockComment) {}
            */
            TEST(Ukf, Real) {}
            """)
        self.assertEqual((1, 0), check.count_cases(pathlib.Path("t.cpp"), src))

    def test_counts_python_cases_at_any_indent(self):
        src = textwrap.dedent("""\
            def test_module_level():
                pass

            class T(unittest.TestCase):
                def test_method(self, proc_output):
                    pass

                def helper(self):
                    pass
            """)
        self.assertEqual((2, 0), check.count_cases(pathlib.Path("t.py"), src))

    def test_refuses_a_parameterized_test_instead_of_guessing(self):
        with self.assertRaises(check.Uncountable):
            check.count_cases(ROOT / "t.cpp", "TEST_P(Suite, Case) {}\n")


class TallyTest(unittest.TestCase):

    def make_tree(self, root):
        files = {
            "fusioncore_core/CMakeLists.txt":
                "ament_add_gtest(test_a tests/test_a.cpp)\n"
                "ament_add_gtest(test_b tests/test_b.cpp TIMEOUT 300)\n",
            "fusioncore_core/tests/test_a.cpp":
                "TEST(A, One) {}\nTEST(A, DISABLED_Two) {}\n",
            "fusioncore_core/tests/test_b.cpp": "TEST(B, One) {}\n",
            # Present on disk but never registered, so it must not count.
            "fusioncore_core/tests/test_unbuilt.cpp": "TEST(U, One) {}\n",
            "fusioncore_ros/CMakeLists.txt":
                "add_launch_test(tests/test_launch.py TIMEOUT 180)\n",
            "fusioncore_ros/tests/test_launch.py":
                "class T:\n    def test_x(self):\n        pass\n"
                "    def test_y(self):\n        pass\n",
            "fusioncore_ublox/CMakeLists.txt":
                "ament_add_pytest_test(test_m test/test_m.py)\n",
            "fusioncore_ublox/test/test_m.py":
                "def test_a():\n    pass\ndef test_b():\n    pass\n",
        }
        for rel, text in files.items():
            path = root / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)

    def test_counts_registered_files_only(self):
        with tempfile.TemporaryDirectory() as d:
            root = pathlib.Path(d)
            self.make_tree(root)
            counts = check.tally(root)
        self.assertEqual({"cases": 3, "disabled": 1, "wrappers": 2},
                         counts["fusioncore_core"])
        self.assertEqual({"cases": 2, "disabled": 0, "wrappers": 1},
                         counts["fusioncore_ros"])
        self.assertEqual({"cases": 2, "disabled": 0, "wrappers": 1},
                         counts["fusioncore_ublox"])
        want = check.expected(counts)
        self.assertEqual((7, 3, 2, 2), want["cases"])
        self.assertEqual((1,), want["disabled"])
        self.assertEqual((11, 4), want["total"])


class DocumentedTest(unittest.TestCase):

    def test_reads_the_wrapped_sentence(self):
        found, missing = check.documented(DOC)
        self.assertEqual([], missing)
        self.assertEqual((7, 3, 2, 2), found["cases"])
        self.assertEqual((1,), found["disabled"])
        self.assertEqual((10, 3), found["total"])

    def test_a_missing_sentence_is_an_error_not_a_pass(self):
        """If someone rewords the line, the check must say so rather than go quiet."""
        found, missing = check.documented("No tally here.")
        self.assertEqual(3, len(missing))

    def test_current_doc_matches_the_source(self):
        """The real tree. Adding or removing a test without updating
        docs/reference/architecture.md fails here."""
        found, missing = check.documented(check.DOC.read_text())
        self.assertEqual([], missing)
        self.assertEqual(check.expected(check.tally()), found)


if __name__ == "__main__":
    unittest.main()

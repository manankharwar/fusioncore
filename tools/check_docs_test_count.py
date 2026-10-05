#!/usr/bin/env python3
"""Check that the test tally in docs/reference/architecture.md matches the source.

The tally in the Status section was wrong four times in a row (#136, #140, #154):
114, then 272, then 266, while main had already moved to 294. Every value was right
on the day someone counted it by hand and stale a few weeks later, because nothing
compared the sentence with the tests it describes. This does.

The counts come from the source, the same way a careful person counts them:

  * the test files a package actually registers in its CMakeLists.txt
    (ament_add_gtest, add_launch_test, ament_add_pytest_test), so a file that
    exists but is never built does not count;
  * one case per TEST / TEST_F macro in a registered C++ file, and one per
    `def test_...` in a registered Python file;
  * one CTest wrapper entry per registration, which is what makes
    `colcon test-result --all` print more than the number of real cases;
  * DISABLED_ cases separately, since gtest reports them as skipped.

TEST_P and TYPED_TEST expand to a number of cases that the source alone does not
show. If one appears, this exits 1 and says so rather than printing a wrong figure.

Usage:
    python3 tools/check_docs_test_count.py
"""

import argparse
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
DOC = ROOT / "docs" / "reference" / "architecture.md"
PACKAGES = ("fusioncore_core", "fusioncore_ros", "fusioncore_ublox")

# ament_add_gtest(name file ...) and ament_add_pytest_test(name file ...) take a
# target name first; add_launch_test(file ...) takes the file directly.
REGISTER = re.compile(
    r"^\s*(?:ament_add_gtest|ament_add_pytest_test)\(\s*\w+\s+([\w/.]+)"
    r"|^\s*add_launch_test\(\s*([\w/.]+)", re.M)

CPP_CASE = re.compile(r"^\s*TEST(?:_F)?\s*\(\s*\w+\s*,\s*(\w+)\s*\)", re.M)
CPP_UNCOUNTABLE = re.compile(r"^\s*(TEST_P|TYPED_TEST|TYPED_TEST_P)\s*\(", re.M)
PY_CASE = re.compile(r"^\s*def (test\w*)\s*\(", re.M)
BLOCK_COMMENT = re.compile(r"/\*.*?\*/", re.S)

# The three figures the Status section quotes. Whitespace may include a line break
# because the sentence is wrapped.
DOC_CASES = re.compile(
    r"(\d+)\s+individual\s+test\s+cases\s+\((\d+)\s+in\s+fusioncore_core,\s+"
    r"(\d+)\s+in\s+fusioncore_ros,\s+(\d+)\s+in\s+fusioncore_ublox\)")
DOC_DISABLED = re.compile(r"(\d+)\s+are\s+disabled\s+on\s+purpose")
DOC_TOTAL = re.compile(
    r"reports (\d+),\s+which\s+adds\s+the\s+(\d+)\s+per-package\s+CTest\s+"
    r"wrapper\s+entries")


class Uncountable(Exception):
    pass


def registered_tests(cmake_text):
    return [a or b for a, b in REGISTER.findall(cmake_text)]


def count_cases(path, text):
    """Return (cases, disabled) for one registered test file."""
    if path.suffix == ".py":
        names = PY_CASE.findall(text)
    else:
        text = BLOCK_COMMENT.sub("", text)
        bad = CPP_UNCOUNTABLE.findall(text)
        if bad:
            raise Uncountable(
                f"{path.relative_to(ROOT)} uses {bad[0]}, whose case count the "
                "source does not show; extend this checker before documenting it")
        names = CPP_CASE.findall(text)
    return len(names), sum(1 for n in names if n.startswith("DISABLED_"))


def tally(root=ROOT):
    """Per-package {cases, disabled, wrappers} counted from the source tree."""
    out = {}
    for pkg in PACKAGES:
        cmake = root / pkg / "CMakeLists.txt"
        files = registered_tests(cmake.read_text())
        cases = disabled = 0
        for rel in files:
            path = root / pkg / rel
            n, d = count_cases(path, path.read_text())
            cases += n
            disabled += d
        out[pkg] = {"cases": cases, "disabled": disabled, "wrappers": len(files)}
    return out


def documented(text):
    """The figures the doc quotes, or a list of the sentences it could not find."""
    found, missing = {}, []
    for name, regex in (("cases", DOC_CASES), ("disabled", DOC_DISABLED),
                        ("total", DOC_TOTAL)):
        hits = regex.findall(text)
        if len(hits) != 1:
            missing.append(f"{name}: expected one match of /{regex.pattern}/, "
                           f"found {len(hits)}")
            continue
        hit = hits[0] if isinstance(hits[0], tuple) else (hits[0],)
        found[name] = tuple(int(x) for x in hit)
    return found, missing


def expected(counts):
    cases = sum(c["cases"] for c in counts.values())
    wrappers = sum(c["wrappers"] for c in counts.values())
    return {
        "cases": (cases,) + tuple(counts[p]["cases"] for p in PACKAGES),
        "disabled": (sum(c["disabled"] for c in counts.values()),),
        "total": (cases + wrappers, wrappers),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.parse_args(argv)
    try:
        counts = tally()
    except Uncountable as e:
        print(e)
        return 1

    found, missing = documented(DOC.read_text())
    for m in missing:
        print(f"{DOC.relative_to(ROOT)}: {m}. The sentence moved or the regex is wrong")
    if missing:
        return 1

    want = expected(counts)
    drift = [k for k in want if found[k] != want[k]]
    for k in drift:
        print(f"{DOC.relative_to(ROOT)}: {k} says {found[k]}, the source says {want[k]}")

    for pkg in PACKAGES:
        c = counts[pkg]
        print(f"{pkg}: {c['cases']} cases ({c['disabled']} disabled), "
              f"{c['wrappers']} wrapper entries")
    print(f"documented tally {'drifted' if drift else 'matches the source'}")
    return 1 if drift else 0


if __name__ == "__main__":
    sys.exit(main())

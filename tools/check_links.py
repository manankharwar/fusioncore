#!/usr/bin/env python3
"""Check that every outbound link in the docs and ADOPTERS.md still resolves.

ADOPTERS.md is the first file a prospective user reads to decide whether this is
real, and nothing checks that any of its links still work. A dead link there costs
more than a dead link in a doc page. mkdocs.yml builds the site in CI, but a 404 on
an outbound link is not a build failure.

This checker walks every tracked .md file, pulls the URLs out of markdown links and
bare URLs, and does a HEAD request per link with a timeout.

The one rule that keeps this checker alive is that a transient network error is NOT
a failure. A flaky external host failing the build on an unrelated PR is how link
checkers get deleted six months later. So:

  * HTTP 404 or 410  -> the link is GONE. Hard failure, exit 1.
  * any other status -> the link resolves. Pass.
  * network error or timeout -> could not reach it. Reported, but NOT fatal.

Hosts on the ignore list are skipped entirely. They are the ones that legitimately
rate-limit or require auth (a calendar, a login wall), so a HEAD request to them is
not a meaningful signal.

Usage:
    python3 tools/check_links.py
"""

import argparse
import pathlib
import re
import subprocess
import sys
import urllib.error
import urllib.parse
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parent.parent

# A markdown link [text](url) or a bare URL in prose. An anchor-only link
# (#section) is not an outbound link and is ignored. Two capture groups, one per
# branch; extract_links() picks whichever matched.
LINK = re.compile(r"\[[^\]]*\]\((https?://[^)\s]+)\)|(?<![(\w])(https?://[^\s)\]>]+)")


def extract_links(text):
    """Yield the URLs in a markdown document, one per link, in order."""
    for m in LINK.finditer(text):
        url = m.group(1) if m.group(1) is not None else m.group(2)
        # Strip trailing markdown/HTML artifacts that are not part of the URL:
        # a closing brace from image syntax, a closing quote from an HTML attribute.
        yield url.rstrip('}"')

# Hosts that legitimately rate-limit or require auth. A HEAD request to them is not
# a meaningful signal, so they are skipped. Keep this list short.
IGNORE_HOSTS = {
    "calendly.com",
}

# A HEAD request can be answered with 405, or even 404, by servers that only serve
# GET. Before a link is called gone, it is retried with GET, so a server that
# refuses HEAD is not reported as a dead link.
TIMEOUT = 10

# One result per URL per run. The same profile link appears in several files.
_CACHE = {}


def _status(url, method):
    req = urllib.request.Request(url, method=method)
    try:
        with urllib.request.urlopen(req, timeout=TIMEOUT) as resp:
            return resp.status
    except urllib.error.HTTPError as e:
        return e.code


def check_url(url):
    """Return 1 if the link is gone (404/410), else 0.

    A network error or timeout is reported but NOT fatal, so CI does not go red on
    an unrelated flaky host. A 404 or 410 is a hard failure.
    """
    host = urllib.parse.urlparse(url).netloc
    if host in IGNORE_HOSTS:
        return 0
    if url in _CACHE:
        return _CACHE[url]
    try:
        status = _status(url, "HEAD")
        if status in (404, 405, 410):
            status = _status(url, "GET")
    except (OSError, urllib.error.URLError):
        print(f"  could not reach {url} (network error, not counted as a failure)")
        _CACHE[url] = 0
        return 0
    result = 0
    if status in (404, 410):
        print(f"  GONE ({status}): {url}")
        result = 1
    _CACHE[url] = result
    return result


def tracked_docs():
    out = subprocess.run(["git", "ls-files", "*.md"], cwd=ROOT,
                         capture_output=True, text=True).stdout.split()
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.parse_args(argv)

    docs = tracked_docs()
    if not docs:
        print("no tracked .md files found, the git ls-files call is wrong")
        return 1

    failures = 0
    checked = 0
    for d in docs:
        try:
            text = (ROOT / d).read_text()
        except OSError:
            continue
        for url in sorted(set(extract_links(text))):
            checked += 1
            failures += check_url(url)

    print(f"checked {checked} links across {len(docs)} files")
    if failures:
        print(f"{failures} dead link(s) found")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

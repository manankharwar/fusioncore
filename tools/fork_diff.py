#!/usr/bin/env python3
"""Which forks have diverged, so their work can be found instead of lost.

Someone who forks a repo and then commits to it has hit something. Usually they
fix it locally and never say a word, so the fix dies in the fork and the next
person hits the same wall.

That is not hypothetical here. gnss_doppler_bridge.cpp could not compile against
real ublox_msgs for months, because NavPVT has no header and the target sits
behind find_package(ublox_msgs QUIET) so it was silently skipped everywhere. It
was found by running this check and reading a fork, not from a bug report.

Usage:
    python3 tools/fork_diff.py                      # public API, 60 req/hr
    GITHUB_TOKEN=ghp_... python3 tools/fork_diff.py # authenticated, 5000/hr

Prints only forks that are AHEAD, newest first. Everything else is noise.
"""

import json
import os
import sys
import urllib.error
import urllib.request

REPO = os.environ.get("FORK_DIFF_REPO", "manankharwar/fusioncore")
API = "https://api.github.com"


def get(path):
    req = urllib.request.Request(API + path, headers={"Accept": "application/vnd.github+json"})
    token = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
    if token:
        req.add_header("Authorization", "Bearer " + token)
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        if e.code == 403:
            sys.exit("rate limited. Set GITHUB_TOKEN to raise the limit to 5000/hr.")
        return None
    except Exception:
        return None


def main():
    forks = get(f"/repos/{REPO}/forks?per_page=100&sort=newest")
    if forks is None:
        sys.exit(f"could not list forks of {REPO}")

    print(f"{len(forks)} forks of {REPO}\n")
    ahead = []
    for f in forks:
        owner = f["owner"]["login"]
        branch = f.get("default_branch", "main")
        # Compare against OUR default branch, not theirs, so "ahead" means
        # commits they wrote and we do not have.
        cmp = get(f"/repos/{REPO}/compare/main...{owner}:{branch}")
        if not cmp:
            continue
        n = cmp.get("ahead_by", 0)
        if n:
            ahead.append((n, f["full_name"], f["owner"]["type"], cmp.get("commits", [])))

    if not ahead:
        print("No fork is ahead. Nothing to collect.")
        return 0

    ahead.sort(reverse=True)
    for n, name, typ, commits in ahead:
        flag = "  <- ORGANISATION" if typ == "Organization" else ""
        print(f"{n:>3} ahead  {name}{flag}")
        for c in commits[-5:]:
            a = c["commit"]["author"]
            print(f"           {a['date'][:10]}  {a['name']}")
            print(f"           {c['commit']['message'].splitlines()[0][:78]}")
        print()

    print("Read the diffs. A fork that is ahead is someone who hit something.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

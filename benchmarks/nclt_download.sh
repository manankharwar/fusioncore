#!/bin/bash
# Download NCLT sensor data for one or more sequences.
#
# Usage:
#   bash benchmarks/nclt_download.sh 2012-01-08
#   bash benchmarks/nclt_download.sh 2012-06-15 2012-08-20 2013-04-05
#   bash benchmarks/nclt_download.sh baseline     # the three benchmarked sequences
#   bash benchmarks/nclt_download.sh all          # all 12
#
# Data lands in $NCLT_DATA (default ~/nclt) as one directory per sequence:
#
#   ~/nclt/2012-06-15/ms25.csv, gps.csv, gps_rtk.csv, odometry_mu_100hz.csv, ...
#
# That is the layout tools/run_nclt.sh reads. benchmarks/run_one.sh instead wants
# the CSVs under "benchmarks/nclt/<date>/raw files/", so a symlink is created
# there pointing at the downloaded directory, and both harnesses then work.
#
# The data root is kept outside the repo on purpose: it is 681 MB for three
# sequences, it is not ours to redistribute, and the hourly local backup
# deliberately excludes it because it is re-downloadable from here.
#
# WHY THIS WAS REWRITTEN: the dataset moved to S3 and the old per-file URLs
#   http://robots.engin.umich.edu/nclt/<date>/sensor_data/<file>.csv.gz
# now return 404 for every file of every sequence. The script had been failing
# silently in the sense that it printed [FAIL] five times per sequence and still
# exited 0, so following the repo's own reproduction instructions produced an
# empty data directory and no error. Found 2026-10-08 while re-measuring the
# baseline, by which point the local copy was the only copy on this machine.
#
# Dataset: http://robots.engin.umich.edu/nclt/
# Citation: Carlevaris-Bianco et al., "University of Michigan North Campus
#   Long-Term Vision and Lidar Dataset", IJRR 2016.
# Licence: ODbL 1.0 (see the dataset page).
set -euo pipefail

S3="https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu/sensor_data"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA_ROOT="${NCLT_DATA:-$HOME/nclt}"

ALL_SEQUENCES=(
    2012-01-08 2012-02-04 2012-03-31 2012-05-11
    2012-06-15 2012-08-20 2012-09-28 2012-10-28
    2012-11-04 2012-12-01 2013-02-23 2013-04-05
)

# The three sequences benchmark_baseline.json actually gates on.
BASELINE_SEQUENCES=(2012-06-15 2012-08-20 2013-04-05)

# What the player needs. Checked after extraction, so a changed tarball layout
# fails loudly here instead of producing a confusing error 50 minutes into a run.
REQUIRED=(ms25.csv ms25_euler.csv gps.csv gps_rtk.csv odometry_mu_100hz.csv)

if [ $# -eq 0 ]; then
    echo "Usage: bash benchmarks/nclt_download.sh <date> [<date> ...]"
    echo "       bash benchmarks/nclt_download.sh baseline   # ${BASELINE_SEQUENCES[*]}"
    echo "       bash benchmarks/nclt_download.sh all"
    echo ""
    echo "Available sequences:"
    for s in "${ALL_SEQUENCES[@]}"; do echo "  $s"; done
    exit 1
fi

case "$1" in
    all)      SEQUENCES=("${ALL_SEQUENCES[@]}") ;;
    baseline) SEQUENCES=("${BASELINE_SEQUENCES[@]}") ;;
    *)        SEQUENCES=("$@") ;;
esac

command -v curl >/dev/null || { echo "ERROR: curl not found. sudo apt install curl"; exit 1; }

FAILED=()
for SEQ in "${SEQUENCES[@]}"; do
    echo ""
    echo "=== $SEQ ==="
    DEST="$DATA_ROOT/$SEQ"

    MISSING=0
    for f in "${REQUIRED[@]}"; do [ -f "$DEST/$f" ] || MISSING=1; done
    if [ "$MISSING" -eq 0 ]; then
        echo "  [skip] already complete at $DEST"
    else
        URL="$S3/${SEQ}_sen.tar.gz"
        TMP="$(mktemp -d)"
        trap 'rm -rf "$TMP"' EXIT
        echo "  Downloading ${SEQ}_sen.tar.gz ..."
        if ! curl -fL --retry 3 --retry-delay 5 --progress-bar -o "$TMP/sen.tar.gz" "$URL"; then
            echo "  [FAIL] download failed: $URL"
            FAILED+=("$SEQ")
            rm -rf "$TMP"; trap - EXIT; continue
        fi

        # The archive holds <date>/<files>, so extract one level above the
        # sequence directory and let tar create it.
        mkdir -p "$DATA_ROOT"
        echo "  Extracting ..."
        tar xzf "$TMP/sen.tar.gz" -C "$DATA_ROOT"
        rm -rf "$TMP"; trap - EXIT

        for f in "${REQUIRED[@]}"; do
            if [ ! -f "$DEST/$f" ]; then
                echo "  [FAIL] $f missing after extraction. The tarball layout may have"
                echo "         changed; expected $DEST/$f"
                FAILED+=("$SEQ")
                continue 2
            fi
        done
        echo "  [ok]   $(du -sh "$DEST" | cut -f1) in $DEST"
    fi

    # Compatibility link for benchmarks/run_one.sh, which reads "<seq>/raw files".
    LINK="$SCRIPT_DIR/nclt/$SEQ/raw files"
    mkdir -p "$SCRIPT_DIR/nclt/$SEQ"
    if [ -L "$LINK" ]; then
        :
    elif [ -d "$LINK" ] && [ -z "$(ls -A "$LINK")" ]; then
        rmdir "$LINK" && ln -s "$DEST" "$LINK"
    elif [ ! -e "$LINK" ]; then
        ln -s "$DEST" "$LINK"
    else
        echo "  [warn] $LINK exists and is not empty, leaving it alone"
    fi
done

echo ""
if [ ${#FAILED[@]} -gt 0 ]; then
    echo "FAILED: ${FAILED[*]}"
    echo "Check https://robots.engin.umich.edu/nclt/ for the current download layout."
    exit 1
fi

echo "Download complete. Run a benchmark:"
echo "  bash tools/run_nclt.sh 2012-06-15 ~/nclt/run1 1.0"
echo "  bash tools/rerun_baseline.sh            # all baseline sequences"

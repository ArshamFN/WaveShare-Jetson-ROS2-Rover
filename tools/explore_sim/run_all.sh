#!/bin/bash
# Re-runs every comparison in the Session 016 log and prints the tables.
# About an hour on two cores; the results land in results/.
set -euo pipefail
cd "$(dirname "$0")"
for m in s015 oct7; do
  python3 compare.py "$m" A A2 B C D rover --starts 10
done
for m in s015 oct7; do
  python3 compare.py "$m" D rover --starts 30
done
python3 summarize.py

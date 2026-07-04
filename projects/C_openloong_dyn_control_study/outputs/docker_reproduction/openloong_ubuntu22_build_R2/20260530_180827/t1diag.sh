#!/usr/bin/env bash
echo "net: $(getent hosts archive.ubuntu.com >/dev/null 2>&1 && echo dns_ok || echo dns_fail)"
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq 2>&1 | tail -2
apt-get install -y -qq xvfb 2>&1 | tail -3
echo "xvfb_bin: $(command -v Xvfb || echo MISSING) | xvfb-run: $(command -v xvfb-run || echo MISSING)"

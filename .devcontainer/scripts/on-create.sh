#!/bin/bash

set -ue

sudo chown vscode:vscode -R /workspace

git config --global --add safe.directory $(pwd)
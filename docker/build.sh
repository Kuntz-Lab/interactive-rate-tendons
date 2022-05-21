#!/bin/bash

set -e # fail on error
set -u # fail on undefined variable

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

cd "${SCRIPT_DIR}"

docker build \
  ./ \
  --file Dockerfile \
  --build-arg user="${USER}" \
  --build-arg user_id="$(id -u)" \
  --build-arg user_group_id="$(id -g)" \
  --build-arg user_shell="${SHELL}" \
  --build-arg user_home="${HOME}" \
  --tag tendon-dev

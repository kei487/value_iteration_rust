#!/bin/bash
# Source PetaLinux settings and execute the given command
set -e

if [ -f "${PETALINUX}/settings.sh" ]; then
    source "${PETALINUX}/settings.sh"
fi

exec "$@"

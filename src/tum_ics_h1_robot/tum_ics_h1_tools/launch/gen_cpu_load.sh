#!/bin/bash

if [ ! $# -eq 1 ]; then
    echo "ERROR: You need to provide one arg for the time in sec to generate cpu load."
    exit 1
fi

yes :|sh&sleep $1; kill $!

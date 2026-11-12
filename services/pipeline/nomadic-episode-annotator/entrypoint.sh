#!/bin/bash

set -eo pipefail

# One worker process: the job store's file lock admits a single owner.
# Loopback only: the API has no authentication and uploads with the station's Nomadic key.
# Its only caller, the data-collection-ui server, connects at 127.0.0.1.
exec uv run --no-sync uvicorn main:app --host 127.0.0.1 --port 3050

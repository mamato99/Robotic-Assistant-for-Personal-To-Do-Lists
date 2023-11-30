#!/bin/bash

TB_PKG=$( cd -- "$(dirname "${0}")" >/dev/null 2>&1 ; pwd -P )

cd "$TB_PKG/../flask_server"
python3 app.py

#!/bin/bash
. $IDF_PATH/export.sh
idf.py build flash monitor

#!/bin/bash
COLS=$1
ROWS=$2
./generate_sheet_music.py -c $COLS -r $ROWS  -o /dev/null -N

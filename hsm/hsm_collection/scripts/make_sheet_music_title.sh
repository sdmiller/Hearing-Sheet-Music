#!/bin/bash
HOW_MANY=$1
COLS=$2
ROWS=$3
for i in `seq 1 $HOW_MANY`; 
do ./generate_sheet_music.py -c $COLS -r $ROWS -t -o ../data/generated/without_acc/titled/c${COLS}r${ROWS}_${i}.png; done

#! /usr/bin/env bash
if [[ "$#" != 2 ]]; then
  echo "usage: ./set_ptu.sh p t"
  exit 2
fi

rosservice call /pitcam/set_pan_tilt -- $1 $2

#!/bin/bash
for ((i=0;i<$1;i++)); do
    rosservice call tello_$i/command land &
done

#!/bin/bash

cd ../scenes
echo `pwd`
../src/rt $1.lua && open $1.png
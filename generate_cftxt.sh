#!/bin/bash
cat .ccls | sed "s/%c //g" | sed "s/clang//g" | sed "s/%cpp //g" | sed "s/ -/\n-/g" | sed "s/-mcpu=.*//g" | sed "s/-mfloat-.*//g" | sed "s/-mfpu=.*//g" > compile_flags.txt

echo "
-march=aarch64
-xc++" >> compile_flags.txt

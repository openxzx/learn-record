#!/usr/bin/env python3

import sys

sys.path.append('..')
print(sys.path)

import subdir1.subdir1

subdir1.subdir1.test()

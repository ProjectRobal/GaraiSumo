#!/bin/bash

idf.py build

python3 ota/ota.py 192.168.2.204/ota ./build Garai
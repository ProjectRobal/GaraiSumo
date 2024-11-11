#!/bin/bash

idf.py build

python3 ota/ota.py 192.168.2.205/ota ./build Garai
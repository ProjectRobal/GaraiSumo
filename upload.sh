#!/bin/bash

idf.py build

python3 ota/ota.py 192.168.4.1/ota ./build Garai
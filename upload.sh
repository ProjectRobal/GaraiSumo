#!/bin/bash

idf.py build

python3 ota/ota.py 192.168.50.234/ota ./build Garai